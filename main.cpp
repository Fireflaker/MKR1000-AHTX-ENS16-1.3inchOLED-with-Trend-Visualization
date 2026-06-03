// COMPLETE FINAL CODE - FIXED: CO2/TVOC less sensitive thresholds

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_SleepyDog.h>
#include <Adafruit_FreeTouch.h>
#include <DFRobot_ENS160.h>
#include <TimeLib.h>
#include <U8g2lib.h>
#include <math.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Adafruit_AHTX0 aht;
DFRobot_ENS160_I2C ens160(&Wire, 0x53);
Adafruit_FreeTouch touch_rate_pad = Adafruit_FreeTouch(A5, OVERSAMPLE_4, RESISTOR_100K, FREQ_MODE_NONE);
Adafruit_FreeTouch touch_window_pad = Adafruit_FreeTouch(A3, OVERSAMPLE_4, RESISTOR_100K, FREQ_MODE_NONE);

#define WDT_TIMEOUT_MS 8000
#define FORCED_RESET_MS (6UL * 60UL * 60UL * 1000UL)
#define SERIAL_WAIT_MAX_MS 1500
#define SENSOR_RETRY_MS 5000
#define BUZZER_PIN_A 2
#define BUZZER_PIN_B 5
#define ALERT_RING_MAX_MS 5000
#define ALERT_RING_COOLDOWN_MS 30000
#define TOUCH_DELTA_RATE 80
#define TOUCH_DELTA_WINDOW 80
#define TOUCH_TOGGLE_DEBOUNCE_MS 350
#define RECENT_SAMPLE_CAPACITY 120
#define COARSE_BIN_MS 10000UL
#define COARSE_BIN_CAPACITY 120

const uint16_t UPDATE_INTERVAL_OPTIONS_MS[] = {500, 200, 1000, 10000};
const char* UPDATE_RATE_LABELS[] = {"2", "5", "1", "0.1"};
const uint8_t UPDATE_RATE_COUNT = 4;
const uint32_t AVG_WINDOW_OPTIONS_MS[] = {5000UL, 30000UL, 120000UL, 900000UL};
const char* AVG_WINDOW_LABELS[] = {"5s", "30s", "2m", "15m"};
const uint8_t AVG_WINDOW_COUNT = 4;

enum AlertType : uint8_t { ALERT_NONE = 0, ALERT_AQI = 1, ALERT_TVOC = 2, ALERT_CO2 = 3 };

struct PackedSample {
  uint32_t ts;
  int16_t t10;
  int16_t h10;
  int16_t aqi10;
  int16_t co2;
  int16_t tvoc;
};

struct CoarseBin {
  uint32_t start_ms;
  int16_t t10;
  int16_t h10;
  int16_t aqi10;
  int16_t co2;
  int16_t tvoc;
  uint16_t count_t;
  uint16_t count_h;
  uint16_t count;
};

PackedSample recent_samples[RECENT_SAMPLE_CAPACITY];
uint16_t recent_head = 0;
uint16_t recent_count = 0;

CoarseBin coarse_bins[COARSE_BIN_CAPACITY];
uint16_t coarse_head = 0;
uint16_t coarse_count = 0;
bool active_bin_init = false;
uint32_t active_bin_start_ms = 0;
int32_t active_sum_t10 = 0;
int32_t active_sum_h10 = 0;
int32_t active_sum_aqi10 = 0;
int32_t active_sum_co2 = 0;
int32_t active_sum_tvoc = 0;
uint16_t active_count_t = 0;
uint16_t active_count_h = 0;
uint16_t active_count = 0;

uint32_t sample_count_total = 0;
uint16_t sample_count_window = 0;

bool aht_ok = false;
bool ens_ok = false;
bool touch_rate_ok = false;
bool touch_rate_calibrated = false;
bool touch_rate_active = false;
bool touch_rate_prev = false;
uint16_t touch_rate_baseline = 0;
bool touch_window_ok = false;
bool touch_window_calibrated = false;
bool touch_window_active = false;
bool touch_window_prev = false;
uint16_t touch_window_baseline = 0;
uint8_t update_rate_idx = 0;
uint16_t current_update_interval_ms = UPDATE_INTERVAL_OPTIONS_MS[0];
uint8_t avg_window_idx = 1;
unsigned long boot_ms = 0;
unsigned long last_sensor_retry_ms = 0;
unsigned long last_rate_toggle_ms = 0;
unsigned long last_window_toggle_ms = 0;

bool ring_active = false;
bool ring_tone_on = false;
uint8_t ring_completed_beeps = 0;
uint8_t last_rung_alert = ALERT_NONE;
uint8_t active_alert = ALERT_NONE;
unsigned long ring_start_ms = 0;
unsigned long ring_phase_start_ms = 0;
unsigned long next_ring_allowed_ms = 0;
uint16_t ring_freq_hz = 0;
uint16_t ring_on_ms = 0;
uint16_t ring_off_ms = 0;
uint8_t ring_total_beeps = 0;

void buzzer_off()
{
  digitalWrite(BUZZER_PIN_A, LOW);
  digitalWrite(BUZZER_PIN_B, LOW);
}

void play_hbridge_tone_blocking(uint16_t freq_hz, uint16_t duration_ms)
{
  if (freq_hz == 0 || duration_ms == 0) return;

  unsigned long half_period_us = 500000UL / freq_hz;
  unsigned long end_us = micros() + (unsigned long)duration_ms * 1000UL;
  bool polarity = false;

  while ((long)(micros() - end_us) < 0)
  {
    polarity = !polarity;
    digitalWrite(BUZZER_PIN_A, polarity ? HIGH : LOW);
    digitalWrite(BUZZER_PIN_B, polarity ? LOW : HIGH);
    delayMicroseconds(half_period_us);
    Watchdog.reset();
  }

  buzzer_off();
}

void set_ring_pattern(uint8_t alert)
{
  if (alert == ALERT_CO2)
  {
    ring_freq_hz = 1900;
    ring_on_ms = 120;
    ring_off_ms = 100;
    ring_total_beeps = 8;
  }
  else if (alert == ALERT_TVOC)
  {
    ring_freq_hz = 1500;
    ring_on_ms = 120;
    ring_off_ms = 140;
    ring_total_beeps = 6;
  }
  else
  {
    ring_freq_hz = 1300;
    ring_on_ms = 100;
    ring_off_ms = 180;
    ring_total_beeps = 4;
  }
}

void start_ring(uint8_t alert, unsigned long now_ms)
{
  set_ring_pattern(alert);
  ring_active = true;
  ring_tone_on = true;
  ring_completed_beeps = 0;
  ring_start_ms = now_ms;
  ring_phase_start_ms = now_ms;
  active_alert = alert;
}

void stop_ring(unsigned long now_ms)
{
  ring_active = false;
  ring_tone_on = false;
  ring_completed_beeps = 0;
  buzzer_off();
  next_ring_allowed_ms = now_ms + ALERT_RING_COOLDOWN_MS;
}

void service_ring(unsigned long now_ms)
{
  if (!ring_active) return;

  if (now_ms - ring_start_ms >= ALERT_RING_MAX_MS)
  {
    stop_ring(now_ms);
    return;
  }

  if (ring_tone_on)
  {
    play_hbridge_tone_blocking(ring_freq_hz, ring_on_ms);
    ring_tone_on = false;
    ring_phase_start_ms = millis();
    ring_completed_beeps++;
  }
  else if (now_ms - ring_phase_start_ms >= ring_off_ms)
  {
    if (ring_completed_beeps >= ring_total_beeps)
    {
      stop_ring(now_ms);
      return;
    }
    ring_tone_on = true;
    ring_phase_start_ms = now_ms;
  }
}

void update_touch_pad(Adafruit_FreeTouch &pad, bool pad_ok, bool &touch_calibrated, uint16_t &touch_baseline, bool &touch_active, uint16_t threshold)
{
  if (!pad_ok) return;

  uint16_t raw = pad.measure();
  if (raw == 0) return;

  if (!touch_calibrated)
  {
    touch_baseline = raw;
    touch_calibrated = true;
  }

  touch_active = (raw > (uint16_t)(touch_baseline + threshold));
  if (!touch_active)
  {
    touch_baseline = (uint16_t)(((uint32_t)touch_baseline * 15UL + raw) / 16UL);
  }
}

int16_t quantize_tenths(float value)
{
  long q = lroundf(value * 10.0f);
  if (q > 32767L) q = 32767L;
  if (q < -32767L) q = -32767L;
  return (int16_t)q;
}

void push_coarse_bin(const CoarseBin &bin)
{
  if (bin.count == 0) return;
  coarse_bins[coarse_head] = bin;
  coarse_head = (uint16_t)((coarse_head + 1) % COARSE_BIN_CAPACITY);
  if (coarse_count < COARSE_BIN_CAPACITY) coarse_count++;
}

void finalize_active_bin()
{
  if (!active_bin_init || active_count == 0) return;

  CoarseBin b = {};
  b.start_ms = active_bin_start_ms;
  b.t10 = active_count_t ? (int16_t)(active_sum_t10 / (int32_t)active_count_t) : -32768;
  b.h10 = active_count_h ? (int16_t)(active_sum_h10 / (int32_t)active_count_h) : -32768;
  b.aqi10 = (int16_t)(active_sum_aqi10 / (int32_t)active_count);
  b.co2 = (int16_t)(active_sum_co2 / (int32_t)active_count);
  b.tvoc = (int16_t)(active_sum_tvoc / (int32_t)active_count);
  b.count_t = active_count_t;
  b.count_h = active_count_h;
  b.count = active_count;
  push_coarse_bin(b);
}

void reset_active_bin(uint32_t start_ms)
{
  active_bin_start_ms = start_ms;
  active_sum_t10 = 0;
  active_sum_h10 = 0;
  active_sum_aqi10 = 0;
  active_sum_co2 = 0;
  active_sum_tvoc = 0;
  active_count_t = 0;
  active_count_h = 0;
  active_count = 0;
  active_bin_init = true;
}

void append_sample_to_history(uint32_t now_ms, float t, float h, float aqi, float co2, float tvoc)
{
  int16_t t10 = isnan(t) ? (int16_t)-32768 : quantize_tenths(t);
  int16_t h10 = isnan(h) ? (int16_t)-32768 : quantize_tenths(h);
  int16_t aqi10 = quantize_tenths(aqi);
  int16_t co2_i = (int16_t)lroundf(co2);
  int16_t tvoc_i = (int16_t)lroundf(tvoc);

  recent_samples[recent_head] = {now_ms, t10, h10, aqi10, co2_i, tvoc_i};
  recent_head = (uint16_t)((recent_head + 1) % RECENT_SAMPLE_CAPACITY);
  if (recent_count < RECENT_SAMPLE_CAPACITY) recent_count++;

  uint32_t bin_start = now_ms - (now_ms % COARSE_BIN_MS);
  if (!active_bin_init)
  {
    reset_active_bin(bin_start);
  }

  while (active_bin_start_ms + COARSE_BIN_MS <= bin_start)
  {
    finalize_active_bin();
    reset_active_bin(active_bin_start_ms + COARSE_BIN_MS);
  }

  if (t10 != (int16_t)-32768)
  {
    active_sum_t10 += t10;
    active_count_t++;
  }
  if (h10 != (int16_t)-32768)
  {
    active_sum_h10 += h10;
    active_count_h++;
  }
  active_sum_aqi10 += aqi10;
  active_sum_co2 += co2_i;
  active_sum_tvoc += tvoc_i;
  active_count++;

  sample_count_total++;
}

void compute_window_averages(uint32_t now_ms, uint32_t window_ms, float &t_avg, float &h_avg, float &aqi_avg, float &co2_avg, float &tvoc_avg, uint16_t &count_used)
{
  uint32_t window_start = (now_ms > window_ms) ? (now_ms - window_ms) : 0;

  float sum_t = 0.0f;
  float sum_h = 0.0f;
  float sum_aqi = 0.0f;
  float sum_co2 = 0.0f;
  float sum_tvoc = 0.0f;
  uint32_t cnt_t = 0;
  uint32_t cnt_h = 0;
  uint32_t cnt = 0;

  uint32_t recent_oldest_ts = now_ms;
  if (recent_count > 0)
  {
    uint16_t recent_oldest_idx = (uint16_t)((recent_head + RECENT_SAMPLE_CAPACITY - recent_count) % RECENT_SAMPLE_CAPACITY);
    recent_oldest_ts = recent_samples[recent_oldest_idx].ts;
  }

  if (coarse_count > 0 && window_start < recent_oldest_ts)
  {
    uint16_t idx = (uint16_t)((coarse_head + COARSE_BIN_CAPACITY - coarse_count) % COARSE_BIN_CAPACITY);
    for (uint16_t i = 0; i < coarse_count; i++)
    {
      const CoarseBin &b = coarse_bins[idx];
      uint32_t b_end = b.start_ms + COARSE_BIN_MS;
      if (b_end <= window_start)
      {
        idx = (uint16_t)((idx + 1) % COARSE_BIN_CAPACITY);
        continue;
      }
      if (b.start_ms >= recent_oldest_ts)
      {
        break;
      }

      if (b.t10 != (int16_t)-32768)
      {
        sum_t += ((float)b.t10 / 10.0f) * (float)b.count_t;
        cnt_t += b.count_t;
      }
      if (b.h10 != (int16_t)-32768)
      {
        sum_h += ((float)b.h10 / 10.0f) * (float)b.count_h;
        cnt_h += b.count_h;
      }
      sum_aqi += ((float)b.aqi10 / 10.0f) * (float)b.count;
      sum_co2 += (float)b.co2 * (float)b.count;
      sum_tvoc += (float)b.tvoc * (float)b.count;
      cnt += b.count;

      idx = (uint16_t)((idx + 1) % COARSE_BIN_CAPACITY);
    }
  }

  if (recent_count > 0)
  {
    uint16_t idx = (uint16_t)((recent_head + RECENT_SAMPLE_CAPACITY - recent_count) % RECENT_SAMPLE_CAPACITY);
    for (uint16_t i = 0; i < recent_count; i++)
    {
      const PackedSample &s = recent_samples[idx];
      if (s.ts >= window_start)
      {
        if (s.t10 != (int16_t)-32768)
        {
          sum_t += (float)s.t10 / 10.0f;
          cnt_t++;
        }
        if (s.h10 != (int16_t)-32768)
        {
          sum_h += (float)s.h10 / 10.0f;
          cnt_h++;
        }
        sum_aqi += (float)s.aqi10 / 10.0f;
        sum_co2 += (float)s.co2;
        sum_tvoc += (float)s.tvoc;
        cnt++;
      }
      idx = (uint16_t)((idx + 1) % RECENT_SAMPLE_CAPACITY);
    }
  }

  t_avg = cnt_t ? (sum_t / (float)cnt_t) : NAN;
  h_avg = cnt_h ? (sum_h / (float)cnt_h) : NAN;
  aqi_avg = cnt ? (sum_aqi / (float)cnt) : NAN;
  co2_avg = cnt ? (sum_co2 / (float)cnt) : NAN;
  tvoc_avg = cnt ? (sum_tvoc / (float)cnt) : NAN;
  count_used = (uint16_t)min((uint32_t)65535, cnt);
}

void try_init_sensors()
{
  if (!aht_ok) aht_ok = aht.begin();
  if (!ens_ok) ens_ok = (ens160.begin() == 0);
}

void setup()
{
  Serial.begin(115200);
  unsigned long serial_wait_start = millis();
  while (!Serial && millis() - serial_wait_start < SERIAL_WAIT_MAX_MS) delay(10);

  Watchdog.enable(WDT_TIMEOUT_MS);
  Watchdog.reset();

  pinMode(BUZZER_PIN_A, OUTPUT);
  pinMode(BUZZER_PIN_B, OUTPUT);
  buzzer_off();

  touch_rate_ok = touch_rate_pad.begin();
  if (touch_rate_ok)
  {
    uint32_t base_sum = 0;
    uint8_t base_n = 0;
    for (int i = 0; i < 12; i++)
    {
      uint16_t v = touch_rate_pad.measure();
      if (v > 0) { base_sum += v; base_n++; }
      delay(5);
    }
    if (base_n > 0)
    {
      touch_rate_baseline = (uint16_t)(base_sum / base_n);
      touch_rate_calibrated = true;
    }
  }

  touch_window_ok = touch_window_pad.begin();
  if (touch_window_ok)
  {
    uint32_t base_sum = 0;
    uint8_t base_n = 0;
    for (int i = 0; i < 12; i++)
    {
      uint16_t v = touch_window_pad.measure();
      if (v > 0) { base_sum += v; base_n++; }
      delay(5);
    }
    if (base_n > 0)
    {
      touch_window_baseline = (uint16_t)(base_sum / base_n);
      touch_window_calibrated = true;
    }
  }

  Wire.begin();
  Wire.setTimeout(50);

  try_init_sensors();
  Serial.println(F("Air Quality Dashboard boot"));
  Serial.print(F("AHT: "));
  Serial.println(aht_ok ? F("OK") : F("RETRY"));
  Serial.print(F("ENS160: "));
  Serial.println(ens_ok ? F("OK") : F("RETRY"));
  Serial.print(F("TOUCH(A5 rate): "));
  Serial.println(touch_rate_ok ? F("OK") : F("UNAVAILABLE"));
  Serial.print(F("TOUCH(A3 window): "));
  Serial.println(touch_window_ok ? F("OK") : F("UNAVAILABLE"));
  Serial.print(F("RateHz: "));
  Serial.println(UPDATE_RATE_LABELS[update_rate_idx]);
  Serial.print(F("AvgWindow: "));
  Serial.println(AVG_WINDOW_LABELS[avg_window_idx]);

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.sendBuffer();

  boot_ms = millis();
  start_ring(ALERT_AQI, boot_ms);
  ring_freq_hz = 2200;
  ring_on_ms = 80;
  ring_off_ms = 60;
  ring_total_beeps = 3;
  Serial.println(F("Buzzer self-test chirp"));

}

// FIXED: TYPE-SPECIFIC THRESHOLDS - T/H sensitive, CO2/TVOC coarse
// SIMPLIFIED get_trend(): SEPARATE LOGIC FOR EACH SENSOR TYPE

// ULTRA SIMPLE: Compare SCREEN CURRENT vs SCREEN AVG
int get_trend(float current, float avg, int sensor_type)
{
  float delta = current - avg;  // 
  
  // T/H (0.2/0.5/1.0)
  if (sensor_type == 0 || sensor_type == 1) {  // T or H
    if (delta > 1.0) return 3;      // avg >> current = +++
    if (delta > 0.5) return 2;      // avg > current = ++ 
    if (delta > 0.2) return 1;      // avg > current = +
    if (delta < -1.0) return -3;    // avg << current = ---
    if (delta < -0.5) return -2;    // avg < current = --
    if (delta < -0.2) return -1;    // avg < current = -
  }
  
  // AQI (sensitive)
  if (sensor_type == 2) {
    if (delta > 1.0) return 3;
    if (delta > 0.5) return 2;
    if (delta > 0.2) return 1;
    if (delta < -1.0) return -3;
    if (delta < -0.5) return -2; 
    if (delta < -0.2) return -1;
  }
  
  // CO2 (50/100/200 from your screen data)
  if (sensor_type == 3) {
    if (delta > 200) return 3;     // 1092→1212 avg-current=120 → ++
    if (delta > 100) return 2;
    if (delta > 50)  return 1;
    if (delta < -200) return -3;
    if (delta < -100) return -2;
    if (delta < -50)  return -1;
  }
  
  // TVOC (200/400/700)
  if (sensor_type == 4) {
    if (delta > 700) return 3;
    if (delta > 400) return 2;
    if (delta > 200) return 1;
    if (delta < -700) return -3;
    if (delta < -400) return -2;
    if (delta < -200) return -1;
  }
  
  return 0;
}


void print_trend(int direction, float change, int y_base)
{
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.setCursor(112, y_base - 2);
  float a = fabs(change);
  
  if (direction > 0) {
    if (a >= 20.0 || a >= 1.0)  u8g2.print(F("+++"));  // CO2 +25, T +1.2°C
    else if (a >= 10.0 || a >= 0.5) u8g2.print(F("++"));
    else if (a >= 4.0 || a >= 0.2) u8g2.print(F("+"));
    else u8g2.print(F("=="));
  } else if (direction < 0) {
    if (a >= 20.0 || a >= 1.0)  u8g2.print(F("---"));
    else if (a >= 10.0 || a >= 0.5) u8g2.print(F("--"));
    else if (a >= 4.0 || a >= 0.2) u8g2.print(F("-"));
    else u8g2.print(F("=="));
  } else {
    u8g2.print(F("=="));
  }
}

void loop()
{
  static unsigned long last_update_ms = 0;
  unsigned long now = millis();

  if (now - boot_ms >= FORCED_RESET_MS)
  {
    NVIC_SystemReset();
  }

  if ((!aht_ok || !ens_ok) && (now - last_sensor_retry_ms >= SENSOR_RETRY_MS))
  {
    last_sensor_retry_ms = now;
    try_init_sensors();
  }

  update_touch_pad(touch_rate_pad, touch_rate_ok, touch_rate_calibrated, touch_rate_baseline, touch_rate_active, TOUCH_DELTA_RATE);
  update_touch_pad(touch_window_pad, touch_window_ok, touch_window_calibrated, touch_window_baseline, touch_window_active, TOUCH_DELTA_WINDOW);

  if (touch_rate_active && !touch_rate_prev && (now - last_rate_toggle_ms >= TOUCH_TOGGLE_DEBOUNCE_MS))
  {
    update_rate_idx = (uint8_t)((update_rate_idx + 1) % UPDATE_RATE_COUNT);
    current_update_interval_ms = UPDATE_INTERVAL_OPTIONS_MS[update_rate_idx];
    last_rate_toggle_ms = now;

    play_hbridge_tone_blocking(2600, 35);
    delay(20);
    play_hbridge_tone_blocking(3200, 35);

    Serial.print(F("RateHz: "));
    Serial.println(UPDATE_RATE_LABELS[update_rate_idx]);
  }

  if (touch_window_active && !touch_window_prev && (now - last_window_toggle_ms >= TOUCH_TOGGLE_DEBOUNCE_MS))
  {
    avg_window_idx = (uint8_t)((avg_window_idx + 1) % AVG_WINDOW_COUNT);
    last_window_toggle_ms = now;

    play_hbridge_tone_blocking(1800, 30);

    Serial.print(F("AvgWindow: "));
    Serial.println(AVG_WINDOW_LABELS[avg_window_idx]);
  }

  touch_rate_prev = touch_rate_active;
  touch_window_prev = touch_window_active;

  Watchdog.reset();
  service_ring(now);
  if (now - last_update_ms < current_update_interval_ms) return;
  last_update_ms = now;

  sensors_event_t hum, temp;
  float T = NAN, H = NAN;
  if (aht_ok)
  {
    aht.getEvent(&hum, &temp);
    if (!isnan(temp.temperature)) T = temp.temperature;
    if (!isnan(hum.relative_humidity)) H = hum.relative_humidity;
  }

  uint8_t aqi_raw = 0;
  uint16_t eco2_raw = 0;
  uint16_t tvoc_raw = 0;
  if (ens_ok)
  {
    aqi_raw = ens160.getAQI();
    eco2_raw = ens160.getECO2();
    tvoc_raw = ens160.getTVOC();
  }

  append_sample_to_history(now, T, H, (float)aqi_raw, (float)eco2_raw, (float)tvoc_raw);

  float t_avg = NAN;
  float h_avg = NAN;
  float aqi_avg = NAN;
  float co2_avg = NAN;
  float tvoc_avg = NAN;
  compute_window_averages(now, AVG_WINDOW_OPTIONS_MS[avg_window_idx], t_avg, h_avg, aqi_avg, co2_avg, tvoc_avg, sample_count_window);

  bool aqi_alert = !isnan(aqi_avg) && (aqi_avg >= 3.0f);
  bool tvoc_alert = !isnan(tvoc_avg) && (tvoc_avg >= 600.0f);
  bool co2_alert = !isnan(co2_avg) && (co2_avg >= 1000.0f);

  uint8_t wanted_alert = co2_alert ? ALERT_CO2 : (tvoc_alert ? ALERT_TVOC : (aqi_alert ? ALERT_AQI : ALERT_NONE));

  if (wanted_alert == ALERT_NONE)
  {
    last_rung_alert = ALERT_NONE;
  }
  else if (!ring_active)
  {
    bool can_ring = (now >= next_ring_allowed_ms) || (wanted_alert != last_rung_alert);
    if (can_ring)
    {
      start_ring(wanted_alert, now);
      last_rung_alert = wanted_alert;
    }
  }

  // PASS CURRENT VALUE for threshold scaling
int t_trend = get_trend(T, t_avg, 0);        // T vs t_avg
int h_trend = get_trend(H, h_avg, 1);        // H vs h_avg
int aqi_trend = get_trend(aqi_raw, aqi_avg, 2);
int co2_trend = get_trend(eco2_raw, co2_avg, 3);   // 1150 vs 1092 = ++
int tvoc_trend = get_trend(tvoc_raw, tvoc_avg, 4);


  // DEBUG SERIAL
  Serial.print(F("T:")); Serial.print(T,1); Serial.print(F("/")); Serial.print(t_avg,3);
  Serial.print(F(" CO2:")); Serial.print(eco2_raw); Serial.print(F("/")); Serial.print(co2_avg,1);
  Serial.print(F(" N:")); Serial.print(sample_count_window); Serial.println();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  int base_y = 10;
  int row_h = 12;

  // T Row
  u8g2.setCursor(0, base_y - 2);
  u8g2.print(F("T ")); u8g2.print(T, 1); u8g2.print(F("C ")); 
  u8g2.setCursor(52, base_y - 2);
  u8g2.print(t_avg, 3); u8g2.print(F("C"));
  print_trend(t_trend, t_avg - T, base_y);

  // H Row
  u8g2.setCursor(0, base_y + row_h - 2);
  u8g2.print(F("H ")); u8g2.print(H, 1); u8g2.print(F("% ")); 
  u8g2.setCursor(52, base_y + row_h - 2);
  u8g2.print(h_avg, 3); u8g2.print(F("%"));
  print_trend(h_trend, h_avg - H, base_y + row_h);

  // AQI
  u8g2.setCursor(0, base_y + row_h * 2 - 2);
  u8g2.print(F("AQI ")); u8g2.print((float)aqi_raw, 1); u8g2.print(F(" ")); 
  u8g2.setCursor(52, base_y + row_h * 2 - 2);
  u8g2.print(aqi_avg, 2);
  print_trend(aqi_trend, aqi_avg - aqi_raw, base_y + row_h * 2);

  // CO2
  u8g2.setCursor(0, base_y + row_h * 3 - 2);
  u8g2.print(F("CO2 ")); u8g2.print((int)eco2_raw); u8g2.print(F(" ")); 
  u8g2.setCursor(60, base_y + row_h * 3 - 2);
  u8g2.print(co2_avg, 1);
  print_trend(co2_trend, co2_avg - eco2_raw, base_y + row_h * 3);

  // TVOC
  u8g2.setCursor(0, base_y + row_h * 4 - 2);
  u8g2.print(F("TVOC ")); u8g2.print((int)tvoc_raw); u8g2.print(F(" ")); 
  u8g2.setCursor(60, base_y + row_h * 4 - 2);
  u8g2.print(tvoc_avg, 1);
  print_trend(tvoc_trend, tvoc_avg - tvoc_raw, base_y + row_h * 4);

  // Bottom
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.setCursor(0, 63);
  time_t uptime_sec = (time_t)(now / 1000UL);
  tmElements_t tm;
  breakTime(uptime_sec, tm);
  unsigned long uptime_hours = (unsigned long)(tm.Day - 1) * 24UL + (unsigned long)tm.Hour;
  char uptime_buf[16];
  snprintf(uptime_buf, sizeof(uptime_buf), "%lu:%02d:%02d", uptime_hours, tm.Minute, tm.Second);
  u8g2.print(F("R")); u8g2.print(UPDATE_RATE_LABELS[update_rate_idx]); u8g2.print(F("Hz "));
  u8g2.print(F("W")); u8g2.print(AVG_WINDOW_LABELS[avg_window_idx]); u8g2.print(F(" "));
  u8g2.print(F("C")); u8g2.print(sample_count_total); u8g2.print(F(" "));
  u8g2.print(F("N")); u8g2.print(sample_count_window); u8g2.print(F(" "));
  u8g2.print(F("U")); u8g2.print(uptime_buf);

  // CO2 flash
  static unsigned long flash_time = 0;
  static bool flash_state = false;
  if (aqi_alert || tvoc_alert || co2_alert)
  {
    if (millis() - flash_time > 300) { flash_state = !flash_state; flash_time = millis(); }
    if (flash_state)
    {
      if (aqi_alert) u8g2.drawFrame(0, base_y + row_h * 2 - row_h + 1, 128, row_h);
      if (co2_alert) u8g2.drawFrame(0, base_y + row_h * 3 - row_h + 1, 128, row_h);
      if (tvoc_alert) u8g2.drawFrame(0, base_y + row_h * 4 - row_h + 1, 128, row_h);
    }
  }

  u8g2.sendBuffer();
  Watchdog.reset();
}
