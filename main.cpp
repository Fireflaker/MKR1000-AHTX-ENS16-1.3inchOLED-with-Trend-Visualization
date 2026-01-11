// COMPLETE FINAL CODE - FIXED: CO2/TVOC less sensitive thresholds

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <DFRobot_ENS160.h>
#include <U8g2lib.h>
#include <math.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Adafruit_AHTX0 aht;
DFRobot_ENS160_I2C ens160(&Wire, 0x53);

#define MAX_HISTORY 1000
float t_history[MAX_HISTORY], h_history[MAX_HISTORY];
float aqi_history[MAX_HISTORY], co2_history[MAX_HISTORY], tvoc_history[MAX_HISTORY];
int history_count = 0;
int history_idx = 0;

void setup()
{
  Serial.begin(115200);
  while (!Serial) delay(10);
  Wire.begin();
  if (!aht.begin() || ens160.begin() != 0)
  {
    Serial.println(F("Sensors failed"));
    while (1) delay(100);
  }
  Serial.println(F("Sensors OK - Air Quality Dashboard"));
  u8g2.begin();
  for (int i = 0; i < MAX_HISTORY; i++)
  {
    t_history[i] = h_history[i] = NAN;
    aqi_history[i] = co2_history[i] = tvoc_history[i] = NAN;
  }
}

float moving_average(float *hist, int n_samples)
{
  if (n_samples <= 0 || history_count < n_samples) return NAN;
  float sum = 0;
  int valid_count = 0;
  int start = (history_idx - n_samples + MAX_HISTORY) % MAX_HISTORY;
  for (int i = 0; i < n_samples; i++)
  {
    int idx = (start + i) % MAX_HISTORY;
    if (!isnan(hist[idx]))
    {
      sum += hist[idx];
      valid_count++;
    }
  }
  return valid_count ? sum / valid_count : NAN;
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
  sensors_event_t hum, temp;
  float T = NAN, H = NAN;
  aht.getEvent(&hum, &temp);
  if (!isnan(temp.temperature)) T = temp.temperature;
  if (!isnan(hum.relative_humidity)) H = hum.relative_humidity;

  uint8_t aqi_raw = ens160.getAQI();
  uint16_t eco2_raw = ens160.getECO2();
  uint16_t tvoc_raw = ens160.getTVOC();

  if (!isnan(T)) t_history[history_idx] = T;
  if (!isnan(H)) h_history[history_idx] = H;
  aqi_history[history_idx] = (float)aqi_raw;
  co2_history[history_idx] = (float)eco2_raw;
  tvoc_history[history_idx] = (float)tvoc_raw;

  history_idx = (history_idx + 1) % MAX_HISTORY;
  if (history_count < MAX_HISTORY) history_count++;

  pinMode(A4, OUTPUT); digitalWrite(A4, LOW);
  pinMode(A6, OUTPUT); digitalWrite(A6, HIGH);
  int pot = analogRead(A5);
  digitalWrite(A6, LOW);
  int avg_samples = constrain(map(pot, 0, 1023, 10, 1000), 16, history_count);  // Min 16

  float t_avg = moving_average(t_history, avg_samples);
  float h_avg = moving_average(h_history, avg_samples);
  float aqi_avg = moving_average(aqi_history, avg_samples);
  float co2_avg = moving_average(co2_history, avg_samples);
  float tvoc_avg = moving_average(tvoc_history, avg_samples);

  // PASS CURRENT VALUE for threshold scaling
int t_trend = get_trend(T, t_avg, 0);        // T vs t_avg
int h_trend = get_trend(H, h_avg, 1);        // H vs h_avg
int aqi_trend = get_trend(aqi_raw, aqi_avg, 2);
int co2_trend = get_trend(eco2_raw, co2_avg, 3);   // 1150 vs 1092 = ++
int tvoc_trend = get_trend(tvoc_raw, tvoc_avg, 4);


  // DEBUG SERIAL
  Serial.print(F("T:")); Serial.print(T,1); Serial.print(F("/")); Serial.print(t_avg,3);
  Serial.print(F(" CO2:")); Serial.print(eco2_raw); Serial.print(F("/")); Serial.print(co2_avg,1);
  Serial.print(F(" N:")); Serial.print(avg_samples); Serial.println();

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
  u8g2.print(F("Pot:")); u8g2.print(pot); u8g2.print(F(" Hist:")); u8g2.print(history_count); 
  u8g2.print(F(" N:")); u8g2.print(avg_samples);

  // CO2 flash
  static unsigned long flash_time = 0;
  static bool flash_state = false;
  if (!isnan(co2_avg) && co2_avg > 1200)
  {
    if (millis() - flash_time > 300) { flash_state = !flash_state; flash_time = millis(); }
    if (flash_state) u8g2.drawFrame(0, base_y + row_h * 3 - row_h + 1, 128, row_h);
  }

  u8g2.sendBuffer();
  delay(950);
}
