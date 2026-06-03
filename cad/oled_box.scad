$fn = 48;

part = "assembly"; // base, lid, assembly

internal_length = 72;
internal_width = 34;
internal_height = 50;

wall = 2;
floor_thickness = 2;
top_thickness = 2;
outer_corner_radius = 3;
lip_height = 6;
lip_wall = 2;
lip_clearance = 0.35;
lip_overlap_into_body = 2.0;
flange_loft_drop = 2.0;
lid_thumb_notch_width = 16;
lid_thumb_notch_depth = 1.4;
lid_thumb_notch_height = 8;

base_internal_height = 25;
lid_internal_height = internal_height - base_internal_height;

outer_length = internal_length + wall * 2;
outer_width = internal_width + wall * 2;
base_outer_height = base_internal_height + floor_thickness;
lid_outer_height = lid_internal_height + top_thickness;
total_outer_height = base_outer_height + lid_outer_height;

screen_open_width = 23;
screen_open_length = 34.5;
screen_front_offset = 4;

usb_cutout_width = 12;
usb_cutout_height = 8;
usb_cutout_center_height = total_outer_height / 3;
usb_cutout_y = (outer_width - usb_cutout_width) / 2;
usb_cutout_z = usb_cutout_center_height - usb_cutout_height / 2;

lip_outer_length = internal_length;
lip_outer_width = internal_width;
lip_inner_length = lip_outer_length - lip_wall * 2;
lip_inner_width = lip_outer_width - lip_wall * 2;

module rounded_prism(length, width, height, radius) {
    linear_extrude(height = height)
        hull() {
            translate([radius, radius]) circle(r = radius);
            translate([length - radius, radius]) circle(r = radius);
            translate([radius, width - radius]) circle(r = radius);
            translate([length - radius, width - radius]) circle(r = radius);
        }
}

module flange_support_loft() {
    if (flange_loft_drop > 0) {
        ring_bottom_z = base_outer_height - lip_overlap_into_body;

        // Left side ramp
        hull() {
            translate([wall, wall, ring_bottom_z - flange_loft_drop])
                cube([0.01, internal_width, 0.01]);
            translate([wall, wall, ring_bottom_z])
                cube([lip_wall, internal_width, 0.01]);
        }

        // Right side ramp
        hull() {
            translate([outer_length - wall, wall, ring_bottom_z - flange_loft_drop])
                cube([0.01, internal_width, 0.01]);
            translate([outer_length - wall - lip_wall, wall, ring_bottom_z])
                cube([lip_wall, internal_width, 0.01]);
        }

        // Front side ramp
        hull() {
            translate([wall, wall, ring_bottom_z - flange_loft_drop])
                cube([internal_length, 0.01, 0.01]);
            translate([wall, wall, ring_bottom_z])
                cube([internal_length, lip_wall, 0.01]);
        }

        // Back side ramp
        hull() {
            translate([wall, outer_width - wall, ring_bottom_z - flange_loft_drop])
                cube([internal_length, 0.01, 0.01]);
            translate([wall, outer_width - wall - lip_wall, ring_bottom_z])
                cube([internal_length, lip_wall, 0.01]);
        }
    }
}

module base_shell() {
    difference() {
        rounded_prism(outer_length, outer_width, base_outer_height, outer_corner_radius);
        translate([wall, wall, floor_thickness])
            cube([internal_length, internal_width, base_internal_height + 0.1]);
        translate([-0.1, usb_cutout_y, usb_cutout_z])
            cube([wall + 0.2, usb_cutout_width, usb_cutout_height]);
    }

    translate([(outer_length - lip_outer_length) / 2, (outer_width - lip_outer_width) / 2, base_outer_height - lip_overlap_into_body])
        difference() {
            cube([lip_outer_length, lip_outer_width, lip_height]);
            translate([(lip_outer_length - lip_inner_length) / 2, (lip_outer_width - lip_inner_width) / 2, -0.1])
                cube([lip_inner_length, lip_inner_width, lip_height + 0.2]);
        }

    flange_support_loft();
}

module lid_shell() {
    difference() {
        rounded_prism(outer_length, outer_width, lid_outer_height, outer_corner_radius);
        translate([wall, wall, -0.1])
            cube([internal_length, internal_width, lid_internal_height + 0.1]);
        translate([screen_front_offset, (outer_width - screen_open_width) / 2, lid_outer_height - top_thickness - 0.1])
            cube([screen_open_length, screen_open_width, top_thickness + 0.2]);
        translate([-0.1, (outer_width - lid_thumb_notch_width) / 2, 0])
            cube([lid_thumb_notch_depth + 0.2, lid_thumb_notch_width, lid_thumb_notch_height]);
    }
}

module assembly_view() {
    base_shell();
    translate([0, 0, base_outer_height])
        lid_shell();
}

module print_layout() {
    base_shell();
    translate([outer_length + 10, 0, 0])
        lid_shell();
}

if (part == "base") {
    base_shell();
} else if (part == "lid") {
    lid_shell();
} else if (part == "print") {
    print_layout();
} else {
    assembly_view();
}
