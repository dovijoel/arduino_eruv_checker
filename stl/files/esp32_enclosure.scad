/*
Created by: David Taylor
Description: ESP32 DevKit screwless enclosure for projects where you don't need access to the pins such as ESPresense
Dimensions based on: ESP32 DevKit Case by bkgoodman at https://www.thingiverse.com/thing:4125952
*/

include <MCAD/boxes.scad>
include <esp_modules.scad>

//Fragment number (higher values have rounder circles but slower rendoring)
fn=36;//[36:360]
//Diameter of the light guides
lightGuideDiameter=5;//[0:0.05:5]

// NodeMCU V3 USB-C dimensions
board_x = 30.8;
board_y = 58.0;
board_z = 1.6;
fit_tol = 1.2; // tolerance for easy fit
case_wall = 2.0;

case_x = board_x + 2*case_wall + fit_tol; // external X
case_y = board_y + 2*case_wall + fit_tol; // external Y
case_z = 30; // keep height as before

inner_x = board_x + fit_tol;
inner_y = board_y + fit_tol;
inner_z = case_z + 0.01;

translate([0,0,case_z/2])lid();
enclosure();

module enclosure()
{
    difference()
    {
        union()
        {
            // Pin locations updated for new board size
            pin_offset_x = (board_x/2) - 4.0;
            pin_offset_y = (board_y/2) - 4.5;
            translate([pin_offset_x, pin_offset_y,-case_z/2+5]) pin();
            translate([-pin_offset_x, pin_offset_y,-case_z/2+5]) pin();
            translate([pin_offset_x, -pin_offset_y,-case_z/2+5]) pin();
            translate([-pin_offset_x, -pin_offset_y,-case_z/2+5]) pin();
        }
        //pin cutouts
        translate([-(board_x/2+2.5),0,-case_z/2+2.5])cube([2.9,board_y,5.1], center=true);
        translate([(board_x/2+2.5),0,-case_z/2+2.5])cube([2.9,board_y,5.1], center=true);
    }
    difference()
    {
        //sides
        translate([0,0,-case_z/2])roundedBox([case_x, case_y, case_z], 3.97, true, $fn=fn);
        //inner void
        translate([0,0,-case_z/2])roundedBox([inner_x, inner_y, inner_z], 3, true, $fn=fn);
        //usb port opening (centered on Y axis)
        translate([0,case_y/2-case_wall,5-case_z/2])rotate([90,0,0])roundedBox([8.5,8,26], 1, true, $fn=fn);
        //inner lid clip indents
        translate([case_x/2-2,0,3.75-case_z/2])rotate([90,0,0])cylinder(h=20, d=1, $fn=fn, center=true);
        translate([-(case_x/2-2),0,3.75-case_z/2])rotate([90,0,0])cylinder(h=20, d=1, $fn=fn, center=true);
    }
    //bottom (now flush)
    translate([0,0,-case_z/2])roundedBox([case_x, case_y, 2], 3.97, true, $fn=fn);
    //inner lid clip
    translate([case_x/2-2,0,4.75-case_z/2])rotate([90,0,0])cylinder(h=20, d=1, $fn=fn, center=true);
    translate([-(case_x/2-2),0,4.75-case_z/2])rotate([90,0,0])cylinder(h=20, d=1, $fn=fn, center=true);
    // NodeMCU_V3USBC module for visualization
    translate([0,0,5-case_z/2]) NodeMCU_V3USBC();
}

module lid()
{
    difference()
    {
        union()
        {
            //top
            translate([0,0,-2])roundedBox([case_x, case_y, 2], 3.97, true, $fn=fn);
            //lip
            translate([0,0,0])roundedBox([inner_x-2*case_wall, inner_y-2*case_wall, 2], 3, true, $fn=fn);
            // GPS PA1010D corner pins (4 pins)
            gps_x = 25.5/2;
            gps_y = 25.4/2;
            pin_d = 2;
            pin_h = 5;
            for (x = [-gps_x+1.5, gps_x-1.5])
                for (y = [-gps_y+1.5, gps_y-1.5])
                    translate([x, y, 1])
                        cylinder(h=pin_h, d=pin_d, $fn=24);
        }
        //outer indents
        translate([(case_x/2)-1.5,0,-0.5])rotate([90,0,0])cylinder(h=20, d=1, $fn=fn, center=true);
        translate([-(case_x/2)+1.5,0,-0.5])rotate([90,0,0])cylinder(h=20, d=1, $fn=fn, center=true);
        //lid clip strain relief
        translate([0,10.4,0.5])cube([60,1,3], center=true);
        translate([0,-10.4,0.5])cube([60,1,3], center=true);
        //inner void
        translate([0,0,0.05])roundedBox([inner_x-4, inner_y-4, 2.1], 2.5, true, $fn=fn);
        // THREE LED holes at the bottom edge (where button cutouts were)
        led_y = -(case_y/2-case_wall-2.5); // y-position at new front edge
        led_z = 0; // on the lid surface
        led_spacing = 12; // spacing between LEDs for wider case
        for (i = [-1,0,1])
            translate([i*led_spacing, led_y, led_z])
                cylinder(h=10, d=lightGuideDiameter, $fn=fn, center=true);
        //usb port opening
        translate([0,-case_y/2+case_wall,3.75])rotate([90,0,0])roundedBox([8.5,8,10], 1, true, $fn=fn);
        //vents Start
        translate([4,case_y/2-7,0])rotate([0,0,-30])roundedBox([15,1,10],0.5,true, $fn=fn);
        translate([0,case_y/2-10,0])rotate([0,0,-30])roundedBox([25,1,10],0.5,true, $fn=fn);
        translate([0,case_y/2-15,0])rotate([0,0,-30])roundedBox([25,1,10],0.5,true, $fn=fn);
        translate([-2,case_y/2-19,0])rotate([0,0,-30])roundedBox([20,1,10],0.5,true, $fn=fn);
        translate([-6,case_y/2-22,0])rotate([0,0,-30])roundedBox([10,1,10],0.5,true, $fn=fn);
    }
}

module pin()
{
    // Lower pin for ESP32: only 5mm clearance
    cylinder(h=5, d1=3.13, d2=2.08, $fn=fn, center=true);
    translate([0,0,-2])cylinder(h=4, d=5, $fn=fn, center=true);
}