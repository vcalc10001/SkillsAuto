#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;

static bool auto_started = false;

void default_constants();

void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();


/*
 0 = Skills
 1 = Red WP (4 Rings + Ladder)
 2 = Red Right Side (2 ring + Ladder)
 3 = Blue WP (4 rings + Ladder)
 4 = Blue Left Side (2 ring + Ladder)
 5 = Red ELIMS RUSH (******** NOT IMPLEMENTED ********)
 6 = Blue ELIMS RUSH(******** NOT IMPLEMENTED ********)
 7 = Drive Test
 8 = Turn Test
*/
static int current_auton_selection = 3;

void run_selected_auto();
void skills_auto();
void red_wp_auto();
void blue_wp_auto();
void red_right_qual_nopid_auto();
void blue_left_qual_nopid_auto();

/* ************************************ */
/* Bunch of pre-tuned Driving functions */
/* ************************************ */
void drive_distance_small(float distance);

void drive_distance_medium(float distance);

void drive_distance_large(float distance);

/* ********************************* */
/* Bunch of pre-tuned turn functions */
/* ********************************* */
//Use for tiny turns (less than 30 degrees) - This is not yet tuned
void turn_to_heading_tiny(float targetHeading);
//Use for small turns (30-60 degrees) - Tuned to 45
void turn_to_heading_small(float targetHeading);
//Use for medium turns (60-120 degrees)
void turn_to_heading_medium(float targetHeading);
//Use for large turns (120-150 degrees)
void turn_to_heading_large(float targetHeading);
//Use for x-large turns (> 150 degrees)
void turn_to_heading_xlarge(float targetHeading);

// Adjust heading after turning (call for situations when accuracy matters a lot)
void adjustHeading(double targetHeading, double tolerance, double timeout);