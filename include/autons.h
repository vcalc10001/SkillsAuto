#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;
static int current_auton_selection = 1;
static bool auto_started = false;

void run_auto();

void default_constants();

void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();

void QRedRight();
void ERedRight();
void QBlueLeft();
void EBlueLeft();

void skills_auto();

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
//Use for medium turns (60-120 degrees) - tuned to 90
void turn_to_heading_medium(float targetHeading);
//Use for large turns (> 120 degrees) - This is not yet tuned
void turn_to_heading_large(float targetHeading);