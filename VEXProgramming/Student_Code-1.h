#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

void student_main();    // The main entry point to the student code

// Add your function prototypes below
void drive_straight_distance(double distance, double Kp, double Ki);
int convert_power(double power_percent);
double get_pos_wheel();
double get_pos_arm();
void move_arm(double armAngle);
void drive_until_black();
void test();
void drive_straight_ultrasonic(double distance);
void turn_angle(double initial_angle, double backlash);
double get_angle();
int line_following_case();
void line_following();
// DO NOT ADD ANY PROTOTYPES AFTER THIS LINE
#endif  // STUDENT_CODE_H