#include "motioncontroller.h"
int n;
double initial_y;
int lines_crossed;
int on_line;
double laser_values[10];

int mission_follow_line();
int mission_fwd_turn();
int mission_push_box();
int mission_funky_wall();
int mission_laser();
int mission_square();
