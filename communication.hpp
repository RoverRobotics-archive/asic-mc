

void req_info();
// todo: print info; information that is not changed during normal robot
// operation

void req_status();
// todo: print status; information that may change after the robot is booted

void set_target_velocity(float transverse_meters_per_second,
                         float rotational_radians_per_second);