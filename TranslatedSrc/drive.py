from vex import *
from drive import *
from odom import *
from PID import *
from util import *

def Drive(drive_setup, DriveL, DriveR, wheel_diameter, wheel_ratio, gyro_scale, DriveLF_port, DriveRF_port, DriveLB_port, DriveRB_port, ForwardTracker_port, ForwardTracker_diameter, ForwardTracker_center_distance, SidewaysTracker_port, SidewaysTracker_diameter, SidewaysTracker_center_distance):
  wheel_diameter(wheel_diameter)
  wheel_ratio(wheel_ratio)
  gyro_scale(gyro_scale)
  drive_in_to_deg_ratio(wheel_ratio/360.0*M_PI*wheel_diameter)
  ForwardTracker_center_distance(ForwardTracker_center_distance)
  ForwardTracker_diameter(ForwardTracker_diameter)
  ForwardTracker_in_to_deg_ratio(M_PI*ForwardTracker_diameter/360.0)
  SidewaysTracker_center_distance(SidewaysTracker_center_distance)
  SidewaysTracker_diameter(SidewaysTracker_diameter)
  SidewaysTracker_in_to_deg_ratio(M_PI*SidewaysTracker_diameter/360.0)
  drive_setup(drive_setup)
  DriveL(DriveL)
  DriveR(DriveR)
  Gyro(brain_inertial)
  DriveLF(DriveLF_port, is_reversed(DriveLF_port))
  DriveRF(DriveRF_port, is_reversed(DriveRF_port))
  DriveLB(DriveLB_port, is_reversed(DriveLB_port))
  DriveRB(DriveRB_port, is_reversed(DriveRB_port))
  R_ForwardTracker(ForwardTracker_port),
  R_SidewaysTracker(SidewaysTracker_port),
  E_ForwardTracker(ThreeWire.Port[ForwardTracker_port-1]),
  E_SidewaysTracker(ThreeWire.Port[SidewaysTracker_port-1])


def drive_with_percentage(leftPercentage, rightPercentage):
  DriveL.spin(fwd, leftVoltage, volt)
  DriveR.spin(fwd, rightVoltage,volt)


def set_turn_constants(funtion_turn_max_percentage, funtion_turn_kp, function_turn_ki, function_turn_kd, function_turn_starti):
  global turn_max_percentage, turn_kp, turn_ki, turn_kd, turn_starti
  function_turn_max_percentage = turn_max_percentage
  function_turn_kp = turn_kp
  function_turn_ki = turn_ki
  function_turn_kd = turn_kd
  function_turn_starti = turn_starti

def set_drive_constants(float drive_max_percentage, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  this->drive_max_percentage = drive_max_percentage;
  this->drive_kp = drive_kp;
  this->drive_ki = drive_ki;
  this->drive_kd = drive_kd;
  this->drive_starti = drive_starti;
} 

def set_heading_constants(float heading_max_percentage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  this->heading_max_percentage = heading_max_percentage;
  this->heading_kp = heading_kp;
  this->heading_ki = heading_ki;
  this->heading_kd = heading_kd;
  this->heading_starti = heading_starti;
}

def set_swing_constants(float swing_max_percentage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  this->swing_max_percentage = swing_max_percentage;
  this->swing_kp = swing_kp;
  this->swing_ki = swing_ki;
  this->swing_kd = swing_kd;
  this->swing_starti = swing_starti;
} 

def set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
  this->turn_settle_error = turn_settle_error;
  this->turn_settle_time = turn_settle_time;
  this->turn_timeout = turn_timeout;
}

def set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout){
  this->drive_settle_error = drive_settle_error;
  this->drive_settle_time = drive_settle_time;
  this->drive_timeout = drive_timeout;
}

def set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
  this->swing_settle_error = swing_settle_error;
  this->swing_settle_time = swing_settle_time;
  this->swing_timeout = swing_timeout;
}

float get_absolute_heading(){ 
  return( reduce_0_to_360( Gyro.rotation()*360.0/gyro_scale ) ); 
}

float get_left_position_in(){
  return( DriveL.position(deg)*drive_in_to_deg_ratio );
}

float get_right_position_in(){
  return( DriveL.position(deg)*drive_in_to_deg_ratio );
}

def turn_to_angle(float angle){
  turn_to_angle(angle, turn_max_percentage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

def turn_to_angle(float angle, float turn_max_percentage){
  turn_to_angle(angle, turn_max_percentage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

def turn_to_angle(float angle, float turn_max_percentage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_angle(angle, turn_max_percentage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

def turn_to_angle(float angle, float turn_max_percentage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  desired_heading = angle;
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_percentage, turn_max_percentage);
    drive_with_percentage(output, -output);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

def drive_distance(float distance){
  drive_distance(distance, desired_heading, drive_max_percentage, heading_max_percentage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

def drive_distance(float distance, float heading){
  drive_distance(distance, heading, drive_max_percentage, heading_max_percentage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

def drive_distance(float distance, float heading, float drive_max_percentage, float heading_max_percentage){
  drive_distance(distance, heading, drive_max_percentage, heading_max_percentage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

def drive_distance(float distance, float heading, float drive_max_percentage, float heading_max_percentage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_distance(distance, heading, drive_max_percentage, heading_max_percentage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

def drive_distance(float distance, float heading, float drive_max_percentage, float heading_max_percentage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  desired_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_percentage, drive_max_percentage);
    heading_output = clamp(heading_output, -heading_max_percentage, heading_max_percentage);

    drive_with_percentage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

def left_swing_to_angle(float angle){
  left_swing_to_angle(angle, swing_max_percentage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

def left_swing_to_angle(float angle, float swing_max_percentage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_percentage, turn_max_percentage);
    DriveL.spin(fwd, output, volt);
    DriveR.stop(hold);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

def right_swing_to_angle(float angle){
  right_swing_to_angle(angle, swing_max_percentage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

def right_swing_to_angle(float angle, float swing_max_percentage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_percentage, turn_max_percentage);
    DriveR.spin(reverse, output, volt);
    DriveL.stop(hold);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

float get_ForwardTracker_position(){
  if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
    return(E_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }else{
    return(R_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }
}

float get_SidewaysTracker_position(){
  if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_ONE_ROTATION){
    return(0);
  }else if (drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
    return(E_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }else{
    return(R_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }
}

def position_track(){
  while(1){
    odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
    task::sleep(5);
  }
}

def set_coordinates(float X_position, float Y_position, float orientation_deg){
  odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
}

int position_track_task(){
  chassis.position_track();
  return(0);
}

float get_X_position(){
  return(odom.X_position);
}

float get_Y_position(){
  return(odom.Y_position);
}

def drive_to_point(float X_position, float Y_position){
  drive_to_point(X_position, Y_position, drive_max_percentage, heading_max_percentage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

def drive_to_point(float X_position, float Y_position, float drive_max_percentage, float heading_max_percentage){
  drive_to_point(X_position, Y_position, drive_max_percentage, heading_max_percentage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

def drive_to_point(float X_position, float Y_position, float drive_max_percentage, float heading_max_percentage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_point(X_position, Y_position, drive_max_percentage, heading_max_percentage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

def drive_to_point(float X_position, float Y_position, float drive_max_percentage, float heading_max_percentage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  while(drivePID.is_settled() == false){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);
    
    if (drive_error<drive_settle_error) { heading_output = 0; }

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_percentage, fabs(heading_scale_factor)*drive_max_percentage);
    heading_output = clamp(heading_output, -heading_max_percentage, heading_max_percentage);

    drive_with_percentage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

def turn_to_point(float X_position, float Y_position){
  turn_to_point(X_position, Y_position, 0, turn_max_percentage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

def turn_to_point(float X_position, float Y_position, float extra_angle_deg){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_percentage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

def turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_percentage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_percentage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

def turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_percentage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_percentage, turn_max_percentage);
    drive_with_percentage(output, -output);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

def holonomic_drive_to_point(float X_position, float Y_position){
  holonomic_drive_to_point(X_position, Y_position, get_absolute_heading(), drive_max_percentage, heading_max_percentage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

def holonomic_drive_to_point(float X_position, float Y_position, float angle){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_percentage, heading_max_percentage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

def holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_percentage, float heading_max_percentage){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_percentage, heading_max_percentage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

def holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_percentage, float heading_max_percentage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_percentage, heading_max_percentage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

def holonomic_drive_to_point(X_position, Y_position, angle, drive_max_percentage, heading_max_percentage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti):
  
  drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  
  turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  
  while(!(drivePID.is_settled() && turnPID.is_settled() ) ){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float turn_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());

    float drive_output = drivePID.compute(drive_error);
    float turn_output = turnPID.compute(turn_error);

    drive_output = clamp(drive_output, drive_max_percentage, drive_max_percentage);
    turn_output = clamp(turn_output, -heading_max_percentage, heading_max_percentage);

    float heading_error = atan2(Y_position-get_Y_position(), X_position-get_X_position());

    DriveLF.spin(fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) + turn_output, volt);
    DriveLB.spin(fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) + turn_output, volt);
    DriveRB.spin(fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) - turn_output, volt);
    DriveRF.spin(fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) - turn_output, volt);
    task::sleep(10);
  }
  DriveLF.stop(hold);
  DriveLB.stop(hold);
  DriveRB.stop(hold);
  DriveRF.stop(hold);
}

def control_arcade():
  DriveL.spin(Foward)
  DriveR.spin(Foward)
  DriveL.set_velocity(controller.Axis3.value()+controller.Axis1.value()), PERCENT)
  DriveR.set_velocity(controller.Axis3.value()-controller.Axis1.value()), PERCENT)


def control_tank():
  DriveL.spin(Foward)
  DriveR.spin(Foward)
  DriveL.set_velocity(controller.Axis3.value(), PERCENT)
  DriveR.set_velocity(controller.Axis2.value(), PERCENT)
