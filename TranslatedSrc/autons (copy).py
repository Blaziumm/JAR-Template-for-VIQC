from vex import *
from drive import *
from odom import *
from PID import *
from util import *

def default_constants():
  set_drive_constants(10, 1.5, 0, 10, 0)
  set_heading_constants(6, .4, 0, 1, 0)
  set_turn_constants(12, .4, .03, 3, 15)
  set_swing_constants(12, .3, .001, 2, 15)
  set_drive_exit_conditions(1.5, 300, 5000)
  set_turn_exit_conditions(1, 300, 3000)
  set_swing_exit_conditions(1, 300, 3000)


def drive_test():
  drive_distance(6)
  drive_distance(12)
  drive_distance(18)
  drive_distance(-36)


def turn_test():
  turn_to_angle(5)
  turn_to_angle(30)
  turn_to_angle(90)
  turn_to_angle(225)
  turn_to_angle(0)


def swing_test():
  left_swing_to_angle(90)
  right_swing_to_angle(0)


def full_test():
  drive_distance(24)
  turn_to_angle(-45)
  drive_distance(-36)
  right_swing_to_angle(-90)
  drive_distance(24)
  turn_to_angle(0)


def odom_test():
  set_coordinates(0, 0, 0)
  while(1):
    Brain.Screen.clearScreen()
    Brain.Screen.printAt(0,50, "X: %f", get_X_position())
    Brain.Screen.printAt(0,70, "Y: %f", get_Y_position())
    Brain.Screen.printAt(0,90, "Heading: %f", get_absolute_heading())
    Brain.Screen.printAt(0,110, "ForwardTracker: %f", get_ForwardTracker_position())
    Brain.Screen.printAt(0,130, "SidewaysTracker: %f", get_SidewaysTracker_position())
    wait(10, MSEC)  


def tank_odom_test():
  set_coordinates(0, 0, 0)
  drive_to_point(6, 18)
  turn_to_point(12,0, 180)
  drive_to_point(12, 0)
  turn_to_angle(100)
  drive_to_point(0, 0)


def holonomic_odom_test():
  set_coordinates(0, 0, 0)
  holonomic_drive_to_point(0, 18, 90)
  holonomic_drive_to_point(18, 0, 180)
  holonomic_drive_to_point(0, 18, 270)
  holonomic_drive_to_point(0, 0, 0)
