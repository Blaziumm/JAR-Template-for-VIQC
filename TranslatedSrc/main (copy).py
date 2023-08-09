from vex import *
from drive import *
from odom import *
from PID import *
from util import *


#*---------------------------------------------------------------------------*/
#*                             VEXcode Config                                */
#*                                                                           */
#*  Before you do anything else, start by configuring your motors and        */
#*  sensors using the V5 port icon in the top right of the screen. Doing     */
#*  so will update robot-config.cpp and robot-config.h automatically, so     */
#*  you don't have to.                                                       */
#*---------------------------------------------------------------------------*/

#*---------------------------------------------------------------------------*/
#*                             JAR-Template Config                           */
#*                                                                           */
#*  Where all the magic happens. Follow the instructions below to input      */
#*  all the physical constants and values for your robot. You should         */
#*  already have configured your robot manually with the sidebar configurer. */
#*---------------------------------------------------------------------------*/

chassis(

#Specify your drive setup below. There are seven options:
#Keep this as ZERO_TRACKER:
ZERO_TRACKER,

#Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
#You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

#Left Motors:
motor_group(),

#Right Motors:
motor_group(),

#Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT1,

#Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

#External ratio, must be in decimal, in the format of input teeth/output teeth.
#If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
#If the motor drives the wheel directly, this value is 1:
1.6,

#Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
#For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

#*---------------------------------------------------------------------------*/
#*                                  PAUSE!                                   */
#*                                                                           */
#*  The rest of the drive constructor is for robots using POSITION TRACKING. */
#*  If you are not using position tracking, leave the rest of the values as  */
#*  they are.                                                                */
#*---------------------------------------------------------------------------*/

#PAUSE! The rest of the drive constructor is for robot using POSITION TRACKING.
#If you are not using position tracking, leave the rest of the values as they are.

#Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
#LF:      #RF:    
PORT1,     -PORT2,

#LB:      #RB: 
PORT3,     -PORT4,

#If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
#If this is a rotation sensor, leave it in "PORT1" format, inputting the port below.
#If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,

#Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

#Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
#This distance is in inches:
-2,

#Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,

#Sideways tracker diameter (reverse to make the direction switch):
-2.75,

#Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

#*---------------------------------------------------------------------------*/
#*                          Initilization Functions                          */
#*                                                                           */
#*  You may want to perform some actions before the competition starts.      */
#*  Do them in the following function.  You must return from this function   */
#*  or the autonomous and usercontrol tasks will not be started.  This       */
#*  function is only called once after the V5 has been powered on and        */
#*  not every time that the robot is disabled.                               */
#*---------------------------------------------------------------------------*/

current_auton_selection = 0;
auto_started = false;

def initialization():
  #Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();


#---------------------------------------------------------------------------*/
#                                                                           */
#                              User Control Task                            */
#                                                                           */
#  This task is used to control your robot during the user control phase of */
#  a VEX Competition.                                                       */
#                                                                           */
#  You must modify the code to add your own robot specific commands here.   */
#---------------------------------------------------------------------------*/

def usercontrol():
  # User control code here, inside the loop
  while (1=1) {
    # This is the main execution loop for the user control program.
    # Each time through the loop your program should update motor + servo
    # values based on feedback from the joysticks.

    # ........................................................................
    # Insert user code here. This is where you use the joystick values to
    # update your motors, etc.
    # ........................................................................

    #Replace this line with chassis.control_tank(); for tank drive.
    control_arcade();

    wait(20, msec); # Sleep the task for a short amount of time to
                    # prevent wasted resources.


#
# Main will set up the competition functions and callbacks.
#
int main() {
  # Run the pre-autonomous function.
  initialization()
  usercontrol()
  # Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
