# 2021_at_home - Robot Rodeo - 9-Oct-2021

Added a lift mechanism, added code for that and for real autonomous.

## Driver Control

The xbox controller is the primary drive control. Buttons on the Joystick
provide the secondary control for shooting, ball pickup, and lift.

### XBox Primary Driver

The control is field relative. Face the controller downfield, and the
robot will move (translate with no rotation) in the direction of the
left-hand stick motion. Right stick side-to-side controls rotation.

When the left bumper is pressed, right stick will be ignored and the
limelight will be used to orient the robot so the shooter tracks the
target.

Other buttons:
* right bumper - shoot (lift a ball into the shooter).
* button B - run the shooter while button B is pressed.

### Joystick Secondary Controller

* Button 9 - hook pneumatics down (down by default when the robot is disabled).
* Button 10 - hook pneumatics up (in position to latch on the bar).
* Button 11 - unwind the lift winch while pressed, stops when you release.
* Button 12 - continue to lift robot while pressed, stops when you release.