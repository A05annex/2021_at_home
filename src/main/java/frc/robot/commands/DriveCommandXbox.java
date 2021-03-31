// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.NavX;
import frc.robot.subsystems.DriveSubsystem;
import org.a05annex.util.Utl;

public class DriveCommandXbox extends CommandBase {

  private final XboxController m_xbox;
  private final Joystick m_stick;
  private final DriveSubsystem m_driveSubsystem;
  private final NavX m_navx = NavX.getInstance();

  private double m_lastStickX = 0.0;
  private double m_lastStickY = 0.0;
  private double m_lastStickTwist = 0.0;

  /**
   * Drive using an xbox controller, with left stick Y being forward, left stick X being strafe,
   * and right stick X being rotate.
   */
  public DriveCommandXbox(XboxController xbox, Joystick stick, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_xbox = xbox;
    m_stick = stick;
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get stick values
    double stickY;
    double stickX;
    double stickTwist;
    if (Constants.USE_CONTROLLER.equals("XBOX")) {
      stickY = -m_xbox.getY(GenericHID.Hand.kLeft);
      stickX = m_xbox.getX(GenericHID.Hand.kLeft);
      stickTwist = m_xbox.getX(GenericHID.Hand.kRight);
    } else {
      // if not xbox, controller should be joystick
      stickY = -m_stick.getY();
      stickX = m_stick.getX();
      stickTwist = m_stick.getTwist();
    }
    // Limit the rate of change to reduce chance of break-away skidding.
    stickX = Utl.clip(stickX, m_lastStickX - Constants.DRIVE_MAX_SPEED_INC, m_lastStickX + Constants.DRIVE_MAX_SPEED_INC);
    stickY = Utl.clip(stickY, m_lastStickY - Constants.DRIVE_MAX_SPEED_INC, m_lastStickY + Constants.DRIVE_MAX_SPEED_INC);
    stickTwist = Utl.clip(stickTwist, m_lastStickTwist - Constants.DRIVE_MAX_ROTATE_INC, m_lastStickTwist + Constants.DRIVE_MAX_ROTATE_INC);
    m_lastStickX = stickX;
    m_lastStickY = stickY;
    m_lastStickTwist = stickTwist;



    // do deadband on speed
    double distance = Utl.length(stickY,stickX);
    double speed;
    if (distance < Constants.DRIVE_DEADBAND) {
      speed = 0.0;
    } else {
      if (distance > 1.0) {
        distance = 1.0;
      }
      speed = (distance - Constants.DRIVE_DEADBAND) / (1.0 - Constants.DRIVE_DEADBAND);
    }
    // add gain and sensitivity
    speed = Math.pow(speed, Constants.DRIVE_SPEED_SENSITIVITY) * Constants.DRIVE_SPEED_GAIN;

    // either do rotation with right stick, or PID to last expected heading
    double rotation;
    // Use the stick forward, strafe, twist as specified by the driver. NOTE: if the driver does
    // not specify a twist, then this command should be applying a rotation correction to maintain
    // the current heading.
    double rotMult = (stickTwist < 0.0) ? -1.0 : 1.0;
    stickTwist = Math.abs(stickTwist);
    // do deadband on rotation and track to heading if not touching right stick
    if (stickTwist < Constants.TWIST_DEADBAND) {
      // OK, no twist is specified - we should be trying to maintain the expected heading
      NavX.HeadingInfo headingInfo = m_navx.getHeadingInfo();
      if (null != headingInfo) {
        rotation = (headingInfo.expectedHeading - headingInfo.heading) * Constants.DRIVE_ORIENTATION_kP;
        // we noted that any time the robot stopped to wheels went to rotation only mode and started
        // hunting for the expected heading. As soon as we started, all the wheels were in the wrong
        // direction. Adding the speed multiplier (always positive, so it doesn't affect the direction)
        // means that when the robot stops, the wheels are directed as expected.
        rotation = Utl.clip(rotation,-0.5,0.5) * speed;
      } else {
        rotation = 0.0;
      }
    } else {
      // OK, twist is specified, we should be updating the expected heading to match the
      // current robot heading.
      rotation = (stickTwist - Constants.TWIST_DEADBAND) / (1.0 - Constants.TWIST_DEADBAND);
      m_navx.setExpectedHeadingToCurrent();
      // add sensitivity, gain and sign
      rotation = Math.pow(rotation, Constants.TWIST_SENSITIVITY) * Constants.TWIST_GAIN * rotMult;
    }
    // find direction, if the speed is 0 then it won't rotate
    double direction = Math.atan2(stickX, stickY);
    m_driveSubsystem.swerveDriveFieldRelative(direction, speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.swerveDriveComponents(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
