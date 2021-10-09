// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.commands.SetLimelightPipeline;
import org.a05annex.util.geo2d.KochanekBartelsSpline;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // subsystems
  private final DriveSubsystem m_driveSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  //private final DriveCommand m_driveCommand;
  private final DriveCommandXbox m_driveCommandXbox;
  private final OdometryTargetError m_odometryTargetError;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ShooterPneumaticSubsystem m_shooterPneumaticSubsystem;
  private final CollectorSubsystem m_collectorSubsystem;

  // commands

  // controllers
  private final XboxController m_xbox = new XboxController(0);
  private final Joystick m_stick = new Joystick(1);

  // buttons
  private final POVButton m_xboxDpadUp = new POVButton(m_xbox, 0);
  private final POVButton m_xboxDpadLeft = new POVButton(m_xbox, 270);
  private final POVButton m_xboxDpadDown = new POVButton(m_xbox, 180);
  private final POVButton m_xboxDpadRight = new POVButton(m_xbox, 90);
  private final JoystickButton m_xboxA = new JoystickButton(m_xbox, 1);
  private final JoystickButton m_xboxB = new JoystickButton(m_xbox, 2);
  private final JoystickButton m_xboxX = new JoystickButton(m_xbox, 3);
  private final JoystickButton m_xboxY = new JoystickButton(m_xbox, 4);
  private final JoystickButton m_xboxLeftBumper = new JoystickButton(m_xbox, 5); // used for pointing at target
  private final JoystickButton m_xboxRightBumper = new JoystickButton(m_xbox, 6);

  private final JoystickButton m_button3 = new JoystickButton(m_stick, 3);
  private final JoystickButton m_button4 = new JoystickButton(m_stick, 4);
  private final JoystickButton m_button5 = new JoystickButton(m_stick, 5);
  private final JoystickButton m_button6 = new JoystickButton(m_stick, 6);
  private final JoystickButton m_button7 = new JoystickButton(this.m_stick, 7);
  private final JoystickButton m_button8 = new JoystickButton(this.m_stick, 8);
  private final JoystickButton m_button9 = new JoystickButton(this.m_stick, 9);
  private final JoystickButton m_button10 = new JoystickButton(this.m_stick, 10);
  private final JoystickButton m_button11 = new JoystickButton(this.m_stick, 11);
  private final JoystickButton m_button12 = new JoystickButton(this.m_stick, 12);

  // Digital input switchboard
  private DigitalInput switch0 = new DigitalInput(0);
  private DigitalInput switch1 = new DigitalInput(1);
  private DigitalInput switch2 = new DigitalInput(2);
  private DigitalInput switch3 = new DigitalInput(3);
  private DigitalInput switch4 = new DigitalInput(4);

  private FollowPathCommand m_autonomousCommand = null;
  private KochanekBartelsSpline m_autonomousSpline = null;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // subsystems
    m_driveSubsystem = new DriveSubsystem();
    m_odometryTargetError = new OdometryTargetError(m_driveSubsystem);
    m_shooterSubsystem = ShooterSubsystem.getInstance();
    m_shooterPneumaticSubsystem = ShooterPneumaticSubsystem.getInstance();
    m_limelightSubsystem = LimelightSubsystem.getInstance();
    m_collectorSubsystem = CollectorSubsystem.getInstance();

    // commands
    //m_driveCommand = new DriveCommand(m_stick, m_driveSubsystem);
    m_driveCommandXbox = new DriveCommandXbox(m_xbox, m_stick, m_driveSubsystem, m_limelightSubsystem);

    // set default commands
    //m_driveSubsystem.setDefaultCommand(m_driveCommand);
    m_driveSubsystem.setDefaultCommand(m_driveCommandXbox);

    // set the default autonomous command -
    Constants.AutonomousPath.setAutonomousToId(readAutoID());
    m_autonomousSpline = Constants.AutonomousPath.load();
    if ( m_autonomousSpline != null) {
      m_autonomousCommand = new FollowPathCommand(m_autonomousSpline, m_driveSubsystem);
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // m_xboxA.whenPressed(new FollowPathCommand(Filesystem.getDeployDirectory().toString() + "/figure_eight_path.json", m_driveSubsystem));
//    m_xboxA.whenPressed(new InstantCommand(m_shooterPneumaticSubsystem::liftShooter, m_shooterSubsystem));
//    m_xboxB.whenPressed(new InstantCommand(m_shooterPneumaticSubsystem::dropShooter, m_shooterSubsystem));
//    m_xboxA.whenPressed(new ShootCommand(m_shooterPneumaticSubsystem));
//    m_xboxRightBumper.whenHeld(new RunAndShootCommand());
//    m_xboxA.whenPressed(new FollowPathCommand(Constants.AutonomousPath.load(), m_driveSubsystem));

    m_xboxRightBumper.whenPressed(new ShootCommand());
//    m_xboxA.whenPressed(new RunShooterForTimeCommand(60.0));
    m_xboxB.whenHeld(new RunShooter());
//    m_xboxY.whenHeld(new ShootCameraCommand());
//    m_xboxB.whenPressed(new RunCollectorCommand(1.0));
    m_xboxY.whenPressed(new RunCollectorCommand(0.0));
    m_xboxX.whenPressed(new RunCollectorCommand(-1.0 * Constants.COLLECTOR_POWER));
    m_xboxA.whenPressed(new RunCollectorCommand(Constants.COLLECTOR_POWER));

    m_button5.whenPressed(new InstantCommand(Constants::bumpUpperShooterSpeedPlus));
    m_button3.whenPressed(new InstantCommand(Constants::bumpUpperShooterSpeedMinus));
    m_button6.whenPressed(new InstantCommand(Constants::bumpLowerShooterSpeedPlus));
    m_button4.whenPressed(new InstantCommand(Constants::bumpLowerShooterSpeedMinus));
    m_button12.whenHeld(new RunWinchCommand(1.0));  // winch robot up
    m_button11.whenHeld(new RunWinchCommand(-1.0)); // winch robot down
    m_button10.whenPressed(new DeployHookCommand(true));
    m_button9.whenPressed(new DeployHookCommand(false));
//    m_button8.whenPressed(new InstantCommand(Constants::bumpUpperkFPlus));
//    m_button7.whenPressed(new InstantCommand(Constants::bumpUpperkFMinus));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public FollowPathCommand getAutonomousCommand() {
    // In this method is called it means the robot are getting ready to run an autonomous path. Before we run tha
    // path we need to make sure the NavX is initialized to the robot heading (which may not be 0.0) and that
    // the swerve drive modules are prepared (oriented in the right direction) for the first command in the path.

    return m_autonomousCommand;
  }

  public int readDriverID() {
    return (switch0.get() ? 0 : 1) + (switch1.get() ? 0 : 2);
  }

  public int readAutoID() {
    return (switch2.get() ? 0 : 1) + (switch3.get() ? 0 : 2) + (switch4.get() ? 0 : 4);
  }

  public XboxController getXbox() {
    return m_xbox;
  }

  public Joystick getStick() {
    return m_stick;
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_driveSubsystem;
  }
}
