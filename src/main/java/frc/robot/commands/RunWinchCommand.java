package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RobotWinchSubsystem;


public class RunWinchCommand extends CommandBase {
  private final RobotWinchSubsystem robotWinchSubsystem = RobotWinchSubsystem.getInstance();

  private double m_speed;

  public RunWinchCommand(double speed) {
    // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.robotWinchSubsystem);
    m_speed = speed;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    RobotWinchSubsystem.getInstance().setSpeed(m_speed);

  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    RobotWinchSubsystem.getInstance().setSpeed(0.0);
  }
}
