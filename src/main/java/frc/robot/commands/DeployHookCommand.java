package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RobotHookSubsystem;


public class DeployHookCommand extends CommandBase {
  private final RobotHookSubsystem robotHookSubsystem = RobotHookSubsystem.getInstance();

  private final boolean m_deploy;

  public DeployHookCommand(boolean deploy) {
    // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.robotHookSubsystem);
    m_deploy = deploy;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (m_deploy) {
      RobotHookSubsystem.getInstance().liftHook();
    } else {
      RobotHookSubsystem.getInstance().dropHook();
    }
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {

  }
}
