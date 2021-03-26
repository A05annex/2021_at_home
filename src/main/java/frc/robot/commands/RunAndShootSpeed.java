package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterPneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class RunAndShootSpeed extends CommandBase {
  private final ShooterPneumaticSubsystem m_shooterPneumaticSubsystem = ShooterPneumaticSubsystem.getInstance();
  private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
  private boolean m_shootFlag;
  private boolean m_doneShootingFlag;
  private int m_ticksElapsed;

  public RunAndShootSpeed() {
    // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(m_shooterPneumaticSubsystem, m_shooterSubsystem);
  }

  @Override
  public void initialize() {
    m_shootFlag = false;
    m_doneShootingFlag = false;
    m_ticksElapsed = 0;
  }

  @Override
  public void execute() {
    m_shooterSubsystem.setUpperShooter(Constants.SHOOTER_UPPER_SPEED);
    m_shooterSubsystem.setLowerShooter(Constants.SHOOTER_LOWER_SPEED);

    if (m_shooterSubsystem.isLowerReady() && m_shooterSubsystem.isUpperReady() && !m_shootFlag) {
      m_shootFlag = true;
      m_shooterPneumaticSubsystem.liftShooter();
    }

    if (m_shootFlag && !m_doneShootingFlag) {
      m_ticksElapsed++;
      if (m_ticksElapsed >= Constants.SHOOT_TICKS) {
        m_doneShootingFlag = true;
        m_shooterPneumaticSubsystem.dropShooter();
      }
    }
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setUpperShooter(0.0);
    m_shooterSubsystem.setLowerShooter(0.0);
    m_shooterPneumaticSubsystem.dropShooter();
  }
}
