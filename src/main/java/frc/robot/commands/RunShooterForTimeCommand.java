package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class RunShooterForTimeCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final double m_seconds;
    private long m_targetTime;
    private long m_currentTime;
    private double last_lower = 0.0;
    private double last_upper = 0.0;

    public RunShooterForTimeCommand(double seconds) {
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_shooterSubsystem);
        m_seconds = seconds;
    }

    @Override
    public void initialize() {
        m_targetTime = (long) (System.currentTimeMillis() + m_seconds*1000.0);
        m_currentTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        m_currentTime = System.currentTimeMillis();
        if (last_lower < Constants.SHOOTER_LOWER_SPEED) { // TODO: replace this with optimal speed for power port challenge shooting
            last_lower += Constants.RAMP_UP_INC;
        }
        m_shooterSubsystem.setLowerShooter(last_lower);

        if (last_upper < Constants.SHOOTER_UPPER_SPEED) { // TODO: replace this with optimal speed for power port challenge shooting
            last_upper += Constants.RAMP_UP_INC;
        }
        m_shooterSubsystem.setUpperShooter(last_upper);
    }

    @Override
    public boolean isFinished() {
        return m_currentTime >= m_targetTime;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setUpperShooter(0.0);
        m_shooterSubsystem.setLowerShooter(0.0);
    }
}
