package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class RunShooter extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem;
    private double m_speed;

    /**
     * Runs both shooter motors at the given power. When interrupted, set both power to 0.
     * @param shooterSubsystem The shooter subsystem.
     * @param speed (double) The speed to run at, between -1.0 and 1.0.
     */
    public RunShooter(ShooterSubsystem shooterSubsystem, double speed) {
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        m_shooterSubsystem = shooterSubsystem;
        m_speed = speed;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //TODO: set these to use m_speed
        m_shooterSubsystem.setUpperShooter(Constants.SHOOTER_SPEED);
        m_shooterSubsystem.setLowerShooter(Constants.SHOOTER_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setUpperShooter(0.0);
        m_shooterSubsystem.setLowerShooter(0.0);
    }
}
