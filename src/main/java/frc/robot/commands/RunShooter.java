package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class RunShooter extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem;
    private double m_speed;

    /**
     * Runs shooter motors at the powers defined in constants. When interrupted, set both power to 0.
     * @param shooterSubsystem The shooter subsystem.
     */
    public RunShooter(ShooterSubsystem shooterSubsystem) {
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //TODO: set these to use m_speed
        m_shooterSubsystem.setUpperShooter(Constants.SHOOTER_UPPER_SPEED);
        m_shooterSubsystem.setLowerShooter(Constants.SHOOTER_LOWER_SPEED);
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
