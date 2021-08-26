package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


public class ShootCameraCommand extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final LimelightSubsystem m_limelightSubsystem = LimelightSubsystem.getInstance();

    /**
     * Runs shooter motors at the powers defined in constants. When interrupted, set both power to 0.
     */
    public ShootCameraCommand() {
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem);
        addRequirements(m_shooterSubsystem);
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double distance = m_limelightSubsystem.distanceToTarget();
        double upperSpeed = 0.0;
        double lowerSpeed = 0.0;

        m_shooterSubsystem.setUpperShooter(upperSpeed);
        m_shooterSubsystem.setLowerShooter(lowerSpeed);
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
