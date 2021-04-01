package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


public class DriveToLimelightCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private double m_targetY;
    private double m_directionMult;

    public DriveToLimelightCommand(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem, double targetY) {
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.m_targetY = targetY;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        if (m_targetY > limelightSubsystem.getY()) {
            m_directionMult = 1.0;
        } else {
            m_directionMult = -1.0;
        }
    }

    @Override
    public void execute() {
        driveSubsystem.swerveDriveFieldRelative(0, 0.5 * m_directionMult, 0);
    }

    @Override
    public boolean isFinished() {
        if (m_directionMult == 1.0) {
            if (limelightSubsystem.getY() >= m_targetY) {
                return true;
            }
        } else {
            if (limelightSubsystem.getY() <= m_targetY) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.swerveDriveComponents(0,0, 0);
    }
}

