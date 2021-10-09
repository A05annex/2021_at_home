package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.NavX;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OdometryTargetError;


public class AutoLimelightAdjustCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final LimelightSubsystem m_limelightSubsystem = LimelightSubsystem.getInstance();
    private final NavX m_navx = NavX.getInstance();
    private double m_targetHeadingError;
    private int m_ticksElapsed;

    public AutoLimelightAdjustCommand(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        m_targetHeadingError = m_limelightSubsystem.GetTargetHeadingError();
        m_ticksElapsed = 0;
    }

    @Override
    public void execute() {
        m_targetHeadingError = m_limelightSubsystem.GetTargetHeadingError();
        m_driveSubsystem.setHeading(m_driveSubsystem.getFieldHeading() + m_targetHeadingError);
        m_navx.setExpectedHeadingToCurrent();
        m_ticksElapsed++;
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        if (m_targetHeadingError > 0) {
            return m_targetHeadingError <= Constants.LIMELIGHT_RADIAN_THRESHOLD
                    || m_ticksElapsed >= Constants.LIMELIGHT_ADJUST_TIMEOUT;
        } else {
            return m_targetHeadingError >= Constants.LIMELIGHT_RADIAN_THRESHOLD
                    || m_ticksElapsed >= Constants.LIMELIGHT_ADJUST_TIMEOUT;
        }

    }

    @Override
    public void end(boolean interrupted) {

    }
}
