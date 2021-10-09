package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterPneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoShootCommand extends CommandBase {
    private final long m_shootTimeInc = 2000L;
    private long m_currentTime;
    private long m_shootTime1;
    private long m_shootTime2;
    private long m_shootTime3;
    private int m_timeFlag; // keeps track of how many balls already shot
    private int m_shootFlag; // 0 = not shooting, 1 = lift shooter, 2 = shooter already lifted
    private int m_ticksElapsed;
    private final ShooterPneumaticSubsystem m_shooterPneumaticSubsystem = ShooterPneumaticSubsystem.getInstance();
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();

    public AutoShootCommand() {
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_shooterPneumaticSubsystem, m_shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_currentTime = System.currentTimeMillis();
        m_shootTime1 = (m_currentTime + m_shootTimeInc);
        m_shootTime2 = (m_currentTime + m_shootTimeInc*2);
        m_shootTime3 = (m_currentTime + m_shootTimeInc*3);
        m_timeFlag = 0;
        m_shootFlag = 0;
    }

    @Override
    public void execute() {
        m_currentTime = System.currentTimeMillis();
        m_shooterSubsystem.setUpperShooter(Constants.SHOOTER_UPPER_SPEED);
        m_shooterSubsystem.setLowerShooter(Constants.SHOOTER_LOWER_SPEED);

        if (m_currentTime >= m_shootTime1 && m_timeFlag < 1) {
            m_shootFlag = 1;
            m_timeFlag = 1;
        }
        if (m_currentTime >= m_shootTime2 && m_timeFlag < 2) {
            m_shootFlag = 1;
            m_timeFlag = 2;
        }
        if (m_currentTime >= m_shootTime3 && m_timeFlag < 3) {
            m_shootFlag = 1;
            m_timeFlag = 3;
        }

        updateShooter();
    }

    @Override
    public boolean isFinished() {
        return m_timeFlag == 3 && m_shootFlag == 0;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setUpperShooter(0.0);
        m_shooterSubsystem.setLowerShooter(0.0);
        m_shooterPneumaticSubsystem.dropShooter();
    }

    private void updateShooter() {
        if (m_shootFlag == 1) { // runs once
            m_shooterPneumaticSubsystem.liftShooter();
            m_ticksElapsed = 0;
            m_shootFlag = 2;
        } else if (m_shootFlag == 2) { // runs repeatedly
            m_ticksElapsed++;
            if (m_ticksElapsed >= Constants.SHOOT_TICKS) {
                m_shooterPneumaticSubsystem.dropShooter();
                m_shootFlag = 0;
            }
        }
    }
}
