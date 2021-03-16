package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this ShooterSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     * <p>
     * The INSTANCE field is volatile to ensure that multiple threads
     * offer it correctly when it is being initialized by ensuring changes
     * made in one thread are immediately reflected in other threads.
     */
    private volatile static ShooterSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this ShooterSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code ShooterSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ShooterSubsystem getInstance() {
        // Use "Double Checked Locking" to ensure thread safety and best performance
        // Fast (non-synchronized) check to reduce overhead of acquiring a lock when it's not needed
        if (INSTANCE == null) {
            // Lock to make thread safe
            synchronized (ShooterSubsystem.class) {
                // check nullness again as multiple threads can reach and
                // pass the initial non-synchronized null check above
                if (INSTANCE == null) {
                    INSTANCE = new ShooterSubsystem();
                }
            }
        }
        return INSTANCE;
    }

    private final TalonSRX m_lowerShooter = new TalonSRX(Constants.MotorControllers.SHOOTER_LOWER);
    private final TalonSRX m_upperShooter = new TalonSRX(Constants.MotorControllers.SHOOTER_UPPER);
    private double m_lastSetLowerSpeed;
    private double m_lastSetUpperSpeed;

    /**
     * Creates a new instance of this ShooterSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ShooterSubsystem() {
        m_lowerShooter.configFactoryDefault();
        m_lowerShooter.setNeutralMode(NeutralMode.Coast);
        m_lowerShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_lowerShooter.setSensorPhase(true);
        m_lowerShooter.setInverted(true);
        m_upperShooter.configFactoryDefault();
        m_upperShooter.setNeutralMode(NeutralMode.Coast);
        m_upperShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_upperShooter.setSensorPhase(true);
        m_upperShooter.setInverted(true);
        updateAllPID();
        m_lastSetUpperSpeed = 0.0;
        m_lastSetLowerSpeed = 0.0;
    }

    public void updateAllPID() {
        setTalonPID(m_lowerShooter, Constants.SHOOTER_kP, Constants.SHOOTER_kI, Constants.SHOOTER_kF_LOWER);
        setTalonPID(m_upperShooter, Constants.SHOOTER_kP, Constants.SHOOTER_kI, Constants.SHOOTER_kF_UPPER);
    }

    public void setTalonPID(TalonSRX talon, double kP, double kI, double kF) {
        talon.config_kP(0, kP);
        talon.config_kI(0, kI);
        talon.config_kF(0, kF);
    }

    public void setLowerShooter(double speed) {
        m_lastSetLowerSpeed = Constants.MAX_LOWER_SHOOTER_RPM * speed;
        m_lowerShooter.set(ControlMode.Velocity, m_lastSetLowerSpeed);
    }

    public void setUpperShooter(double speed) {
        m_lastSetUpperSpeed = Constants.MAX_UPPER_SHOOTER_RPM * speed;
        m_upperShooter.set(ControlMode.Velocity, m_lastSetUpperSpeed);
    }

    public double getUpperShooterSpeed() {
        return m_upperShooter.getSelectedSensorVelocity();
    }

    public double getLowerShooterSpeed() {
        return m_lowerShooter.getSelectedSensorVelocity();
    }

    public boolean isLowerReady() {
        if (getLowerShooterSpeed() > m_lastSetLowerSpeed * Constants.SPINUP_THRESHOLD) {
            return true;
        }
        return false;
    }

    public boolean isUpperReady() {
        if (getUpperShooterSpeed() > m_lastSetUpperSpeed * Constants.SPINUP_THRESHOLD) {
            return true;
        }
        return false;
    }

}

