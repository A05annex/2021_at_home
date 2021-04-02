package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectorSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this CollectorSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     * <p>
     * The INSTANCE field is volatile to ensure that multiple threads
     * offer it correctly when it is being initialized by ensuring changes
     * made in one thread are immediately reflected in other threads.
     */
    private volatile static CollectorSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this CollectorSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code CollectorSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static CollectorSubsystem getInstance() {
        // Use "Double Checked Locking" to ensure thread safety and best performance
        // Fast (non-synchronized) check to reduce overhead of acquiring a lock when it's not needed
        if (INSTANCE == null) {
            // Lock to make thread safe
            synchronized (CollectorSubsystem.class) {
                // check nullness again as multiple threads can reach and
                // pass the initial non-synchronized null check above
                if (INSTANCE == null) {
                    INSTANCE = new CollectorSubsystem();
                }
            }
        }
        return INSTANCE;
    }

    private final TalonSRX m_collector = new TalonSRX(Constants.MotorControllers.SWEEPER);

    /**
     * Creates a new instance of this CollectorSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private CollectorSubsystem() {
        m_collector.configFactoryDefault();
        m_collector.setNeutralMode(NeutralMode.Coast);
    }

    public void setCollectorPower(double power) {
        m_collector.set(ControlMode.PercentOutput, power);
    }
}

