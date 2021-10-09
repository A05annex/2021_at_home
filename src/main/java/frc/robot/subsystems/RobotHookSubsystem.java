package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The subsystem that controls the pneumatics for the hook deployment.
 */
public class RobotHookSubsystem extends SubsystemBase {

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

  /**
   * The Singleton instance of this RobotHookSubsystem. Code should use
   * the {@link #getInstance()} method to get the single instance (rather
   * than trying to construct an instance of this class.)
   */
  private final static RobotHookSubsystem INSTANCE = new RobotHookSubsystem();

  private final Solenoid m_hookSolenoid = new Solenoid(Constants.Pneumatics.HOOK);
  /**
   * Returns the Singleton instance of this RobotHookSubsystem. This static method
   * should be used, rather than the constructor, to get the single instance
   * of this class. For example: {@code RobotHookSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static RobotHookSubsystem getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this RobotHookSubsystem. This constructor
   * is private since this class is a Singleton. Code should use
   * the {@link #getInstance()} method to get the singleton instance.
   */
  private RobotHookSubsystem() {
    // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.
  }

  public void liftHook() {
    m_hookSolenoid.set(true);
  }

  public void dropHook() {
    m_hookSolenoid.set(false);
  }
}

