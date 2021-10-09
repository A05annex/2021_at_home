package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This subsystem is only the winch to pull in or let out the lifting hook
 */
public class RobotWinchSubsystem extends SubsystemBase {

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

  /**
   * The Singleton instance of this RobotWinchSubsystem. Code should use
   * the {@link #getInstance()} method to get the single instance (rather
   * than trying to construct an instance of this class.)
   */
  private final static RobotWinchSubsystem INSTANCE = new RobotWinchSubsystem();

  private final TalonSRX m_winchMotor = new TalonSRX(Constants.MotorControllers.WINCH);

  /**
   * Returns the Singleton instance of this RobotWinchSubsystem. This static method
   * should be used, rather than the constructor, to get the single instance
   * of this class. For example: {@code RobotWinchSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static RobotWinchSubsystem getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this RobotWinchSubsystem. This constructor
   * is private since this class is a Singleton. Code should use
   * the {@link #getInstance()} method to get the singleton instance.
   */
  private RobotWinchSubsystem() {
    // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    m_winchMotor.configFactoryDefault();
    m_winchMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setSpeed(double speed)
  {
    m_winchMotor.set(ControlMode.PercentOutput, speed);
  }
}

