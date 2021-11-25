/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.a05annex.util.Utl;

public class LimelightSubsystem extends SubsystemBase implements IGetTargetError{
  /**
   * The Singleton instance of this ShooterSubsystem. Code should use
   * the {@link #getInstance()} method to get the single instance (rather
   * than trying to construct an instance of this class.)
   * <p>
   * The INSTANCE field is volatile to ensure that multiple threads
   * offer it correctly when it is being initialized by ensuring changes
   * made in one thread are immediately reflected in other threads.
   */
  private volatile static LimelightSubsystem INSTANCE;

  /**
   * Returns the Singleton instance of this ShooterSubsystem. This static method
   * should be used, rather than the constructor, to get the single instance
   * of this class. For example: {@code ShooterSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static LimelightSubsystem getInstance() {
    // Use "Double Checked Locking" to ensure thread safety and best performance
    // Fast (non-synchronized) check to reduce overhead of acquiring a lock when it's not needed
    if (INSTANCE == null) {
      // Lock to make thread safe
      synchronized (LimelightSubsystem.class) {
        // check nullness again as multiple threads can reach and
        // pass the initial non-synchronized null check above
        if (INSTANCE == null) {
          INSTANCE = new LimelightSubsystem();
        }
      }
    }
    return INSTANCE;
  }

    double m_x;
    double m_y;
    double m_a;

  /**
   * Creates a new LimelightSubsystem.
   */
  public LimelightSubsystem() {
      updateVisionData();
      setPipeline(Constants.PIPELINE_SHOOTER); //PIPELINE_SHOOTER);
  }

  private void updateVisionData() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    m_x = tx.getDouble(0.0) * (Utl.PI/180);
    m_y = ty.getDouble(0.0) * (Utl.PI/180);
    m_a = ta.getDouble(0.0);
  }

  public double getX() {
      return m_x;
  }

  public double getY() {
      return m_y;
  }

  public double getA() {
      return m_a;
  }

  public void setPipeline(int pipeline) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("pipeline").setNumber(pipeline);
    // Disable vision for driver camera, otherwise do vision
    if (pipeline == Constants.PIPELINE_DRIVER) {
      table.getEntry("camMode").setNumber(1);
    } else {
      table.getEntry("camMode").setNumber(0);
    }
  }

  public double distanceToTarget() {
    return (Constants.TARGET_HEIGHT - Constants.LIMELIGHT_Z_OFFSET) /
            Math.tan(Constants.LIMELIGHT_ELEVATION_ANGLE.getRadians() + m_y);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateVisionData();
  }

  public double GetTargetHeadingError() {
    return m_x - Constants.LIMELIGHT_OFFSET;
  }
}