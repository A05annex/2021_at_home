package frc.robot.util;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.runner.JUnitPlatform;
import org.junit.runner.RunWith;

import static org.junit.jupiter.api.Assertions.*;

// NOTE: it is expected that this class will move into either the a05annexUtil project or the a05annexRobot
// project. To accomplish this there must be no includes of any robot-specific classes like Constants.java,
// Robot.java, etc. if wpilib classes are included then this class can only be promoted to the a05annexRobot
// library.
/**
 * Test the {@link RobotTelemetry} methods that use the {@link LimelightTelemetry} class to compute expected
 * limelight angles for a specified robot field location, and use the limelight angles and robot heading
 * to compute the field location of the robot.
 */
@RunWith(JUnitPlatform.class)
public class TestRobotLimelightTelemetry {
}
