// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import org.a05annex.util.Utl;
import org.a05annex.util.geo2d.KochanekBartelsSpline;
import org.json.simple.JSONObject;
import org.json.simple.parser.ParseException;

import java.io.FileWriter;
import java.io.IOException;

import static org.a05annex.util.JsonSupport.*;
import static org.a05annex.util.Utl.TWO_PI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    public static final class MotorControllers {
        public static final int
                RF_DRIVE = 1,
                RF_SPIN = 2,
                RR_DRIVE = 3,
                RR_SPIN = 4,
                LR_DRIVE = 5,
                LR_SPIN = 6,
                LF_DRIVE = 7,
                LF_SPIN = 8;
    }

    public static final class AnalogPorts {
        public static final int
                RF = 0,
                RR = 1,
                LR = 2,
                LF = 3;
    }

    public static final class CalibrationOffset {
        public static final double
                RF = 0.916,
                RR = 0.980,
                LR = 0.774,
                LF = 0.853;
    }

    // This is the maximum velocity read by the encoders being used to control the drive speed. The actual
    // maximum is closer to 5700, but we have scaled that down a bit to account for friction, drag, lower
    // battery voltage, etc.
    public static final double MAX_DRIVE_VELOCITY = 5000;

    // length and width from center of the wheels, in m
    public static final double DRIVE_LENGTH = 0.590;
    public static final double DRIVE_WIDTH = 0.590;
    public static final double DRIVE_DIAGONAL = Utl.length(DRIVE_LENGTH, DRIVE_WIDTH);
    public static final double DRIVE_RADIUS = DRIVE_DIAGONAL / 2.0;
    // 18 motor revolutions to 1 spin of the drive wheel
    public static final double RADIANS_TO_SPIN_ENCODER = 18.0 / TWO_PI;

    // maximum linear speed and rotational speed. What we are really interested in knowing is how the robot
    // can travel or turn in one command cycle at full speed because for path following we need to know how
    // big to make the increments along the path, and need a pretty good estimate of where the robot is for
    // making course corrections.
    //
    // When tuning for path following, lots of things like: tire wear; changes in driving surface (plywood, concrete,
    // carpet); tire durometer, etc. change tire diameter and friction with the field surface, which, in turn
    // changes the maximum robot speed. We use this parameter to tune the robot to the field, especially for
    // autonomous paths. If the robot is overshooting, increase this (i.e. if the robot max speed is faster
    // then it will get there faster, and the requested speeds will be reduced and the path will be shorted); if
    // the robot is undershooting, then decrease it (i.e. if the robot max speed is smaller, then more speed will
    // be requested and the path will be lengthened).
    public static final double MAX_METERS_PER_SEC = 3.05;
    public static final double MAX_RADIANS_PER_SEC = MAX_METERS_PER_SEC / DRIVE_RADIUS;

    // PID values for the spin spark motor controller PID loop
    public static double SPIN_kP = 0.25;
    public static double SPIN_kI = 0.0;

    // PID values for the drive spark motor controller speed PID loop
    public static double DRIVE_kP = 0.00003;
    public static double DRIVE_kI = 0.000002;
    public static double DRIVE_kFF = 0.000174;
    public static double DRIVE_IZONE = 200.0;

    // PID values for the drive spark motor controller position PID loop
    public static double DRIVE_POS_kP = 0.13;
    public static double DRIVE_POS_kI = 0.0;

    // drive encoder tics per radian of robot rotation when rotation is controlled by position rather than speed.
    public static double DRIVE_POS_TICS_PER_RADIAN = 10.385;

    // driver enumerator
    public enum DRIVERS {
        NOLAN("Nolan", Filesystem.getDeployDirectory().toString() + "/drivers/nolan.json"),
        KALVIN("Kalvin", Filesystem.getDeployDirectory().toString() + "/drivers/kalvin.json"),
        PROGRAMMERS("programmers", Filesystem.getDeployDirectory().toString() + "/drivers/programmers.json");

        private static final String USE_CONTROLLER = "USE_CONTROLLER";
        private static final String XBOX_CONTROLLER = "XBOX";
        private static final String JOYSTICK_CONTROLLER = "JOYSTICK";

        private static final String DRIVE_DEADBAND = "DRIVE_DEADBAND";
        private static final String DRIVE_SPEED_SENSITIVITY = "DRIVE_SPEED_SENSITIVITY";
        private static final String DRIVE_SPEED_GAIN = "DRIVE_SPEED_GAIN";
        private static final String TWIST_DEADBAND = "TWIST_DEADBAND";
        private static final String TWIST_SENSITIVITY = "TWIST_SENSITIVITY";
        private static final String TWIST_GAIN = "TWIST_GAIN";

        String m_driverName;
        String m_driverFile;

        DRIVERS(String driverName, String driverFile) {
            m_driverName = driverName;
            m_driverFile = driverFile;
        }

        public void load() {
            try {
                JSONObject dict = readJsonFileAsJSONObject(m_driverFile);
                if (null != dict) {
                    // Read in the driver data
                    frc.robot.Constants.USE_CONTROLLER = parseString(dict, USE_CONTROLLER, frc.robot.Constants.USE_CONTROLLER);
                    frc.robot.Constants.DRIVE_DEADBAND = parseDouble(dict, DRIVE_DEADBAND, frc.robot.Constants.DRIVE_DEADBAND);
                    frc.robot.Constants.DRIVE_SPEED_SENSITIVITY = parseDouble(dict, DRIVE_SPEED_SENSITIVITY, frc.robot.Constants.DRIVE_SPEED_SENSITIVITY);
                    frc.robot.Constants.DRIVE_SPEED_GAIN = parseDouble(dict, DRIVE_SPEED_GAIN, frc.robot.Constants.DRIVE_SPEED_GAIN);
                    frc.robot.Constants.TWIST_DEADBAND = parseDouble(dict, TWIST_DEADBAND, frc.robot.Constants.TWIST_DEADBAND);
                    frc.robot.Constants.TWIST_SENSITIVITY = parseDouble(dict, TWIST_SENSITIVITY, frc.robot.Constants.TWIST_SENSITIVITY);
                    frc.robot.Constants.TWIST_GAIN = parseDouble(dict, TWIST_GAIN, frc.robot.Constants.TWIST_GAIN);
                }

            } catch (IOException | ParseException | ClassCastException | NullPointerException e) {
                e.printStackTrace();
            }
        }

        public static void save() {
            JSONObject dict = new JSONObject();
            dict.put(USE_CONTROLLER, frc.robot.Constants.USE_CONTROLLER);
            dict.put(DRIVE_DEADBAND, frc.robot.Constants.DRIVE_DEADBAND);
            dict.put(DRIVE_SPEED_SENSITIVITY, frc.robot.Constants.DRIVE_SPEED_SENSITIVITY);
            dict.put(DRIVE_SPEED_GAIN, frc.robot.Constants.DRIVE_SPEED_GAIN);
            dict.put(TWIST_DEADBAND, frc.robot.Constants.TWIST_DEADBAND);
            dict.put(TWIST_SENSITIVITY, frc.robot.Constants.TWIST_SENSITIVITY);
            dict.put(TWIST_GAIN, frc.robot.Constants.TWIST_GAIN);
            try (FileWriter file = new FileWriter(frc.robot.Constants.currentDriver.m_driverFile)) {
                file.write(dict.toJSONString());
                file.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        public static void setDriverAtID(int ID) {
            switch (ID) {
                case 0:
                    frc.robot.Constants.currentDriver = NOLAN;
                    break;
                case 1:
                    frc.robot.Constants.currentDriver = KALVIN;
                    break;
                case 2:
                    frc.robot.Constants.currentDriver = PROGRAMMERS;
                    break;
                default:
                    frc.robot.Constants.currentDriver = KALVIN;
            }
            frc.robot.Constants.currentDriver.load();
        }

        public static void switchControlScheme() {
            if (frc.robot.Constants.USE_CONTROLLER.equals(JOYSTICK_CONTROLLER)) {
                frc.robot.Constants.USE_CONTROLLER = XBOX_CONTROLLER;
            } else {
                // if not stick, it's xbox
                frc.robot.Constants.USE_CONTROLLER = JOYSTICK_CONTROLLER;
            }
        }
    }

    // used in DriveCommand
    public static DRIVERS currentDriver = DRIVERS.KALVIN;

    public static String USE_CONTROLLER = DRIVERS.JOYSTICK_CONTROLLER;

    public static double DRIVE_DEADBAND = 0.1;
    public static double DRIVE_SPEED_SENSITIVITY = 2.0;
    public static double DRIVE_SPEED_GAIN = 0.5;

    public static double TWIST_DEADBAND = 0.1;
    public static double TWIST_SENSITIVITY = 2.0;
    public static double TWIST_GAIN = 1.0;

    // small number for zero check
    public static final double SMALL = 0.000001;

    // temp variable used in DriveDistance
    public static double DRIVE_SPEED = 0.0;

    // PID values for rotation to target using a PID on current heading and target direction to keep ths robot
    // oriented to the target while driving.
    public static double TARGET_kP = 0.5;

    public static double DRIVE_ORIENTATION_kP = 0.8;
    // Maximum change in speed in 1 command cycle
    public static double DRIVE_MAX_SPEED_INC = 0.2;
    // Maximum change in rotation in 1 command cycle
    public static double DRIVE_MAX_ROTATE_INC = 0.2;

    public enum AutonomousPath {
        BARREL_RACING("Barrel Racing", 0, "2021_barrel_racing_3.json"),
        SLALOM("Slalom", 1, "2021_slalom_3.json"),
        BOUNCE("Bounce", 2, "2021_bounce.json"),
        LIGHTSPEED("Short Slow Slalom", 3, "2021_slalom_short_1.json"),
        CAL_CIRCLE_1("cal circle 1", 4, "cal_circle_1.json"),
        CAL_CIRCLE_2("cal circle 2", 5, "cal_circle_2.json"),
        CAL_STRAIGHT("cal straight", 6, "cal_straight.json"),
        CAL_ROTATE("cal rotate", 7, "cal_rotate.json");

        static AutonomousPath AUTONOMOUS_PATH = AutonomousPath.CAL_STRAIGHT;

        private final String m_pathName;
        private final int m_id;
        private final String m_filename;

        AutonomousPath(String skill, int id, String filename) {
            m_pathName = skill;
            m_id = id;
            m_filename = filename;
        }

        public static String getName() {
            return AUTONOMOUS_PATH.m_pathName;
        }

        /**
         * Load this autonomous path.
         *
         * @return The loaded path, {@code null} if the path could npot be loaded.
         */
        public static KochanekBartelsSpline load() {
            KochanekBartelsSpline spline = new KochanekBartelsSpline();
            if (spline.loadPath(Filesystem.getDeployDirectory().toString() + "/paths/" +
                    AUTONOMOUS_PATH.m_filename)) {
                return spline;
            } else {
                return null;
            }
        }

        public static void setAutonomousToId(int id) {
            AUTONOMOUS_PATH = AutonomousPath.CAL_STRAIGHT;
            for (AutonomousPath path : values()) {
                if (path.m_id == id) {
                    AUTONOMOUS_PATH = path;
                    break;
                }
            }
        }
    }
}
