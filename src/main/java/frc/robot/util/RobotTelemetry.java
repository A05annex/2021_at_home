package frc.robot.util;

import org.a05annex.util.AngleD;
import org.a05annex.util.geo3d.Line3d;
import org.a05annex.util.geo3d.Point3d;
import org.a05annex.util.geo3d.Vector3d;
import org.a05annex.util.geo3d.Xfm4x4d;
import org.jetbrains.annotations.NotNull;

// NOTE: it is expected that this class will move into either the a05annexUtil project or the a05annexRobot
// project. To accomplish this there must be no includes of any robot-specific classes like Constants.java,
// Robot.java, etc. if wpilib classes are included then this class can only be promoted to the a05annexRobot
// library.
/**
 * This is a class for maintaining robot geometry using the limelight. The primary things this class does are:
 * <ul>
 *     <li>Maintain the transformations from field to robot and robot to field coordinate systems;</li>
 *     <li>Maintain a representation of the limelight;</li>
 *     <li>Given a field target location and a robot position on the field, compute the limelight
 *         angles to the target (good for verifying robot/limelight geometry, and for testing)</li>
 *     <li>Given limelight angles and the robot heading, compute the position ofthe robot on the field.</li>
 * </ul>
 */
public class RobotTelemetry {

    // The robot coordinates
    static final int CENTER = 0;
    static final int RF = 1;
    static final int LF = 2;
    static final int LR = 3;
    static final int RR = 4;
    static final int LIMELIGHT = 5;

    static final String[] GEOMETRY_NAMES = {"center     : ", "right front: ", "left front : ",
            "left rear  : ", "right rear : ", "limelight  : " };
    static final Point3d[] ROBOT_GEOMETRY = {
            new Point3d(0.0, 0.0, 0.0), new Point3d(0.1, 0.3, 0.0),
            new Point3d(-0.1, 0.3, 0.0), new Point3d(-0.1, -0.3, 0.0),
            new Point3d(0.1, -0.3, 0.0),
            new Point3d(0.0, 0.0, 0.0)};

    double m_fieldX = 0.0;
    double m_fieldY = 0.0;
    final AngleD m_heading = new AngleD(AngleD.RADIANS, 0.0);

    final Xfm4x4d m_robotToFieldXfm = new Xfm4x4d();
    final Xfm4x4d m_fieldToRobotXfm = new Xfm4x4d();

    final LimelightTelemetry m_limelight = new LimelightTelemetry();

    /**
     * Instantiate a {@code RobotTelemetry} object initialized to center field.
     */
    public RobotTelemetry()
    {

    }

    /**
     * Instantiate a {@code RobotTelemetry} object initialized to  a specific location on the field.
     *
     * @param fieldX The X location of the robot on the field.
     * @param fieldY The Y location of the robot on the field.
     * @param heading The heading of the robot.
     */
    public RobotTelemetry(double fieldX, double fieldY, @NotNull AngleD heading)
    {
        setFieldPosition(fieldX, fieldY, heading);
    }

    /**
     * Set the position of the robot on the field.
     *
     * @param fieldX The X location of the robot on the field.
     * @param fieldY The Y location of the robot on the field.
     * @param heading The heading of the robot.
     */
    public void setFieldPosition(final double fieldX, final double fieldY, @NotNull final AngleD heading)
    {
        m_fieldX = fieldX;
        m_fieldY = fieldY;
        m_heading.setValue(heading);
        // build the transform that positions the robot relative to the field
        // by initializing the transform to an identity, applying the heading
        // rotation, and then translating the robot to some position on the field.
        m_robotToFieldXfm.identity();
        m_robotToFieldXfm.rotate(Xfm4x4d.AXIS_Z, heading.cloneAngleD().mult(-1.0));
        m_robotToFieldXfm.translate(fieldX, fieldY, 0.0);
        // invert that to buld the transform that describes where the field
        // elements are relative to the robot
        m_robotToFieldXfm.invert(m_fieldToRobotXfm);
    }

    /**
     * Set the position of the robot on the field based on the limelight sensing of a target
     *
     * @param heading The robot heading
     * @param targetId The identifier of the target the limelight is sensing. This is the ID of the
     *                 target when it was registered using the
     *                 {@link #addLimelightTarget(String, Point3d, Vector3d, double, double)} method.
     * @param limelightX The X angle to the target centroid as reported by the limelight.
     * @param limelightY The Y angle to the target centroid as reported by the limelight.
     */
    public void setFieldPositionFromLimelight(@NotNull AngleD heading, @NotNull String targetId,
                                              @NotNull AngleD limelightX, @NotNull AngleD limelightY)
    {
        // Make sure the target exists, and get the position of the target relative to
        // the robot. NOTE: we have not yet considered the heading of the robot on the field,
        // so this information is marginally useful at this point.
        LimelightTelemetry.Target target = m_limelight.getTarget(targetId);
        LimelightTelemetry.TargetPosition targetPosition = m_limelight.getTargetRelativeToRobot(
                targetId, limelightX, limelightY);

        // Now we work on making the information from the limelight meaningful:
        // set the field position of the robot so the only meaningful thing is rotations - we are only
        // transforming vectors after this, so only the rotation part of the transform is important.
        setFieldPosition(0.0, 0.0, heading);
        // get that vector from the field position of the limelight to the target by transforming
        // the robotToTarget vector from robot to field space.
        Vector3d fieldToTarget = xfmRobotVecToField(targetPosition.robotToTarget);

        // Reverse the vector so that it is from the target centroid to the limelight center
        Vector3d targetToField = fieldToTarget.cloneVector3d().reverse();

        // use the target to limelight vector (in field space) and the target distance to compute the
        // limelight center on the field.
        Point3d limelightCenter = new Line3d(target.m_centroid, targetToField).
                pointAtDistance(targetPosition.targetDistance);

        // get the vector from the center of the limelight to the center of the robot - note, this vector
        //  is not normalized.
        Vector3d limelightCtrToRobotCtr =
                new Vector3d(m_limelight.getLocationOnRobot(), new Point3d(0.0,0.0,0.0));

        // Transform the limelight ro robot vector by the robot heading so that it is a vector in field
        // space that accounts for the orientation (heading) of the robot
        Vector3d fieldLimelightToRobot = xfmRobotVecToField(limelightCtrToRobotCtr);

        //
        Point3d robotCenter = new Line3d(limelightCenter, fieldLimelightToRobot).pointAtDistance(1.0);

        // Now that we know where the robot center is we can set that for this instance.
        setFieldPosition(robotCenter.getX(), robotCenter.getY(), heading);
    }

    /**
     * Set the limelight position on the robot.
     *
     * @param robotX    The X position of the limelight on the robot.
     * @param robotY    The Y position of the limelight on the robot.
     * @param robotZ    The Z position of the limelight on the robot.
     * @param heading   The heading of the limelight relative to the robot.
     * @param elevation The elevation of the limelight relative to the robot
     */
    public void setLimelightPosition(double robotX, double robotY, double robotZ,
                                     @NotNull AngleD heading, @NotNull AngleD elevation)
    {
        m_limelight.setLimelightPosition(robotX, robotY, robotZ, heading, elevation);
        ROBOT_GEOMETRY[LIMELIGHT].setValue(robotX, robotY, robotZ);
    }

    /**
     *
     */
    public int addLimelightTarget(@NotNull String identifier, @NotNull Point3d centroid, @NotNull Vector3d normal,
                                  double width, double height)
    {
        return m_limelight.addTarget(identifier,  centroid, normal, width, height);
    }

    /**
     * Transform a field-relative point to robot-relative point. This takes afield-relative point, and
     * creates a robot-relative point. The field-relative point is unchanged.
     */
    public Point3d xfmFieldPtToRobot(Point3d fieldPt)
    {
        return m_fieldToRobotXfm.transform(fieldPt, new Point3d());
    }

    /**
     * Transform a field-relative array of points to robot-relative
     * array of points. This takes an array of field-relative points, and
     * creates an array of robot-relative points. The field-relative
     * points are unchanged.
     */
    public Point3d[] xfmFieldPtsToRobot(Point3d[] fieldPts)
    {
        return m_fieldToRobotXfm.cloneAndTransform(fieldPts);
    }

    /**
     * Transform a robot-relative vector to a field-relative vector and
     * return the field-relative vector. This is useful if you want to
     * display a robot direction on a diagram of the field.
     */
    public Vector3d xfmRobotVecToField(Vector3d robotVec)
    {
        return m_robotToFieldXfm.transform(robotVec, new Vector3d());
    }

    /**
     * Transform a robot-relative point to a field-relative point and
     * return the field-relative point. This is useful if you want to
     * display the robot on a diagram of the field.
     */
    public Point3d xfmRobotPtToField(Point3d robotPt)
    {
        return m_robotToFieldXfm.transform(robotPt, new Point3d());
    }

    /**
     * Transform the robot-relative geometry to field-relative coordinates and
     * return those field-relative coordinates. This is useful if you want to
     * display the robot on a diagram of the field.
     */
    public Point3d[] xfmRobotPtsToField()
    {
        return m_robotToFieldXfm.cloneAndTransform(ROBOT_GEOMETRY);
    }

    /**
     * Print an annotated list of the robot geometry transformed to field-relative
     * coordinates for the current field position. This is primarily a debugging method.
     */
    public void printRobotPtsToField()
    {
        Point3d[] pts = xfmRobotPtsToField();
        for (int i = 0; i < pts.length; i++) {
            System.out.println(String.format("%s (%9.3f,%9.3f,%9.3f)", GEOMETRY_NAMES[i],
                    pts[i].getX(), pts[i].getY(), pts[i].getZ()));
        }
    }
}
