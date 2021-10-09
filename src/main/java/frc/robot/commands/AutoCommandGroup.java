package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import org.a05annex.util.geo2d.KochanekBartelsSpline;

public class AutoCommandGroup extends SequentialCommandGroup {

    public AutoCommandGroup(KochanekBartelsSpline spline, DriveSubsystem driveSubsystem) {
        super(new FollowPathCommand(spline, driveSubsystem), new AutoLimelightAdjustCommand(driveSubsystem),
                new AutoShootCommand());
        // Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());

    }

}