package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

public class DriveToTarget extends InstantCommand {

    PathPlannerTrajectory trajectory;
    double xTranslation;
    double yTranslation;
    double rotation;

    public DriveToTarget(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
        xTranslation = xSupplier.getAsDouble();
        yTranslation = ySupplier.getAsDouble();
        rotation = rotationSupplier.getAsDouble();
    }

    @Override
    public void initialize() {
        Translation2d goalTranslation = new Translation2d(xTranslation, yTranslation);
        Rotation2d goalRotation = Rotation2d.fromDegrees(rotation);

        Pose2d startPosition = RobotContainer.drivetrain.getPose();

        PathConstraints constraints = new PathConstraints(MAX_TRAJECTORY_VELOCITY, MAX_TRAJECTORY_ACCELERATION);
        List<EventMarker> eventMarkers = new ArrayList<EventMarker>();

        PathPoint goalPose = new PathPoint(goalTranslation, Rotation2d.fromDegrees(0.), goalRotation, 0);
        PathPoint startPose = new PathPoint(startPosition.getTranslation(), startPosition.getRotation());

        trajectory = PathPlanner.generatePath(constraints, eventMarkers, startPose, goalPose);

        (new DriveFollowTrajectory(trajectory)).schedule();
    }
}
