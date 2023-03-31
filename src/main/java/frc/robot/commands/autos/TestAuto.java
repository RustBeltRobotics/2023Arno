package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveFollowTrajectory;

import static frc.robot.Constants.*;

public class TestAuto extends SequentialCommandGroup {
    public TestAuto() {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("TestPath", MAX_TRAJECTORY_VELOCITY, MAX_TRAJECTORY_ACCELERATION);
        addCommands(
                new DriveFollowTrajectory(path.get(0)),
                new DriveFollowTrajectory(path.get(1)));
    }
}
