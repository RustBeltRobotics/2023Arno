package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveFollowTrajectory;

import static frc.robot.Constants.*;

public class TestAuto extends SequentialCommandGroup {
    public TestAuto() {
        SmartDashboard.putString("Alliance Color:" , DriverStation.getAlliance().toString());
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("TestPath", MAX_TRAJECTORY_VELOCITY, MAX_TRAJECTORY_ACCELERATION);
        List<PathPlannerTrajectory> newPath = new ArrayList<PathPlannerTrajectory>();
        if(DriverStation.getAlliance() == DriverStation.Alliance.Red ){
            for (PathPlannerTrajectory trajR : path) {
                trajR = PathPlannerTrajectory.transformTrajectoryForAlliance(trajR, DriverStation.getAlliance());
                newPath.add(trajR);
                path = newPath;
            }
        }
        else if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
            for (PathPlannerTrajectory trajB : path) {
                trajB = PathPlannerTrajectory.transformTrajectoryForAlliance(trajB, DriverStation.getAlliance());
                newPath.add(trajB);
                path = newPath;
            }
        }
        else{
        } 
        addCommands(
                new DriveFollowTrajectory(path.get(0)),
                new DriveFollowTrajectory(path.get(1)));
    }
}
