package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveFollowTrajectory;
import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.*;

public class HumanPlayerCone2P extends SequentialCommandGroup {
    public HumanPlayerCone2P() {
        SmartDashboard.putString("Alliance Color:" , DriverStation.getAlliance().toString());
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Human Player Cone 2P", MAX_TRAJECTORY_VELOCITY, MAX_TRAJECTORY_ACCELERATION);
        List<PathPlannerTrajectory> newPath = new ArrayList<PathPlannerTrajectory>();
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            for (PathPlannerTrajectory trajR : path) {
                trajR = PathPlannerTrajectory.transformTrajectoryForAlliance(trajR, DriverStation.getAlliance());
                newPath.add(trajR);
                path = newPath;
            }
        }
        else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            for (PathPlannerTrajectory trajB : path) {
                trajB = PathPlannerTrajectory.transformTrajectoryForAlliance(trajB, DriverStation.getAlliance());
                newPath.add(trajB);
                path = newPath;
            }
        }

        addCommands(
                // Place Cone High
                new InstantCommand(() -> RobotContainer.intake.selectCone(), RobotContainer.intake),
                RobotContainer.arm.centerArm(),
                RobotContainer.arm.driveArmTo(() -> ARM_ANGLE_PRESET_SCORE[0][0], () -> ARM_EXTENSION_PRESET_SCORE[0][0]),
                new InstantCommand(() -> RobotContainer.intake.runIntake(1., false), RobotContainer.intake),
                new WaitCommand(.25),
                new InstantCommand(() -> RobotContainer.intake.zeroIntake(), RobotContainer.intake),
                RobotContainer.arm.centerArm(),

                // Drive to game piece and aquire
                new ParallelCommandGroup(
                    new DriveFollowTrajectory(path.get(0)),
                    new SequentialCommandGroup(
                        new InstantCommand(() -> RobotContainer.intake.selectCube(), RobotContainer.intake),
                        new WaitCommand(1.),
                        new InstantCommand(() -> RobotContainer.intake.runIntake(1., true), RobotContainer.intake),
                        RobotContainer.arm.driveArmTo(() -> ARM_ANGLE_PRESET_GROUND_PICKUP[1], () -> ARM_EXTENSION_PRESET_GROUND_PICKUP[1])
                    )
                ),

                new WaitCommand(.25),

                // drive to score
                new ParallelCommandGroup(
                    new DriveFollowTrajectory(path.get(1)),
                    new SequentialCommandGroup(
                        new InstantCommand(() -> RobotContainer.intake.zeroIntake(), RobotContainer.intake),
                        RobotContainer.arm.centerArm(),
                        new WaitCommand(1.5),
                        RobotContainer.arm.driveArmTo(() -> ARM_ANGLE_PRESET_SCORE[0][0], () -> ARM_EXTENSION_PRESET_SCORE[0][0])
                    )
                ),
                new InstantCommand(() -> RobotContainer.intake.runIntake(1., false), RobotContainer.intake),
                new WaitCommand(.25),
                new InstantCommand(() -> RobotContainer.intake.zeroIntake(), RobotContainer.intake),
                RobotContainer.arm.centerArm()

            );
    }
}
