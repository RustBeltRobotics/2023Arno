package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.BalanceRobot;
import frc.robot.commands.DriveFollowTrajectory;
import java.util.List;

import static frc.robot.Constants.*;

public class HumanPlayerBalance extends SequentialCommandGroup {
    public HumanPlayerBalance() {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Human Player Balance", MAX_TRAJECTORY_VELOCITY, MAX_TRAJECTORY_ACCELERATION);

        addCommands(
                // Place Cube High
                new InstantCommand(() -> RobotContainer.intake.selectCube(), RobotContainer.intake),
                RobotContainer.arm.driveArmTo(() -> ARM_ANGLE_PRESET_SCORE[0][1], () -> ARM_EXTENSION_PRESET_SCORE[0][1]),
                new InstantCommand(() -> RobotContainer.intake.runIntake(1., false), RobotContainer.intake),
                new WaitCommand(.25),
                new InstantCommand(() -> RobotContainer.intake.zeroIntake(), RobotContainer.intake),
                RobotContainer.arm.centerArm(),

                new WaitCommand(.25),


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

                new WaitCommand(.5),

                // drive to balance
                new ParallelCommandGroup(
                    new DriveFollowTrajectory(path.get(1)),
                    new SequentialCommandGroup(
                        new InstantCommand(() -> RobotContainer.intake.zeroIntake(), RobotContainer.intake),
                        RobotContainer.arm.centerArm()
                    )
                ),
                new BalanceRobot()
            );
    }
}
