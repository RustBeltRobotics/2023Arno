package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.BalanceRobot;
import frc.robot.commands.DriveFollowTrajectory;
import java.util.List;

import static frc.robot.Constants.*;

public class ChargingStation1PBM extends SequentialCommandGroup {
    public ChargingStation1PBM() {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Charging Station 1PBM", MAX_BALANCE_TRAJECTORY_VELOCITY, MAX_BALANCE_TRAJECTORY_ACCELERATION);

        addCommands(
                // Place Cube High
                new InstantCommand(() -> RobotContainer.intake.selectCube(), RobotContainer.intake),
                RobotContainer.arm.driveArmTo(() -> ARM_ANGLE_PRESET_SCORE[0][1], () -> ARM_EXTENSION_PRESET_SCORE[0][1]),
                new InstantCommand(() -> RobotContainer.intake.runIntake(1., false), RobotContainer.intake),
                new WaitCommand(.25),
                new InstantCommand(() -> RobotContainer.intake.zeroIntake(), RobotContainer.intake),
                RobotContainer.arm.centerArm(),

                new WaitCommand(.25),

                // drive to balance   
                new DriveFollowTrajectory(path.get(0)),
                new DriveFollowTrajectory(path.get(1)),
                new BalanceRobot()
            );
    }
}
