package frc.robot.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.BalanceRobot;

import static frc.robot.Constants.*;

public class ChargingStation1PBM extends SequentialCommandGroup {
    public ChargingStation1PBM() {

        addCommands(
                // Place Cube High
                new InstantCommand(() -> RobotContainer.intake.selectCube(), RobotContainer.intake),
                RobotContainer.arm.driveArmTo(() -> ARM_ANGLE_PRESET_SCORE[0][1], () -> ARM_EXTENSION_PRESET_SCORE[0][1]),
                new InstantCommand(() -> RobotContainer.intake.runIntake(1., false), RobotContainer.intake),
                new WaitCommand(.25),
                new InstantCommand(() -> RobotContainer.intake.zeroIntake(), RobotContainer.intake),
                RobotContainer.arm.centerArm(),

                // Drive to game piece and aquire
                new SequentialCommandGroup(
                    new InstantCommand(() -> RobotContainer.drivetrain.drive(new ChassisSpeeds(1., 0., 0.)), RobotContainer.drivetrain),
                    new WaitCommand(DRIVE_SECONDARY_CURRENT_LIMIT),
                    new InstantCommand(() -> RobotContainer.drivetrain.drive(new ChassisSpeeds(-1., 0., 0.)), RobotContainer.drivetrain),
                    new WaitCommand(DRIVE_SECONDARY_CURRENT_LIMIT),
                    new InstantCommand(() -> RobotContainer.drivetrain.drive(new ChassisSpeeds(0., 0., 0.)), RobotContainer.drivetrain)
                ),
                new WaitCommand(.25),

                // drive to balance
                new BalanceRobot()
            );
            // addCommands(
            //     // Place Cube High
            //     new InstantCommand(() -> RobotContainer.intake.selectCube(), RobotContainer.intake),
            //     RobotContainer.arm.driveArmTo(() -> ARM_ANGLE_PRESET_SCORE[0][1], () -> ARM_EXTENSION_PRESET_SCORE[0][1]),
            //     new InstantCommand(() -> RobotContainer.intake.runIntake(1., false), RobotContainer.intake),
            //     new WaitCommand(.25),
            //     new InstantCommand(() -> RobotContainer.intake.zeroIntake(), RobotContainer.intake),
            //     RobotContainer.arm.centerArm(),

            //     // Drive for mobility
            //     new DriveFollowTrajectory(path.get(0)),

            //     new WaitCommand(.25),

            //     // drive to balance
            //     new DriveFollowTrajectory(path.get(1)),
            //     new BalanceRobot()
            // );

    }
}
