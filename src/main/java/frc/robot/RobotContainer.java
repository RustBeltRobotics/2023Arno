// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here.
    private final Drivetrain drivetrain = new Drivetrain();
    // private final Arm arm = new Arm();
    // private final Claw claw = new Claw();
    private final PhotonCamera frontCamera = new PhotonCamera("FrontCamera");

    // The drive team controllers are defined here.
    private final XboxController driverController = new XboxController(0);
    // private final XboxController operatorController = new XboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(drivetrain,
            () -> -Utilities.modifyAxis(driverController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND, // * speedModXY,
            () -> -Utilities.modifyAxis(driverController.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND, // * speedModXY,
            () -> -Utilities.modifyAxis(driverController.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)); // * speedModR));

        // Set up the default command for the arm.
        // Right stick Y axis -> arm rotation
        // Left stick Y axis -> arm extension
        // arm.setDefaultCommand(new DefaultArmCommand(arm,
        //     () -> -Utilities.modifyAxis(operatorController.getRightY()) * Constants.MAX_ARM_VELOCITY_DEGREES_PER_SECOND,
        //     () -> -Utilities.modifyAxis(operatorController.getLeftY()) * Constants.MAX_ARM_VELOCITY_INCHES_PER_SECOND));
        
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Driver Controller Bindings
        // A button zeros the gyroscope
        new Trigger(driverController::getAButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyroscope()));
        // X button toggles the wheel locks
        new Trigger(driverController::getXButton).onTrue(new InstantCommand(() -> drivetrain.toggleWheelsLocked()));
        // Y button toggles autobalance mode
        new Trigger(driverController::getYButton).onTrue(new InstantCommand(() -> drivetrain.toggleAutoBalance()));

        // Operator Controller Bindings
        // While the A button is pressed, the claw is open. When released, it closes.
        // new Trigger(driverController::getBButton).whileTrue(claw.openClaw()); // FIXME replace with operator controller
        
        // This is just a test method, to help with debugging by printing out arm and claw angles.
        // new Trigger(driverController::getRightBumper).onTrue(Commands.sequence(
            // new InstantCommand(() -> arm.printArm()),
            // new InstantCommand(() -> claw.printClaw())
        // ));
    }
}
