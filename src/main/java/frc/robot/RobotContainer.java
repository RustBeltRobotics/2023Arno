package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoBalanceRoutine;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

import static frc.robot.Constants.*;
import static frc.robot.Utilities.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems are defined here
    public static final Drivetrain drivetrain = new Drivetrain();
    public static final Arm arm = new Arm();
    public static final Intake intake = new Intake();
    public static final Vision vision = new Vision();

    // The drive team controllers are defined here
    public static final XboxController driverController = new XboxController(0);
    public static final XboxController operatorController = new XboxController(1);

    // Limits allowable acceleration
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter rLimiter;

    // Limits maximum speed
    private double maxSpeedFactor = MAX_SPEED_FACTOR_LOW;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Limits allowable acceleration
        xLimiter = new SlewRateLimiter(1. / TIME_TO_MAX_X);
        yLimiter = new SlewRateLimiter(1. / TIME_TO_MAX_Y);
        rLimiter = new SlewRateLimiter(1. / TIME_TO_MAX_R);

        // Set up the default command for the drivetrain
        // The controls are for field-oriented driving
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation        
        drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(drivetrain,
            () -> -modifyAxis(xLimiter.calculate(driverController.getLeftY())) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
            () -> -modifyAxis(yLimiter.calculate(driverController.getLeftX())) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
            () -> -modifyAxis(rLimiter.calculate(driverController.getRightX())) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * maxSpeedFactor));

        // Set up the default command for the arm
        // Right stick Y axis -> arm rotation
        // Left stick Y axis -> arm extension
        arm.setDefaultCommand(new DefaultArmCommand(arm,
                () -> -modifyAxis(operatorController.getRightY()) * 1.,
                () -> -modifyAxis(operatorController.getLeftY()) * 1.));
        
        // Set up the default command for the intake
        // Left trigger -> intake
        // Right trigger -> outtake
        intake.setDefaultCommand(new DefaultIntakeCommand(intake,
                () -> modifyAxis(operatorController.getRightY()),
                () -> modifyAxis(operatorController.getLeftY())));

        // Configure the button bindings
        configureButtonBindings();
    }

    /** Button -> command mappings are defined here. */
    private void configureButtonBindings() {
        // Driver Controller Bindings
        // Pressing A button zeros the gyroscope
        new Trigger(driverController::getAButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyroscope()));
        // Pressing X button toggles the wheel locks
        new Trigger(driverController::getXButton).onTrue(new InstantCommand(() -> drivetrain.toggleWheelsLocked()));
        // Pressing Y button toggles autobalance mode
        new Trigger(driverController::getYButton).onTrue(new InstantCommand(() -> drivetrain.toggleAutoBalance()));
        // Pressing the Right Bumper shifts to high speed
        new Trigger(driverController::getRightBumper).onTrue(new InstantCommand(() -> speedUp()));
        // Pressing the Left Bumper shifts to low speed
        new Trigger(driverController::getLeftBumper).onTrue(new InstantCommand(() -> speedDown()));

        // Operator Controller Bindings
        // Pressing Left Bumper selects a cone as the next gamepiece
        new Trigger(operatorController::getLeftBumper).onTrue(new InstantCommand(() -> intake.selectCone()));
        // Pressing Right Bumper selects a cube as the next gamepiece
        new Trigger(operatorController::getRightBumper).onTrue(new InstantCommand(() -> intake.selectCube()));

        // new Trigger(operatorController::getAButton).whileTrue(arm.driveRotationTo(0.));

        // new Trigger(operatorController::getBButton).whileTrue(arm.driveRotationTo(90.));

        // new Trigger(operatorController::getXButton).whileTrue(arm.driveRotationTo(-90.));


        // new Trigger(operatorController::getAButton).whileTrue(arm.centerArm());

        // new Trigger(operatorController::getBButton).whileTrue(arm.driveArmTo(90., 15.));

        // new Trigger(operatorController::getXButton).whileTrue(arm.driveArmTo(-90., 15.));
    }

    /**
     * This method retunrs the autonomous routine that will be run at the start of
     * the match.
     * <p>
     * For now, we only have one routine, so it just returns that one.
     * Once we have more than one routine, we will want to implement a chooser
     * dropdown on the dashboard.
     * 
     * @return The autonomous routine that will be run
     */
    public Command getAutonomousCommand() {
        return new AutoBalanceRoutine(drivetrain);
    }

    public void speedUp() {
        maxSpeedFactor = MAX_SPEED_FACTOR_HIGH;
    }

    public void speedDown() {
        maxSpeedFactor = MAX_SPEED_FACTOR_LOW;
    }
}
