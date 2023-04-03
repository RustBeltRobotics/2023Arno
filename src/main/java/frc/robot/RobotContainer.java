package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.autos.CableCone2P;
import frc.robot.commands.autos.CableCube2P;
import frc.robot.commands.autos.ChargingStation1PB;
import frc.robot.commands.autos.ChargingStation1PBM;
import frc.robot.commands.autos.HumanPlayer3P;
import frc.robot.commands.autos.HumanPlayerCone2P;
import frc.robot.commands.autos.HumanPlayerCube2P;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

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

    // The drive team controllers are defined here
    public static final XboxController driverController = new XboxController(0);
    public static final XboxController operatorController = new XboxController(1);

    // Limits maximum speed
    private double maxSpeedFactor = .5;

    private int selectedRow = 0;
    private int selectedCol = 0;

    private boolean[][] gridStatus = {
        {false, false, false, false, false, false, false, false, false},
        {false, false, false, false, false, false, false, false, false},
        {false, false, false, false, false, false, false, false, false},
    };

    private boolean [][] subGridStatus = {
        {false, false, false},
        {false, false, false},
        {false, false, false}
    };

    private String chunk = "";

    private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

    public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Set up the default command for the drivetrain
        // The controls are for field-oriented driving
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation        
        drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(drivetrain,
            () -> -modifyAxis(driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
            () -> -modifyAxis(driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND * maxSpeedFactor,
            () -> -modifyAxis(driverController.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * maxSpeedFactor));

        // Set up the default command for the arm
        // Right stick Y axis -> arm rotation
        // Left stick Y axis -> arm extension
        arm.setDefaultCommand(new DefaultArmCommand(arm,
                () -> modifyAxis(operatorController.getLeftY()) * ARM_ROTATION_TRAINING_WHEELS,
                () -> modifyAxis(operatorController.getLeftX()) * ARM_EXTENSION_TRAINING_WHEELS));
        
        // Set up the default command for the intake
        // Left trigger -> intake
        // Right trigger -> outtake
        intake.setDefaultCommand(new DefaultIntakeCommand(intake,
                () -> modifyAxis(operatorController.getRightTriggerAxis()),
                () -> modifyAxis(operatorController.getLeftTriggerAxis())));

        // Configure the button bindings
        configureButtonBindings();
        configureAutos();
    }

    /** Button -> command mappings are defined here. */
    private void configureButtonBindings() {
        // Driver Controller Bindings
        // Pressing A button zeros the gyroscope
        new Trigger(driverController::getAButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyroscope()));
        // Pressing Y button locks the wheels in an X pattern
        new Trigger(driverController::getYButton).onTrue(new InstantCommand(() -> drivetrain.toggleWheelsLocked()));
        // Pressing the Right Bumper shifts to high speed
        new Trigger(driverController::getRightBumper).onTrue(new InstantCommand(() -> speedUp()));
        // Pressing the Left Bumper shifts to low speed
        new Trigger(driverController::getLeftBumper).onTrue(new InstantCommand(() -> speedDown()));
        
        // DriveToPose triggers (currently unused)
        // // Pressing Y button drives to the selected scoring position
        // new Trigger(driverController::getYButton).onTrue(new DriveToPose(() -> DRIVE_X_PRESET_SCORE, () -> DRIVE_Y_PRESET_SCORE[selectedCol], () -> 0.));
        // // Pressing B button drives to the single human player station
        // new Trigger(driverController::getBButton).onTrue(new DriveToPose(() -> DRIVE_X_PRESET_HUMANPLAYER[0], () -> DRIVE_Y_PRESET_HUMANPLAYER[0], () -> 90.));
        // // Pressing back button drives to the left double human player station
        // new Trigger(driverController::getBackButton).onTrue(new DriveToPose(() -> DRIVE_X_PRESET_HUMANPLAYER[1], () -> DRIVE_Y_PRESET_HUMANPLAYER[1], () -> 0.));
        // // Pressing start button drives to the right double human player station
        // new Trigger(driverController::getStartButton).onTrue(new DriveToPose(() -> DRIVE_X_PRESET_HUMANPLAYER[2], () -> DRIVE_Y_PRESET_HUMANPLAYER[2], () -> 0.));
 

        // Operator Controller Bindings
        // Holding A button retracts and centers the arm
        new Trigger(operatorController::getAButton).whileTrue(arm.centerArm());
        // Holding Y button rotates and extends arm to the currently selecting scoring position
        new Trigger(operatorController::getYButton).whileTrue(arm.driveArmTo(() -> ARM_ANGLE_PRESET_SCORE[selectedRow][intake.gamepieceIsCube ? 1 : 0], () -> ARM_EXTENSION_PRESET_SCORE[selectedRow][intake.gamepieceIsCube ? 1 : 0]));
        // Holding B button rotates and extends arm to the chute human player station position
        new Trigger(operatorController::getBButton).whileTrue(arm.driveArmTo(() -> ARM_ANGLE_PRESET_HUMANPLAYER[0], () -> ARM_EXTENSION_PRESET_HUMANPLAYER[0]));
        // Holding B button rotates and extends arm to the chute human player station position
        new Trigger(operatorController::getXButton).whileTrue(arm.driveArmTo(() -> ARM_ANGLE_PRESET_HUMANPLAYER[1], () -> ARM_EXTENSION_PRESET_HUMANPLAYER[1]));
        // Holding back button rotates and extends arm to ground pickup from back position
        new Trigger(operatorController::getBackButton).whileTrue(arm.driveArmTo(() -> ARM_ANGLE_PRESET_GROUND_PICKUP[0], () -> ARM_EXTENSION_PRESET_GROUND_PICKUP[0]));
        // Holding start button rotates and extends arm to ground pickup from front position
        new Trigger(operatorController::getStartButton).whileTrue(arm.driveArmTo(() -> ARM_ANGLE_PRESET_GROUND_PICKUP[1], () -> ARM_EXTENSION_PRESET_GROUND_PICKUP[1]));
        // Pressing Left Bumper selects a cone as the next gamepiece
        new Trigger(operatorController::getLeftBumper).onTrue(new InstantCommand(() -> intake.selectCone()));
        // Pressing Right Bumper selects a cube as the next gamepiece
        new Trigger(operatorController::getRightBumper).onTrue(new InstantCommand(() -> intake.selectCube()));
        // D-pad is used to select a scoring node
        new Trigger(() -> operatorController.getPOV() != -1).onTrue(new InstantCommand(() -> changeSelectedScoreLocation(operatorController.getPOV())));
        // Clicking the right stick zeros the arm encoder
        new Trigger(operatorController::getRightStickButton).onTrue(new InstantCommand(() -> arm.resetEncoders()));
    }

    public void configureAutos() {
        autoChooser.setDefaultOption("Do nothing", new WaitCommand(1.)); 
        autoChooser.addOption("Cable Cone 2P", new CableCone2P());
        autoChooser.addOption("Cable Cube 2P", new CableCube2P());
        autoChooser.addOption("Charging Station 1PB", new ChargingStation1PB());
        autoChooser.addOption("Charging Station 1PBM", new ChargingStation1PBM());
        autoChooser.addOption("Human Player 3P", new HumanPlayer3P());
        autoChooser.addOption("Human Player Cone 2P", new HumanPlayerCone2P());
        autoChooser.addOption("Human Player Cube 2P", new HumanPlayerCube2P());

        matchTab.add(autoChooser).withPosition(2, 3);
    }

    /**
     * This method returns the autonomous routine that will be run at the start of
     * the match.
     * <p>
     * For now, we only have one routine, so it just returns that one.
     * Once we have more than one routine, we will want to implement a chooser
     * dropdown on the dashboard.
     * 
     * @return The autonomous routine that will be run
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void speedUp() {
        maxSpeedFactor += .1;
        maxSpeedFactor = MathUtil.clamp(maxSpeedFactor, .1, 1.);
    }

    public void speedDown() {
        maxSpeedFactor -= .1;
        maxSpeedFactor = MathUtil.clamp(maxSpeedFactor, .1, 1.);
    }

    /** Increment the selected row and column in the direction of the POV */
    public void changeSelectedScoreLocation(int pov) {
        // Set the previous location to false
        gridStatus[selectedRow][selectedCol] = false;
        subGridStatus[selectedRow][selectedCol % 3] = false;

        if (pov == 0) {
            selectedRow += -1;
            selectedCol += 0;
        } else if (pov == 45) {
            selectedRow += -1;
            selectedCol += 1;
        } else if (pov == 90) {
            selectedRow += 0;
            selectedCol += 1;
        } else if (pov == 135) {
            selectedRow += 1;
            selectedCol += 1;
        } else if (pov == 180) {
            selectedRow += 1;
            selectedCol += 0;
        } else if (pov == 225) {
            selectedRow += 1;
            selectedCol += -1;
        } else if (pov == 270) {
            selectedRow += 0;
            selectedCol += -1;
        } else if (pov == 315) {
            selectedRow += -1;
            selectedCol += -1;
        }

        // Clamp the values inside the bounds of the grid (3 high by 9 wide)
        selectedRow = MathUtil.clamp(selectedRow, 0, 2);
        selectedCol = MathUtil.clamp(selectedCol, 0, 8);

        // Set the next location to true
        gridStatus[selectedRow][selectedCol] = true;
        subGridStatus[selectedRow][selectedCol % 3] = true;

        if (selectedCol / 3 == 0) {
            chunk = "Left";
        } else if (selectedCol / 3 == 1) {
            chunk = "Center";
        } else if (selectedCol / 3 == 2) {
            chunk = "Right";
        }
    }

    public boolean[][] getSubGridStatus() {
        return subGridStatus;
    }

    public String getChunk() {
        return chunk;
    }
}
