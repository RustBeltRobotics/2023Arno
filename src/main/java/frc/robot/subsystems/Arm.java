package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase {
    // Motor controllers
    private final CANSparkMax rotationSparkMaxLeft;
    private final CANSparkMax rotationSparkMaxRight;
    private final CANSparkMax extensionSparkMax;

    // Motor built-in encoders
    private final RelativeEncoder rotationLeftEncoder;
    private final RelativeEncoder rotationRightEncoder;
    private final RelativeEncoder extensionEncoder;

    // Absolute CANEncoder
    private final CANCoder rotationAbsoluteEncoder;
 
    // PID controllers
    private final PIDController rotationPID;
    private final PIDController extensionPID;
 
    // Feedforward controllers
    private final ArmFeedforward rotationFF;
    private final ArmFeedforward extensionFF;

    // private final Timer timer;

    public Arm() {
        // Setup the SparkMax objects and encoders. Note that we set all of the
        // important settings in code, even though in theory, these should all be burned
        // to the flash memory already. This is a redundancy for if we forget to burn
        // settings if we need to replace a broken unit.
        rotationSparkMaxLeft = new CANSparkMax(LEFT_ARM_MOTOR, MotorType.kBrushless);
        rotationSparkMaxLeft.restoreFactoryDefaults();
        rotationSparkMaxLeft.setIdleMode(IdleMode.kBrake);
        rotationSparkMaxLeft.setInverted(false);
        rotationSparkMaxLeft.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        rotationSparkMaxLeft.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);
        rotationSparkMaxLeft.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_ANGLE_DEGREES);
        rotationSparkMaxLeft.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_ANGLE_DEGREES);
        rotationSparkMaxLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
        rotationSparkMaxLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
        rotationLeftEncoder = rotationSparkMaxLeft.getEncoder();
        rotationLeftEncoder.setPositionConversionFactor(ARM_ROTATION_CONVERSION);
        rotationLeftEncoder.setVelocityConversionFactor(ARM_ROTATION_CONVERSION / 60.);
        
        rotationSparkMaxRight = new CANSparkMax(RIGHT_ARM_MOTOR, MotorType.kBrushless);
        rotationSparkMaxRight.restoreFactoryDefaults();
        rotationSparkMaxRight.setIdleMode(IdleMode.kBrake); 
        rotationSparkMaxRight.setInverted(true);
        rotationSparkMaxRight.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        rotationSparkMaxRight.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);
        rotationSparkMaxRight.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_ANGLE_DEGREES);
        rotationSparkMaxRight.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_ANGLE_DEGREES);
        rotationSparkMaxRight.enableSoftLimit(SoftLimitDirection.kForward, true);
        rotationSparkMaxRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
        rotationRightEncoder = rotationSparkMaxRight.getEncoder();
        rotationRightEncoder.setPositionConversionFactor(ARM_ROTATION_CONVERSION);
        rotationRightEncoder.setVelocityConversionFactor(ARM_ROTATION_CONVERSION / 60.);
        
        extensionSparkMax = new CANSparkMax(ARM_EXTENSION_MOTOR, MotorType.kBrushless);
        extensionSparkMax.restoreFactoryDefaults();
        extensionSparkMax.setIdleMode(IdleMode.kBrake);
        extensionSparkMax.setInverted(true);
        extensionSparkMax.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        extensionSparkMax.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);
        extensionSparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_EXTENSION_INCHES);
        extensionSparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_EXTENSION_INCHES);
        extensionSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
        extensionSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
        extensionEncoder = extensionSparkMax.getEncoder();
        extensionEncoder.setPositionConversionFactor(ARM_EXTENSION_CONVERSION);
        extensionEncoder.setVelocityConversionFactor(ARM_EXTENSION_CONVERSION / 60.);

        // Setup steer motor relative encoder
        rotationAbsoluteEncoder = new CANCoder(ARM_ABSOLUTE_ENCODER);
        rotationAbsoluteEncoder.configMagnetOffset(ARM_ABSOLUTE_OFFSET);   
        rotationAbsoluteEncoder.configSensorDirection(false);
        rotationAbsoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        
        // Setup PID and feedforward controllers
        rotationPID = new PIDController(ARM_ROTATION_P, ARM_ROTATION_I, ARM_ROTATION_D);
        rotationPID.setTolerance(ARM_ROTATION_TOLERANCE);

        extensionPID = new PIDController(ARM_EXTENSION_P, ARM_EXTENSION_I, ARM_EXTENSION_D);
        extensionPID.setTolerance(ARM_EXTENSION_TOLERANCE);

        rotationFF = new ArmFeedforward(ARM_ROTATION_S, ARM_ROTATION_G, ARM_ROTATION_V, ARM_ROTATION_A);
        extensionFF = new ArmFeedforward(ARM_EXTENSION_S, ARM_EXTENSION_G, ARM_EXTENSION_V, ARM_EXTENSION_A);

        resetEncoders();
    }

    /**
     * Drives the arm at the commanded rotation and extension velocities.
     * 
     * @param rotationRate  The rate (-1 to 1) at which to rotate the arm. Positive
     *                      is towards the front of the robot.
     * @param extensionRate The rate (-1 to 1) at which to extend or retract the
     *                      arm. Positive is extension.
     */
    public void driveArm(double rotationRate, double extensionRate) {
        // Make sure the commanded rates aren't going to damage the robot
        double[] rates = checkDriveRates(rotationRate, extensionRate);

        // Scale to max battery voltage
        rotationRate = rates[0] * MAX_VOLTAGE;
        extensionRate = rates[1] * MAX_VOLTAGE;
        
        // Apply voltages to the motors
        rotationSparkMaxLeft.setVoltage(rotationRate);
        rotationSparkMaxRight.setVoltage(rotationRate);
        extensionSparkMax.setVoltage(extensionRate);
    }

    /**
     * Uses a feedforward controller and a PID controller to drive the arm to a
     * commanded angle. Sets the extension rate to zero to keep rotation and
     * extension separate.
     * 
     * @param angle The angle to rotate the arm to. Straight down is 0 degress,
     *              positive is towards the front of the robot.
     * @return The command for driving to the desired angle.
     */
    public Command driveRotationTo(DoubleSupplier angle) {
        return new FunctionalCommand(
            // initialize(): check endcoder delta and zero if necessary, reset PID
            // controller and set setpoint
            () -> {
                if (Math.abs(getAngle() - getAbsoluteAngle()) >= ARM_ABSOLUTE_TOLERANCE) {
                    resetEncoders();
                }
                rotationPID.reset();
                rotationPID.setSetpoint(angle.getAsDouble());
            },
            // execute(): drive arm with rotation velocity calculated by PID controller and
            // zero extension velocity
            () -> {
                // Calculate feedforward contribution
                double rotationRateFF = rotationFF.calculate(angle.getAsDouble(), 0., 0.);
                // Calculate PID contribution
                double rotationRatePID = rotationPID.calculate(getAngle(), angle.getAsDouble());
                // Combine indiviudal contributions
                double rotationRate = rotationRateFF + rotationRatePID;
                // Ensure no extension velocity
                double extensionRate = 0.;
                // Drive arm
                driveArm(rotationRate, extensionRate);
            },
            // end(): set arm voltages to zero
            interupted -> driveArm(0., 0.),
            // isFinished(): check if PID controller is at setpoint
            () -> rotationPID.atSetpoint(),
            // Require the arm subsystem
            this
        );
    }

    /**
     * Uses a feedforward controller and a PID controller to drive the arm to a
     * commanded extension. Sets the rotation rate to zero to keep rotation and
     * extension separate.
     * 
     * @param extension The extension to move the arm to. Fully retracted is 0,
     *                  positive is exteneded.
     * @return The command for driving to the desired extension.
     */
    public Command driveExtensionTo(DoubleSupplier extension) {
        return new FunctionalCommand(
            // initialize(): reset PID controller and set setpoint
            () -> {
                extensionPID.reset();
                extensionPID.setSetpoint(extension.getAsDouble());
            },
            // execute(): drive arm with extension velocity calculated by PID controller and
            // zero rotation velocity
            () -> {
                // Ensure no rotation velocity
                double rotationRate = 0.;
                // Calculate feedforward contribution
                double extensionRateFF = extensionFF.calculate(extension.getAsDouble(), 0., 0.);
                // Calculate PID contribution
                double extensionRatePID = extensionPID.calculate(getExtension(), extension.getAsDouble());
                // Combine indiviudal contributions
                double extensionRate = extensionRateFF + extensionRatePID;
                // Drive arm
                driveArm(rotationRate, extensionRate);
            },
            // end(): set arm voltages to zero
            interupted -> driveArm(0., 0.),
            // isFinished(): check if PID controller is at setpoint
            () -> extensionPID
                    .atSetpoint(),
            // Require the arm subsystem
            this
        );
    }

    /**
     * Drives the arm to a commanded angle and extension via a sequential command
     * where the arm is first fully retracted, then it is rotated to the commanded
     * angle, and finally extended to the commanded extension.
     * 
     * @param angle     The angle to rotate the arm to. Straight down is 0 degress,
     *                  positive is towards the front of the robot.
     * @param extension The extension to move the arm to. Fully retracted is 0,
     *                  positive is exteneded.
     * @return The command for driving to the desired angle and extension
     */
    public Command driveArmTo(DoubleSupplier angle, DoubleSupplier extension) {
        return new SequentialCommandGroup(
            driveExtensionTo(() -> 0.),
            driveRotationTo(angle),
            driveExtensionTo(extension)
        );
    }

    /**
     * Drives the arm to a the center of the robot and fully retracted via a
     * sequential command where the arm is first fully retracted and then it is
     * rotated to zero degrees.
     * 
     * @return The command for driving to center retracted
     */
    public Command centerArm() {
        return new SequentialCommandGroup(
                driveExtensionTo(() -> 0.),
                driveRotationTo(() -> 0.)
        );
    }

    /**
     * Returns the current absolute encoder angle of the arm.
     * 
     * @return The current arm angle in degrees. Straight down is 0 degress,
     *         positive is towards the front of the robot.
     */
    public double getAngle() {
        return (rotationRightEncoder.getPosition() + rotationLeftEncoder.getPosition()) / 2;
    }

    /**
     * Returns the current angle of the arm by averaging the two motor encoders.
     * 
     * @return The current arm angle in degrees. Straight down is 0 degress,
     *         positive is towards the front of the robot.
     */
    public double getAbsoluteAngle() {
        return rotationAbsoluteEncoder.getPosition();
    }

    /**
     * Returns the current arm extension by reading the motor encoder.
     * 
     * @return The current arm extension. Fully retracted is 0, positive is
     *         exteneded.
     */
    public double getExtension() {
        return extensionEncoder.getPosition();
    }

    private double[] checkDriveRates(double rotationRate, double extensionRate) {
        calculateMaxExtension(getAngle());
        double[] rates = { rotationRate, extensionRate };
        return rates;
    }

    /**
     * Calculates the max allowable extension of the arm at a given angle.
     * TODO: Right now this just returns the abosulte max extension regardless of
     * angle. As the robot is finalized, we will need to flesh this out.
     * 
     * @param angle The angle for which the max allowable extension is calculated.
     * @return The max allowable extension at the input angle.
     */
    private double calculateMaxExtension(double angle) {
        return MAX_ARM_EXTENSION_INCHES;
    }

    public void resetEncoders() {
        rotationAbsoluteEncoder.setPosition(rotationAbsoluteEncoder.getAbsolutePosition());
        rotationLeftEncoder.setPosition(rotationAbsoluteEncoder.getAbsolutePosition());
        rotationRightEncoder.setPosition(rotationAbsoluteEncoder.getAbsolutePosition());
    }

    /** This method is run every 20 ms */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Angle", getAngle());
        SmartDashboard.putNumber("Absolute", getAbsoluteAngle());
        SmartDashboard.putNumber("Extension", getExtension());
    }
}
