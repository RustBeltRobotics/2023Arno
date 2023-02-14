package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;

import static frc.robot.Constants.*;


public class Arm extends SubsystemBase {

    private boolean inputLocked = false;

    private CANSparkMax rotationSparkMaxLeft;
    private CANSparkMax rotationSparkMaxRight;
    private CANSparkMax extensionSparkMax;

    private RelativeEncoder rotationLeftEncoder;
    private RelativeEncoder rotationRightEncoder;
    private RelativeEncoder extensionEncoder;

    public Arm() {
        rotationSparkMaxLeft = new CANSparkMax(LEFT_ARM_MOTOR, MotorType.kBrushless);
        rotationSparkMaxLeft.restoreFactoryDefaults();
        rotationSparkMaxLeft.setIdleMode(IdleMode.kBrake);
        rotationSparkMaxLeft.setInverted(true);
        rotationSparkMaxLeft.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        rotationSparkMaxLeft.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);
        rotationSparkMaxLeft.setSoftLimit(SoftLimitDirection.kForward, MAX_ARM_ANGLE_DEGREES);
        rotationSparkMaxLeft.setSoftLimit(SoftLimitDirection.kReverse, MIN_ARM_ANGLE_DEGREES);
        rotationLeftEncoder = rotationSparkMaxLeft.getEncoder();
        rotationLeftEncoder.setPositionConversionFactor(ARM_ROTATION_CONVERSION);

        rotationSparkMaxRight = new CANSparkMax(RIGHT_ARM_MOTOR, MotorType.kBrushless);
        rotationSparkMaxRight.restoreFactoryDefaults();
        rotationSparkMaxRight.setIdleMode(IdleMode.kBrake);
        rotationSparkMaxRight.setInverted(false);
        rotationSparkMaxRight.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        rotationSparkMaxRight.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);
        rotationSparkMaxRight.setSoftLimit(SoftLimitDirection.kForward, MAX_ARM_ANGLE_DEGREES);
        rotationSparkMaxRight.setSoftLimit(SoftLimitDirection.kReverse, MIN_ARM_ANGLE_DEGREES);
        rotationRightEncoder = rotationSparkMaxRight.getEncoder();
        rotationRightEncoder.setPositionConversionFactor(ARM_ROTATION_CONVERSION);
        
        extensionSparkMax = new CANSparkMax(ARM_EXTENSION_MOTOR, MotorType.kBrushless);
        extensionSparkMax.restoreFactoryDefaults();
        extensionSparkMax.setIdleMode(IdleMode.kBrake);
        extensionSparkMax.setInverted(false); // FIXME: Confirm polarity
        extensionSparkMax.setSmartCurrentLimit(NEO550_SMART_CURRENT_LIMIT);
        extensionSparkMax.setSecondaryCurrentLimit(NEO550_SECONDARY_CURRENT_LIMIT);
        extensionSparkMax.setSoftLimit(SoftLimitDirection.kForward, MAX_ARM_EXTENSION_INCHES);
        extensionSparkMax.setSoftLimit(SoftLimitDirection.kReverse, MIN_ARM_EXTENSION_INCHES);
        extensionEncoder = extensionSparkMax.getEncoder();
        extensionEncoder.setPositionConversionFactor(ARM_EXTENSION_CONVERSION);
    }

    public void driveArm(double rotationRate, double extensionRate) {
        // if (inputLocked == true) {
        //     rotationRate = 0;
        //     extensionRate = 0;
        // } else {
            // if (getAngle() >= MAX_ARM_ANGLE_DEGREES) {
            //     rotationRate = Utilities.clamp(rotationRate, Double.MIN_VALUE, 0);
            // } else if (getAngle() <= -MAX_ARM_ANGLE_DEGREES) {
            //     rotationRate = Utilities.clamp(rotationRate, 0, Double.MAX_VALUE);
            // }
            // if (getExtension() >= calculateMaxExtension(getAngle())) {
            //     extensionRate = Utilities.clamp(extensionRate, Double.MIN_VALUE, 0);
            //     if (getAngle() > 0) {
            //         rotationRate = Utilities.clamp(rotationRate, 0, Double.MAX_VALUE);
            //     } else {
            //         rotationRate = Utilities.clamp(rotationRate, Double.MIN_VALUE, 0);
            //     }
            // }
        // }
        rotationSparkMaxLeft.set(rotationRate / MAX_ARM_VELOCITY_DEGREES_PER_SECOND);
        rotationSparkMaxRight.set(rotationRate / MAX_ARM_VELOCITY_DEGREES_PER_SECOND);
        extensionSparkMax.set(extensionRate / MAX_ARM_VELOCITY_INCHES_PER_SECOND);
    }

    public void driveArmTo(double angle, double extension) {
        double rotationRate = (angle - getAngle()) / (2 * MAX_ARM_ANGLE_DEGREES);
        rotationRate = Utilities.deadband(rotationRate, 0.05) * MAX_ARM_VELOCITY_DEGREES_PER_SECOND;
        
        double extensionRate = (extension - getExtension()) / MAX_ARM_EXTENSION_INCHES;
        extensionRate = Utilities.deadband(extensionRate, 0.05) * MAX_ARM_VELOCITY_INCHES_PER_SECOND;
        
        driveArm(rotationRate, extensionRate);
    }

    private double getAngle() {
        double averageAngle = (rotationRightEncoder.getPosition() + rotationLeftEncoder.getPosition()) / 2;
        return averageAngle;
    }

    private double getExtension() {
        double extension = extensionEncoder.getPosition();
        return extension;
    }

    private double calculateMaxExtension(double angle) {
        angle = Math.abs(angle);

        if (angle < ARM_FRAME_PERIMITER_ANGLE_DEGRESS) {
            return 0; // FIXME: We might need to allow some small amount of extension, so that we can pick up game pieces that are inside the robot
        } else if (angle < ARM_GROUND_ANGLE_DEGRESS) {
            return 1; // FIXME: correct trig/geo
        } else {
            return MAX_ARM_EXTENSION_INCHES;
        }
    }

    public void lockOutInput() {
        inputLocked = !inputLocked;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle Right", rotationRightEncoder.getPosition());
        SmartDashboard.putNumber("Arm Angle Left", rotationLeftEncoder.getPosition());
        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putNumber("Arm Extension", getExtension());
    }
}
