package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities;

public class Arm extends SubsystemBase {

    private boolean inputLocked = false;

    private CANSparkMax rotationSparkMaxLeft;
    private CANSparkMax rotationSparkMaxRight;
    private CANSparkMax extensionSparkMax;

    private RelativeEncoder rotationLeftEncoder;
    private RelativeEncoder rotationRightEncoder;
    private RelativeEncoder extensionEncoder;

    public Arm() {
        rotationSparkMaxLeft = new CANSparkMax(Constants.LEFT_ARM_MOTOR, MotorType.kBrushless);
        rotationSparkMaxLeft.restoreFactoryDefaults();
        rotationSparkMaxLeft.setIdleMode(IdleMode.kBrake);
        rotationSparkMaxLeft.setInverted(false); //FIXME: Confirm polarity
        rotationSparkMaxLeft.setSmartCurrentLimit(40);
        rotationLeftEncoder = rotationSparkMaxLeft.getEncoder();
        rotationLeftEncoder.setPositionConversionFactor(1); // FIXME: set a conversion factor

        rotationSparkMaxRight = new CANSparkMax(Constants.RIGHT_ARM_MOTOR, MotorType.kBrushless);
        rotationSparkMaxRight.restoreFactoryDefaults();
        rotationSparkMaxRight.setIdleMode(IdleMode.kBrake);
        rotationSparkMaxRight.setInverted(true); //FIXME: Confirm polarity
        rotationSparkMaxLeft.setSmartCurrentLimit(40);
        rotationRightEncoder = rotationSparkMaxRight.getEncoder();
        rotationRightEncoder.setPositionConversionFactor(1); // FIXME: set a conversion factor
        
        extensionSparkMax = new CANSparkMax(Constants.ARM_EXTENSION_MOTOR, MotorType.kBrushless);
        extensionSparkMax.restoreFactoryDefaults();
        extensionSparkMax.setIdleMode(IdleMode.kBrake);
        extensionSparkMax.setInverted(false); //FIXME: Confirm polarity
        rotationSparkMaxLeft.setSmartCurrentLimit(20);
        extensionEncoder = extensionSparkMax.getEncoder();
        extensionEncoder.setPositionConversionFactor(1); // FIXME: set a conversion factor
    }

    public void driveArm(double rotationRate, double extensionRate) {
        if (inputLocked == true) {
            rotationRate = 0;
            extensionRate = 0;
        } else {
            if (getAngle() >= Constants.MAX_ARM_ANGLE_DEGREES) {
                rotationRate = Utilities.clamp(rotationRate, Double.MIN_VALUE, 0);
            } else if (getAngle() <= -Constants.MAX_ARM_ANGLE_DEGREES) {
                rotationRate = Utilities.clamp(rotationRate, 0, Double.MAX_VALUE);
            }
            if (getExtension() >= calculateMaxExtension(getAngle())) {
                extensionRate = Utilities.clamp(extensionRate, Double.MIN_VALUE, 0);
                if (getAngle() > 0) {
                    rotationRate = Utilities.clamp(rotationRate, 0, Double.MAX_VALUE);
                } else {
                    rotationRate = Utilities.clamp(rotationRate, Double.MIN_VALUE, 0);
                }
            }
        }
    }

    public void driveArmTo(double angle, double extension) {
        double rotationRate = (angle - getAngle()) / (2 * Constants.MAX_ARM_ANGLE_DEGREES);
        rotationRate = Utilities.deadband(rotationRate, 0.05) * Constants.MAX_ARM_VELOCITY_RADIANS_PER_SECOND;
        
        double extensionRate = (extension - getExtension()) / Constants.MAX_ARM_EXTENSION_METERS;
        extensionRate = Utilities.deadband(extensionRate, 0.05) * Constants.MAX_ARM_VELOCITY_METERS_PER_SECOND;
        
        driveArm(rotationRate, extensionRate);
    }

    private double getAngle() { //FIXME: consider making private
        double averageAngle = (rotationRightEncoder.getPosition() + rotationLeftEncoder.getPosition()) / 2;
        // System.out.println(averageAngle);
        return averageAngle;
    }

    private double getExtension() { //FIXME: consider making private
        double extension = extensionEncoder.getPosition();
        // System.out.println(extension);
        return extension;
    }

    private double calculateMaxExtension(double angle) {
        angle = Math.abs(angle);

        if (angle < Constants.ARM_FRAME_PERIMITER_ANGLE_DEGRESS) {
            return 0; // FIXME: We might need to allow some small amount of extension, so that we can
                      // pick up game pieces that are inside the robot
        } else if (angle < Constants.ARM_GROUND_ANGLE_DEGRESS) {
            return 1; // FIXME: correct trig/geo
        } else {
            return Constants.MAX_ARM_EXTENSION_METERS;
        }
    }

    public void lockOutInput() {
        inputLocked = !inputLocked;
    }

    @Override
    public void periodic() {
        System.out.println("Angle: " + getAngle());
        System.out.println("Extension: " + getExtension());
    }
}
