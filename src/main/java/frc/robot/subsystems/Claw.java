package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Claw extends SubsystemBase {

    private CANSparkMax leftClaw;
    private CANSparkMax rightClaw;

    private RelativeEncoder leftClawEncoder;
    private RelativeEncoder rightClawEncoder;

    public Claw() {
        leftClaw = new CANSparkMax(LEFT_CLAW_MOTOR, MotorType.kBrushless);
        leftClaw.restoreFactoryDefaults();
        leftClaw.setIdleMode(IdleMode.kBrake); // FIXME: if the surgical tubing is unable to overcome brake mode, we should change this to kCoast
        leftClaw.setInverted(false); // FIXME: Confirm polarity
        leftClaw.setSmartCurrentLimit(NEO550_SMART_CURRENT_LIMIT);
        leftClaw.setSecondaryCurrentLimit(NEO550_SECONDARY_CURRENT_LIMIT);
        leftClaw.setSoftLimit(SoftLimitDirection.kForward, MAX_CLAW_ANGLE_DEGREES);
        leftClaw.setSoftLimit(SoftLimitDirection.kReverse, MIN_CLAW_ANGLE_DEGREES);
        leftClawEncoder = leftClaw.getEncoder();
        leftClawEncoder.setPositionConversionFactor(CLAW_ROTATION_CONVERSION);

        rightClaw = new CANSparkMax(RIGHT_CLAW_MOTOR, MotorType.kBrushless);
        rightClaw.restoreFactoryDefaults();
        rightClaw.setIdleMode(IdleMode.kBrake); // FIXME: if the surgical tubing is unable to overcome brake mode, we should change this to kCoast
        rightClaw.setInverted(true); // FIXME: Confirm polarity
        rightClaw.setSmartCurrentLimit(NEO550_SMART_CURRENT_LIMIT);
        rightClaw.setSecondaryCurrentLimit(NEO550_SECONDARY_CURRENT_LIMIT);
        rightClaw.setSoftLimit(SoftLimitDirection.kForward, MAX_CLAW_ANGLE_DEGREES);
        rightClaw.setSoftLimit(SoftLimitDirection.kReverse, MIN_CLAW_ANGLE_DEGREES);
        rightClawEncoder = rightClaw.getEncoder();
        rightClawEncoder.setPositionConversionFactor(CLAW_ROTATION_CONVERSION);
    }

    public Command openClaw() {
        return startEnd(
            () -> { // Open Claw
                leftClaw.set(MAX_CLAW_VELOCITY_DEGREES_PER_SECOND);
                rightClaw.set(MAX_CLAW_VELOCITY_DEGREES_PER_SECOND);
            },
            () -> { // Close Claw
                leftClaw.set(0);
                rightClaw.set(0);
            }
        );
    }

    private double[] getAngles() {
        double leftAngle = leftClawEncoder.getPosition();
        double rightAngle = rightClawEncoder.getPosition();
        return new double[] {leftAngle, rightAngle};
    }

    public void printClaw() {
        double[] angles = getAngles();
        System.out.println("Left Claw Angle: " + angles[0]);
        System.out.println("Right Claw Angle: " + angles[1]);
    }
}
