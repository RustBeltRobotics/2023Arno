package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Claw extends SubsystemBase {

    private CANSparkMax leftClaw;
    private CANSparkMax rightClaw;

    // private RelativeEncoder leftClawEncoder;
    // private RelativeEncoder rightClawEncoder;

    public Claw() {
        leftClaw = new CANSparkMax(LEFT_CLAW_MOTOR, MotorType.kBrushless);
        leftClaw.restoreFactoryDefaults();
        leftClaw.setIdleMode(IdleMode.kBrake); // FIXME: if the surgical tubing is unable to overcome brake mode, we should change this to kCoast
        leftClaw.setInverted(false);
        leftClaw.setSmartCurrentLimit(NEO550_SMART_CURRENT_LIMIT);
        leftClaw.setSecondaryCurrentLimit(NEO550_SECONDARY_CURRENT_LIMIT);
        // leftClaw.setSoftLimit(SoftLimitDirection.kForward, MAX_CLAW_ANGLE_DEGREES);
        // leftClawEncoder = leftClaw.getEncoder();
        // leftClawEncoder.setPositionConversionFactor(CLAW_ROTATION_CONVERSION);

        rightClaw = new CANSparkMax(RIGHT_CLAW_MOTOR, MotorType.kBrushless);
        rightClaw.restoreFactoryDefaults();
        rightClaw.setIdleMode(IdleMode.kBrake); // FIXME: if the surgical tubing is unable to overcome brake mode, we should change this to kCoast
        rightClaw.setInverted(true);
        rightClaw.setSmartCurrentLimit(NEO550_SMART_CURRENT_LIMIT);
        rightClaw.setSecondaryCurrentLimit(NEO550_SECONDARY_CURRENT_LIMIT);
        // rightClaw.setSoftLimit(SoftLimitDirection.kForward, MAX_CLAW_ANGLE_DEGREES);
        // rightClawEncoder = rightClaw.getEncoder();
        // rightClawEncoder.setPositionConversionFactor(CLAW_ROTATION_CONVERSION);
    }

    public Command runClawForward() {
        return startEnd(
            () -> { // Run motors forward
                leftClaw.set(1);
                rightClaw.set(1);
            },
            () -> { // Release motors
                leftClaw.set(0);
                rightClaw.set(0);
            }
        );
    }

    public Command runClawBackward() {
        return startEnd(
            () -> { // Run motors backward
                leftClaw.set(-1);
                rightClaw.set(-1);
            },
            () -> { // Release motors
                leftClaw.set(0);
                rightClaw.set(0);
            }
        );
    }

    @Override
    public void periodic() {}
}
