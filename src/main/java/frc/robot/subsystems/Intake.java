package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

import static frc.robot.Constants.*;

public class Intake extends SubsystemBase {
    private final CANSparkMax leftIntakeMotor;
    private final CANSparkMax rightIntakeMotor;

    /** True -> cube, false -> cone */
    public boolean gamepieceIsCube;

    public int selectedRow = 0;
    public int selectedCol = 0;

    private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

    private GenericEntry gamepieceEntry = matchTab.add("Gamepiece", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color When True", "Purple", "Color When False", "Yellow"))
        .withPosition(0, 3).withSize(1, 1)
        .getEntry();

    public Intake() {
        leftIntakeMotor = new CANSparkMax(LEFT_INTAKE_MOTOR, MotorType.kBrushless);
        leftIntakeMotor.restoreFactoryDefaults();
        leftIntakeMotor.setIdleMode(IdleMode.kBrake);
        leftIntakeMotor.setInverted(false);
        leftIntakeMotor.setSmartCurrentLimit(NEO550_SMART_CURRENT_LIMIT);
        leftIntakeMotor.setSecondaryCurrentLimit(NEO550_SECONDARY_CURRENT_LIMIT);

        rightIntakeMotor = new CANSparkMax(RIGHT_INTAKE_MOTOR, MotorType.kBrushless);
        rightIntakeMotor.restoreFactoryDefaults();
        rightIntakeMotor.setIdleMode(IdleMode.kBrake);
        rightIntakeMotor.setInverted(true);
        rightIntakeMotor.setSmartCurrentLimit(NEO550_SMART_CURRENT_LIMIT);
        rightIntakeMotor.setSecondaryCurrentLimit(NEO550_SECONDARY_CURRENT_LIMIT);
    }

    public void selectCube() {
        gamepieceIsCube = true;
    }

    public void selectCone() {
        gamepieceIsCube = false;
    }
    
    public void runIntake(double speed, boolean intake) {
        double leftSpeed = speed;
        double rightSpeed = speed;
        if (intake == true) {
            // Intake 
            if (gamepieceIsCube == true) {
                // Cube
                leftSpeed *= 1.;
                rightSpeed *= 1.;
            } else {
                // Cone
                leftSpeed *= 1.;
                rightSpeed *= -.5;
            }
        } else {
            // Outtake
            if (gamepieceIsCube == true) {
                // Cube
                leftSpeed *= -1.;
                rightSpeed *= -1.;
            } else {
                // Cone
                leftSpeed *= -1.;
                rightSpeed *= 1.;
            }
        }
        leftIntakeMotor.setVoltage(leftSpeed * MAX_VOLTAGE);
        rightIntakeMotor.setVoltage(rightSpeed * MAX_VOLTAGE);
    }

    public void zeroIntake() {
        leftIntakeMotor.setVoltage(0.);
        rightIntakeMotor.setVoltage(0.);
    }

    public Command stopIntake() {
        return new InstantCommand(() -> zeroIntake());
    }


    public Command startIntake(boolean intake) {
        return new InstantCommand(() -> runIntake(1., intake));
    }

    @Override
    public void periodic() {
        gamepieceEntry.setBoolean(gamepieceIsCube);
    }
}
