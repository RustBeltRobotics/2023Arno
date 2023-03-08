package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

/** This command is used to manually control the arm rotation and extension. */
public class DefaultArmCommand extends CommandBase {
    private final Arm arm;

    // DoubleSupplier objects need to be used, not double
    private final DoubleSupplier rotationSupplier;
    private final DoubleSupplier extensionSupplier;

    public DefaultArmCommand(Arm arm, DoubleSupplier rotationSupplier, DoubleSupplier extensionSupplier) {
        this.arm = arm;
        this.rotationSupplier = rotationSupplier;
        this.extensionSupplier = extensionSupplier;

        // Command requires the arm subsystem
        addRequirements(arm);
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * Send the input rotation and extension velocities to the arm's driveArm method.
     */
    @Override
    public void execute() {
        arm.driveArm(rotationSupplier.getAsDouble(), extensionSupplier.getAsDouble());
    }

    /** When the drive method is interupted, set all velocities to zero. */
    @Override
    public void end(boolean interrupted) {
        arm.driveArm(0.0, 0.0);
    }
}
