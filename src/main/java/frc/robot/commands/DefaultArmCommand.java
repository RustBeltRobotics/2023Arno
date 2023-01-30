package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

public class DefaultArmCommand extends CommandBase {
    private final Arm armSubsystem;

    private final DoubleSupplier rotationSupplier;
    private final DoubleSupplier extensionSupplier;

    public DefaultArmCommand(Arm armSubsystem, DoubleSupplier rotationSupplier, DoubleSupplier extensionSupplier) {
        this.armSubsystem = armSubsystem;
        this.rotationSupplier = rotationSupplier;
        this.extensionSupplier = extensionSupplier;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.driveArm(rotationSupplier.getAsDouble(), extensionSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.driveArm(0.0, 0.0);
    }
}
