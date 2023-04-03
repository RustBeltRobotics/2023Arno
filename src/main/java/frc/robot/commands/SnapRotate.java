package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import static frc.robot.Constants.*;

/**
 * This command takes a file name as an arguement. It will then attempt to run
 * said path by generating a trajectory from the path parameters in said file.
 * PID values are from the Constants file.
 */
public class SnapRotate extends CommandBase {
    private PIDController rController;

    private double rCurrent;
    private double rGoal;

    public SnapRotate(DoubleSupplier rotationSupplier, IntSupplier directionSupplier) {
        addRequirements(RobotContainer.drivetrain);
        rCurrent = rotationSupplier.getAsDouble();
        if (rCurrent == 0) rGoal = 0.;
        else if (rCurrent == 90) rGoal = -90;
        else if (rCurrent == 270) rGoal = 90;
        else if (rCurrent == 180) {
            if (rCurrent > 0) rGoal = 180;
            else rGoal = -180.;
        }
        // rGoal = directionSupplier.getAsInt() == -1 ? getNegative() : getPositive();
    }

    // public double getPositive() {
    //     if (rCurrent >= 90.) return 180.;
    //     else if (rCurrent >= 0.) return 90.;
    //     else if (rCurrent >= -90.) return 0.;
    //     else return -90;
    // }

    // public double getNegative() {
    //     if (rCurrent <= -90.) return -180.;
    //     else if (rCurrent <= 0.) return -90.;
    //     else if (rCurrent <= 90.) return 0.;
    //     else return 90;
    // }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rController = new PIDController(POSE_ROTATION_P, 0., 0.);
        rController.enableContinuousInput(-180., 180.);
        rController.setSetpoint(rGoal);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rVel = rController.calculate(RobotContainer.drivetrain.getPose().getRotation().getDegrees());

        rVel = MathUtil.clamp(rVel, -MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        ChassisSpeeds robotSpeed = new ChassisSpeeds(0., 0., rVel);

        // pass the robotSpeed to the swerveDrive
        RobotContainer.drivetrain.drive(robotSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // stop the robot as we are done with the path.
        RobotContainer.drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return rController.atSetpoint();
    }
}