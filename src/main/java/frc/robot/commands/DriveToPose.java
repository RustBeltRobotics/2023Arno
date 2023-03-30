package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

/**
 * This command takes a file name as an arguement. It will then attempt to run
 * said path by generating a trajectory from the path parameters in said file.
 * PID values are from the Constants file.
 */
public class DriveToPose extends CommandBase {

    private PIDController xController;
    private PIDController yController;
    private PIDController rController;

    private double xGoal;
    private double yGoal;
    private double rGoal;

    public DriveToPose(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
        addRequirements(RobotContainer.drivetrain);
        xGoal = xSupplier.getAsDouble();
        yGoal = ySupplier.getAsDouble();
        rGoal = rotationSupplier.getAsDouble();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        xController = new PIDController(POSE_TRANSLATION_P, 0., 0.);
        yController = new PIDController(POSE_TRANSLATION_P, 0., 0.);
        rController = new PIDController(POSE_ROTATION_P, 0., 0.);
        rController.enableContinuousInput(-180., 180.);

        xController.setSetpoint(xGoal);
        yController.setSetpoint(yGoal);
        rController.setSetpoint(rGoal);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xVel = xController.calculate(RobotContainer.drivetrain.getPose().getX());
        double yVel = yController.calculate(RobotContainer.drivetrain.getPose().getY());
        double rVel = rController.calculate(RobotContainer.drivetrain.getPose().getRotation().getDegrees());

        xVel = MathUtil.clamp(xVel, -MAX_VELOCITY_METERS_PER_SECOND, MAX_VELOCITY_METERS_PER_SECOND);
        yVel = MathUtil.clamp(yVel, -MAX_VELOCITY_METERS_PER_SECOND, MAX_VELOCITY_METERS_PER_SECOND);        
        rVel = MathUtil.clamp(rVel, -MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        ChassisSpeeds robotSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xVel, yVel, rVel), RobotContainer.drivetrain.getPose().getRotation());

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
        return (xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint());
    }
}