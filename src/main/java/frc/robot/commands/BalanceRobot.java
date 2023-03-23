package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

import static frc.robot.Constants.*;

/**
 * Drive up ramp until balanced
 * relies upon pitch and roll angles and velocities
 * Stops moving when robot is coming to a balance
 */
public class BalanceRobot extends CommandBase {
    private double prevPitchAngle = 0;
    private double prevRollAngle = 0;
    private double prevTime = 0;
    private PIDController pidX;
    private PIDController pidY;

    public BalanceRobot() {
        addRequirements(RobotContainer.drivetrain);
        pidX = new PIDController(BALANCE_P, 0., 0.);
        pidY = new PIDController(BALANCE_P, 0., 0.);
    }

    @Override
    public void initialize() {
        pidX.reset();
        pidY.reset();
        prevPitchAngle = RobotContainer.drivetrain.getPitch();
        prevRollAngle = RobotContainer.drivetrain.getRoll();
        prevTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double forwardSpeed = 0.0;
        double strafeSpeed = 0.0;

        double pitchAngle = RobotContainer.drivetrain.getPitch();
        double rollAngle = RobotContainer.drivetrain.getRoll();

        double time = Timer.getFPGATimestamp();
        double deltaTime = time - prevTime;
        double pitchVel = (pitchAngle - prevPitchAngle) / deltaTime;
        double rollVel = (rollAngle - prevRollAngle) / deltaTime;

        prevTime = time;
        prevPitchAngle = pitchAngle;
        prevRollAngle = rollAngle;

        // if we are facing up and the ramp is moving down (or vice versa) we are coming to balance so stop moving
        if (Math.abs(pitchVel) > BALANCE_VELOCITY_TOLERANCE || Math.abs(pitchAngle) < BALANCE_ANGLE_TOLERANCE) {
            forwardSpeed = 0.0;
        } else {
            forwardSpeed = pidX.calculate(pitchAngle, 0.0);
        }

        if (Math.abs(rollVel) > BALANCE_VELOCITY_TOLERANCE || Math.abs(rollAngle) < BALANCE_ANGLE_TOLERANCE) {
            strafeSpeed = 0.0;
        } else {
            strafeSpeed = pidY.calculate(rollAngle, 0.0);
        }

        forwardSpeed = MathUtil.clamp(forwardSpeed, -MAX_BALANCE_VELOCITY, MAX_BALANCE_VELOCITY);
        strafeSpeed = MathUtil.clamp(strafeSpeed, -MAX_BALANCE_VELOCITY, MAX_BALANCE_VELOCITY);

        RobotContainer.drivetrain.drive(new ChassisSpeeds(forwardSpeed * -1., strafeSpeed * -1., 0.));
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivetrain.drive(new ChassisSpeeds(0., 0., 0.));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}