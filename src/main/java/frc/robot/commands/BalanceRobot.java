package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
    // private double prevRollAngle = 0;
    private double prevTime = 0;

    public BalanceRobot() {
        addRequirements(RobotContainer.drivetrain);
    }

    @Override
    public void initialize() {
        prevPitchAngle = RobotContainer.drivetrain.getPitch();
        // prevRollAngle = RobotContainer.drivetrain.getRoll();
        prevTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double forwardSpeed = 0.0;
        // double strafeSpeed = 0.0;

        double pitchAngle = RobotContainer.drivetrain.getPitch();
        // double rollAngle = RobotContainer.drivetrain.getRoll();

        double time = Timer.getFPGATimestamp();
        double deltaTime = time - prevTime;
        double pitchVel = (pitchAngle - prevPitchAngle) / deltaTime;
        // double rollVel = (rollAngle - prevRollAngle) / deltaTime;

        prevTime = time;
        prevPitchAngle = pitchAngle;
        // prevRollAngle = rollAngle;

        // if we are facing up and the ramp is moving down (or vice versa) we are coming to balance so stop moving
        if (Math.abs(pitchVel) > BALANCE_VELOCITY_TOLERANCE || Math.abs(pitchAngle) < BALANCE_ANGLE_TOLERANCE) {
            forwardSpeed = 0.0;
        } else {
            if (pitchAngle < 0) {
                forwardSpeed = MAX_BALANCE_VELOCITY;
            } else {

                forwardSpeed = -MAX_BALANCE_VELOCITY;
            }
        }

        // if (Math.abs(rollVel) > BALANCE_VELOCITY_TOLERANCE || Math.abs(rollAngle) < BALANCE_ANGLE_TOLERANCE) {
        //     strafeSpeed = 0.0;
        // } else {
        //     if (rollAngle < 0) {
        //         strafeSpeed = MAX_BALANCE_VELOCITY;
        //     } else {

        //         strafeSpeed = -MAX_BALANCE_VELOCITY;
        //     }
        // }

        RobotContainer.drivetrain.drive(new ChassisSpeeds(forwardSpeed, 0., 0.)); //strafeSpeed, 0.));
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