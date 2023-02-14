package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

public class AutoBalanceRoutine extends CommandBase {
    private Drivetrain drivetrain;
    private double xVelocity = .5;
    private double yVelocity = 0;
    private double radiansPerSec = 0;
    private int phase;
    private float rollOffset;
    private float pitchOffset;

    public AutoBalanceRoutine(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        phase = 0;
        pitchOffset = drivetrain.navx.getPitch();
        rollOffset = drivetrain.navx.getRoll();
    }
    
    @Override
    public void execute() {
        ChassisSpeeds robotSpeed = new ChassisSpeeds(xVelocity, yVelocity, radiansPerSec);
        float roll = drivetrain.navx.getRoll() - rollOffset;
        float pitch = drivetrain.navx.getPitch() - pitchOffset;
        if (phase == 0) {
            // Drive forward in until the the robot starts driving up the ramp
            // Then turn on auto balance and move to phase 1
            if (Math.abs(roll) < CHARGE_STATION_UNBALANCED_ANGLE) {
            } else {
                phase = 1;
                drivetrain.toggleAutoBalance();
            }
        } else if (phase == 1) {
            // Once the platform is balanced, turn off autobalance, lock the wheels, and move to phase 2
            if (Math.abs(roll) < CHARGE_STATION_BALANCED_ANGLE) {
                phase = 2;
                drivetrain.toggleAutoBalance();
                drivetrain.toggleWheelsLocked();
            }
        } else if (phase == 2) {
            // If the platform becomes unblanaced, unlock the wheels, turn autobalance on, and move back to phase 1
            if (Math.abs(roll) > CHARGE_STATION_BALANCED_ANGLE) {
                phase = 1;
                drivetrain.toggleWheelsLocked();
                drivetrain.toggleAutoBalance();
            }
        }
        drivetrain.drive(robotSpeed);
        System.out.println("Pitch: " + drivetrain.navx.getPitch());
        System.out.println("Roll: " + drivetrain.navx.getRoll());
        System.out.println("Phase: " + phase);
        SmartDashboard.putNumber("Phase", phase);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
        if( phase == 2 )
        {
            drivetrain.toggleWheelsLocked();
            phase = 0;
        }
        else if( phase == 1 )
        {
            drivetrain.toggleAutoBalance();
            phase = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
