package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

/**
 * This auto routine drives onto the bridge and balances there before locking
 * the wheels into an X.
 */
public class AutoBalanceRoutine extends CommandBase {
    private final Drivetrain drivetrain;
    // Velocities used when approaching the ramp.
    // TODO: Consider moving these to the Constants class.
    // TODO: Consider doing this with PID or something more elegant.
    private final double xVelocity = .5;
    private final double yVelocity = 0;
    private final double radiansPerSec = 0;

    /**
     * Integer used to store the what "phase" of the routine we are in.
     * Phase 0: Approaching the ramp
     * Phase 1: Driving up the ramp
     * Phase 2: Balanced on the ramp
     */
    private int phase;

    // Variable used to store the gyro reading at the start of the command, in case the field isn't perfectly level
    // TODO: Calibrate the navX.
    private float rollOffset;

    public AutoBalanceRoutine(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // Command requires the drivetrain subsystem
        addRequirements(drivetrain);
    }

    /**
     * This method is run once at the start of the command.
     * <p>
     * The phase is set to 0, and the roll offset is measured.
     */
    @Override
    public void initialize() {
        phase = 0;
        rollOffset = drivetrain.navx.getRoll();
    }

    /**
     * This method is run ever 20 ms.
     * <p>
     * Calculates the current roll of the robot.
     * Checks what phase we are in, and acts appropriately.
     */
    @Override
    public void execute() {
        // Robot speed used while approaching the ramp. Note, if we are in autobalnce
        // mode or wheels locked mode, i.e. phases 1 and 2 respectively, this input is
        // overridden by the drivetrain's drive method.
        ChassisSpeeds robotSpeed = new ChassisSpeeds(xVelocity, yVelocity, radiansPerSec);
        float roll = drivetrain.navx.getRoll() - rollOffset;
        if (phase == 0) {
            // Drive forward until the the robot starts driving up the ramp
            // Then turn on auto balance and move to phase 1
            if (Math.abs(roll) < CHARGE_STATION_UNBALANCED_ANGLE) {
            } else {
                phase = 1;
                drivetrain.toggleAutoBalance();
            }
        } else if (phase == 1) {
            // Once the platform is balanced, turn off autobalance, lock the wheels, and
            // move to phase 2
            if (Math.abs(roll) < CHARGE_STATION_BALANCED_ANGLE) {
                phase = 2;
                drivetrain.toggleAutoBalance();
                drivetrain.toggleWheelsLocked();
            }
        } else if (phase == 2) {
            // If the platform becomes unblanaced, unlock the wheels, turn autobalance on,
            // and move back to phase 1
            if (Math.abs(roll) > CHARGE_STATION_BALANCED_ANGLE) {
                phase = 1;
                drivetrain.toggleWheelsLocked();
                drivetrain.toggleAutoBalance();
            }
        }
        drivetrain.drive(robotSpeed);
    }

    /**
     * This method is run if the command is interuppted.
     * <p>
     * Ensures that autobalance mode and wheels locked mode are both toggled off, so
     * the driver can take control with no futher input. Also resets the phase to 0.
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
        if (phase == 2) {
            drivetrain.toggleWheelsLocked();
            phase = 0;
        } else if (phase == 1) {
            drivetrain.toggleAutoBalance();
            phase = 0;
        }
    }
}
