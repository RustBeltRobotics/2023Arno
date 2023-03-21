package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
    // NavX connected over MXP
    public final AHRS navx;

    /** For user to reset zero for "forward" on the robot while maintaining absolute field zero for odometry */
    private double gyroOffset;

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final Vision vision;

    // The speed of the robot in x and y translational velocities and rotational velocity
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // Boolean statement to control locking the wheels in an X-position
    private boolean wheelsLocked = false;

    // Boolean statement to control autobalance functionality
    private boolean autoBalanceOn = false;

    // Odometry object for the drivetrain
    private SwerveDriveOdometry odometry;

    // PID controllers used for driving to a commanded x/y/r location
    private PIDController xController;
    private PIDController yController;
    private PIDController rController;
    
    private Timer timer;

    double xP = 1.;
    double yP = 1.;
    double rP = .00001;

    public Drivetrain() {
        // Initialize all modules
        frontLeftModule = new SwerveModule(
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET);

        frontRightModule = new SwerveModule(
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET);

        backLeftModule = new SwerveModule(
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET);

        backRightModule = new SwerveModule(
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET);

        // Zero all relative encoders
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();

        // Initialize and zero gyro
        navx = new AHRS(SPI.Port.kMXP);
        zeroGyroscope();

        // Initialize odometry object
        odometry = new SwerveDriveOdometry(
                KINEMATICS, getGyroscopeRotation(),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backLeftModule.getPosition(),
                        backRightModule.getPosition()
                });
        
        // Initialize PID Controllers
        // TODO: Move constants to constants file
        xController = new PIDController(xP, 0., 0.);
        xController.setTolerance(.1);
        yController = new PIDController(yP, 0., 0.);
        yController.setTolerance(.1);
        rController = new PIDController(rP, 0., 0.);
        rController.setTolerance(.1); // FIXME: is this too aggressive?

        SmartDashboard.putNumber("xP", xP);
        SmartDashboard.putNumber("yP", yP);
        SmartDashboard.putNumber("rP", rP);

        vision = new Vision();

        timer = new Timer();
        timer.start();
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        gyroOffset = -getGyroscopeAngle();
        // navx.reset();
    }

    public double getGyroOffset() {
        return gyroOffset;
    }

    /**
     * Toggles whether or not the wheels are locked. If they are, the wheels are
     * crossed into an X pattern and any other drive input has no effect.
     */
    public void toggleWheelsLocked() {
        wheelsLocked = !wheelsLocked;
    }

    /**
     * Toggles whether or not the robot is in autobalance mode. If it is, the robot
     * tries to zero out any roll or pitch messurements, and any driver input has no
     * effect.
     */
    public void toggleAutoBalance() {
        autoBalanceOn = !autoBalanceOn;
    }

    public double getGyroscopeAngle() {
        return Math.IEEEremainder(360. - navx.getAngle(), 360.);
    }

    // Returns the measurment of the gyroscope yaw. Used for field-relative drive
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(getGyroscopeAngle());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                getGyroscopeRotation(),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backLeftModule.getPosition(),
                        backRightModule.getPosition()
                },
                pose);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.setState(states[0]);
        frontRightModule.setState(states[1]);
        backLeftModule.setState(states[2]);
        backRightModule.setState(states[3]);
    }

    /**
     * Used to drive the robot with the provided ChassisSpeed object. However, if
     * the robot is in autobalance mode, the ChassisSpeed object is ignored, and a
     * new one is calculated based off the pitch and roll of the robot.
     * 
     * @param chassisSpeeds The translational and rotational velocities at which to
     *                      drive the robot.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        if (!autoBalanceOn) {
            // If we're not in autobalance mode, act normally.
            this.chassisSpeeds = chassisSpeeds;
            // this.chassisSpeeds = new ChassisSpeeds(-chassisSpeeds.vxMetersPerSecond, -chassisSpeeds.vyMetersPerSecond, -chassisSpeeds.omegaRadiansPerSecond);
        } else {
            // If we are in autobalance mode, calculate a new ChassisSpeed object based off
            // the measured pitch and roll
            this.chassisSpeeds = new ChassisSpeeds(
                    // X velocity is proportional to the sin of the roll angle
                    MathUtil.applyDeadband(Math.sin(Math.toRadians(navx.getRoll())), Math.toRadians(0.5))
                            * MAX_VELOCITY_METERS_PER_SECOND * AUTOBALANCE_SPEED_FACTOR,
                    // Y velocity is proportional to the sin of the pitch angle
                    MathUtil.applyDeadband(Math.sin(Math.toRadians(navx.getPitch())), Math.toRadians(0.5))
                            * MAX_VELOCITY_METERS_PER_SECOND * AUTOBALANCE_SPEED_FACTOR,
                    // No rotational velocity
                    0.0);
        }
    }
    
    /**
     * Uses a feedforward controller and a PID controller to drive the arm to a
     * commanded extension. Sets the rotation rate to zero to keep rotation and
     * extension separate.
     * 
     * @param extension The extension to move the arm to. Fully retracted is 0,
     *                  positive is exteneded.
     * @return The command for driving to the desired extension.
     */
    public Command driveToTarget(DoubleSupplier xTarget, DoubleSupplier yTarget, DoubleSupplier rTarget) {
        return new FunctionalCommand(
            // initialize(): reset PID controller and set setpoint
            () -> {
                xController.reset();
                yController.reset();
                rController.reset();
                xController.setSetpoint(xTarget.getAsDouble());
                yController.setSetpoint(yTarget.getAsDouble());
                rController.setSetpoint(rTarget.getAsDouble());
            },
            // execute(): drive robot with velocities calculated by PID controllers
            () -> {
                // Calculate velocities
                double xVel = xController.calculate(getPose().getX(), xTarget.getAsDouble());
                double yVel = yController.calculate(getPose().getY(), yTarget.getAsDouble());
                double rVel = rController.calculate(getGyroscopeAngle(), rTarget.getAsDouble());

                drive(new ChassisSpeeds(xVel, yVel, rVel));
            },
            // end(): set drive voltages to zero
            interupted -> drive(new ChassisSpeeds(0., 0., 0.)),
            // isFinished(): check if PID controllers are at setpoint
            () -> (xController.atSetpoint() && yController.atSetpoint()),// FIXME: && rController.atSetpoint()),
            // Require the drivetrain subsystem
            this
        );
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * This method is used to command the individual module states based off the
     * ChassisSpeeds object
     */
    @Override
    public void periodic() {
        // Convert from ChassisSpeeds to SwerveModuleStates
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        // Make sure no modules are being commanded to velocites greater than the max possible velocity
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        if (!wheelsLocked) {
            // If we are not in wheel's locked mode, set the states normally
            frontLeftModule.setState(states[0]);
            frontRightModule.setState(states[1]);
            backLeftModule.setState(states[2]);
            backRightModule.setState(states[3]);
        } else {
            // If we are in wheel's locked mode, set the drive velocity to 0 so there is no
            // movment, and command the steer angle to either plus or minus 45 degrees to
            // form an X pattern.
            frontLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            frontRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            backLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            backRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        }

        // Update the odometry
        odometry.update(
                getGyroscopeRotation(),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(), frontRightModule.getPosition(),
                        backLeftModule.getPosition(), backRightModule.getPosition()
                });
        SmartDashboard.putNumber("X", getPose().getX());
        SmartDashboard.putNumber("Y", getPose().getY());
        SmartDashboard.putNumber("R", getGyroscopeAngle());

        xP = SmartDashboard.getNumber("xP", .1);
        yP = SmartDashboard.getNumber("yP", .1);
        rP = SmartDashboard.getNumber("rP", .0001);

        xController.setP(xP);
        yController.setP(yP);
        rController.setP(rP);

        if (timer.get() >= 5.) {
            double[] location = vision.getFieldLocation();
            if (location != null) {
                resetOdometry(new Pose2d(new Translation2d(location[0], location[1]), getGyroscopeRotation()));
                timer.restart();
            }
        }
    }
}
