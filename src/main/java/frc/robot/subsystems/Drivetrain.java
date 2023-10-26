package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
    // NavX connected over MXP
    public final AHRS navx;

    /**
     * For user to reset zero for "forward" on the robot while maintaining absolute
     * field zero for odometry
     */
    private double gyroOffset;

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    // The speed of the robot in x and y translational velocities and rotational
    // velocity
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // Boolean statement to control locking the wheels in an X-position
    private boolean wheelsLocked = false;

    // Boolean statement to control autobalance functionality
    private boolean autoBalanceOn = false;

    // PID controllers used for driving to a commanded x/y/r location
    private PIDController xController;
    private PIDController yController;
    private PIDController rController;
    private PPHolonomicDriveController pathController;

    private final SwerveDrivePoseEstimator poseEstimator;

    private PIDController pidX;
    private PIDController pidY;

    public Drivetrain() {
        // Initialize all modules - Switched all the modules around at Ruckus because
        // the arm was flipped around so everything was backwards
        frontLeftModule = new SwerveModule(
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET);

        frontRightModule = new SwerveModule(
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET);

        backLeftModule = new SwerveModule(
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET);

        backRightModule = new SwerveModule(
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET);

        // Zero all relative encoders
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();

        // Initialize and zero gyro
        navx = new AHRS(SPI.Port.kMXP);
        zeroGyroscope();

        // Initialize PID Controllers
        xController = new PIDController(TRAJECTORY_TRANSLATION_P, 0., 0.);
        yController = new PIDController(TRAJECTORY_TRANSLATION_P, 0., 0.);
        rController = new PIDController(TRAJECTORY_ROTATION_P, 0., 0.);
        rController.enableContinuousInput(-180., 180.);

        pathController = new PPHolonomicDriveController(xController, yController, rController);

        poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getGyroscopeRotation(), getSwerveModulePositions(),
                new Pose2d());

        pidX = new PIDController(.25, 0., 0.);
        pidY = new PIDController(.25, 0., 0.);

        pidX.reset();
        pidY.reset();
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope(double angle) {
        gyroOffset = -getGyroscopeAngle() + angle;
    }

    public void zeroGyroscope() {
        zeroGyroscope(0.0);
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

    /** @return Pitch in degrees, -180 to 180 */
    public double getPitch() {
        return navx.getPitch();
    }

    /** @return Roll in degrees, -180 to 180 */
    public double getRoll() {
        return navx.getRoll();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), pose);
    }

    public void updateOdometry() {
        poseEstimator.update(getGyroscopeRotation(), getSwerveModulePositions());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.setState(states[0]);
        frontRightModule.setState(states[1]);
        backLeftModule.setState(states[2]);
        backRightModule.setState(states[3]);
    }

    public void resetTrajectoryPIDControllers() {
        xController.reset();
        yController.reset();
        rController.reset();
    }

    /**
     * Using the desiredState and the currentState, use the pathController to find
     * the speeds the robot should be going
     * 
     * @param desiredState {@link PathPlannerState} robot needs to be at
     * @return {@link ChassisSpeeds} motors should move at to reach desired state
     */
    public ChassisSpeeds calculateSpeedsTraj(PathPlannerState desiredState) {
        return pathController.calculate(getPose(), desiredState);
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
        this.chassisSpeeds = chassisSpeeds;
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
        // Make sure no modules are being commanded to velocites greater than the max
        // possible velocity
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
            frontLeftModule.lockModule(45);
            frontRightModule.lockModule(-45);
            backLeftModule.lockModule(-45);
            backRightModule.lockModule(45);
        }

        // Update the odometry
        updateOdometry();
    }
}
