package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** Maximum battery voltage */
    public static final double MAX_VOLTAGE = 12.;

    // Spark Max current limits
    /** Smart drive motor current */
    public static final int DRIVE_SMART_CURRENT_LIMIT = 15; // TODO: Optimize after driver training/feedback
    /** Smart steer motor current */
    public static final int STEER_SMART_CURRENT_LIMIT = 20; // TODO: Optimize after driver training/
    /** Secondary drive motor current */
    public static final int DRIVE_SECONDARY_CURRENT_LIMIT = 80; // TODO: Optimize after driver training/feedback
    /** Secondary steer motor current */
    public static final int STEER_SECONDARY_CURRENT_LIMIT = 80; // TODO: Optimize after driver training/feedback
    /** Smart current limit applied to NEOs */
    public static final int NEO_SMART_CURRENT_LIMIT = 40;
    /** Secondary current limit applied to NEOs */
    public static final int NEO_SECONDARY_CURRENT_LIMIT = 60;
    /** Smart current limit applied to NEO 550s */
    public static final int NEO550_SMART_CURRENT_LIMIT = 20;
    /** Secondary current limit applied to NEO 550s */
    public static final int NEO550_SECONDARY_CURRENT_LIMIT = 30;

    /**
     * If the robot passes from greater than this to less than this, the robot is
     * considered balanced on top of the charge station
     */
    public static final double CHARGE_STATION_BALANCED_ANGLE = 0.5;

    /**
     * If the robot passes from less than this to greater than this, the robot is
     * considered to have started driving up the ramp
     */
    public static final double CHARGE_STATION_UNBALANCED_ANGLE = 5.;

    // Drivetrain Constants
    /**
     * The left-to-right distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.47; // TODO: confirm value for new robot

    /**
     * The front-to-back distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.47; // TODO: confirm value for new robot

    /**
     * Creates a swerve kinematics object, to convert desired chassis velocity into
     * individual module states
     */
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    /** Conversion between rotations and meters */
    public static final double DRIVE_POSITION_CONVERSION = Math.PI
            * SdsModuleConfigurations.MK4I_L2.getWheelDiameter()
            * SdsModuleConfigurations.MK4I_L2.getDriveReduction();

    /** Conversion between rotations per minute and meters per seconds */
    public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60.;

    /**
     * The maximum linear velocity of the robot in meters per second. This is a
     * measure of how fast the robot can move linearly.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676. * DRIVE_VELOCITY_CONVERSION;

    /**
     * The maximum angular velocity of the robot in radians per second. This is a
     * measure of how fast the robot can rotate in place.
     */
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    /** Conversion between rotations and degrees */
    public static final double STEER_POSITION_CONVERSION = 360. * SdsModuleConfigurations.MK4I_L2.getSteerReduction();

    /** Conversion between rotations per minute and degrees per seconds */
    public static final double STEER_VELOCITY_CONVERSION = STEER_POSITION_CONVERSION / 60.;

    /** This factor is applied to the maximum drive velocity during autobalancing */
    public static final double AUTOBALANCE_SPEED_FACTOR = 0.25;

    /** Time (seconds) to get to max x velocity under manual control */
    public static final double TIME_TO_MAX_X = 1.;
    /** Time (seconds) to get to max y velocity under manual control */
    public static final double TIME_TO_MAX_Y = 1.;
    /** Time (seconds) to get to max rotational velocity under manual control */
    public static final double TIME_TO_MAX_R = 1.;

    // Steer PID Constants
    public static final double STEER_P = 0.008;
    public static final double STEER_I = 0.;
    public static final double STEER_D = 0.0002;

    /** Factor applied to maximum drive velocity in manual mode, high speed mode */
    public static final double MAX_SPEED_FACTOR_HIGH = 0.7;
    /** Factor applied to maximum drive velocity in manual mode, low speed mode */
    public static final double MAX_SPEED_FACTOR_LOW = 0.2;

    // CAN IDs
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 14;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 13;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 33;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 17;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 18;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 31;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 12;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 34;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 15;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 16;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 32;

    // Module Offsets - rotational offsets such that the modules all read 0 degrees
    // when facing forward
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.62);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(246.09);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(288.37);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(345.23);

    // Arm Constants
    // CAN IDs
    public static final int LEFT_ARM_MOTOR = 19;
    public static final int RIGHT_ARM_MOTOR = 20;
    public static final int ARM_EXTENSION_MOTOR = 21;

    /** The max allowable angle of the arm */
    public static final double MAX_ARM_ANGLE_DEGREES = 120.; // TODO: Confirm this value
    /** The min allowable angle of the arm */
    public static final double MIN_ARM_ANGLE_DEGREES = -120.; // TODO: Confirm this value

    /** The max allowable extension of the arm */
    public static final double MAX_ARM_EXTENSION_INCHES = 20.; // TODO: Confirm this value
    /** The min allowable extension of the arm */
    public static final double MIN_ARM_EXTENSION_INCHES = 0.; // TODO: Confirm this value

    /**
     * Unit conversion from motor rotation to arm rotation degrees
     * <p>
     * 360 degrees divided by output stackup
     * <p>
     * Output stackup: Gear ratio times pulley ratio times empirical fudge factor
     */
    public static final double ARM_ROTATION_CONVERSION = (360. / ((4. * 4. * 3) * (1.64) * (196. / 180.))); // TODO:
                                                                                                            // Count
                                                                                                            // teeth to
                                                                                                            // eliminate
                                                                                                            // fudge
                                                                                                            // factor
    /**
     * Maximum possible arm rotation velocity in degrees per second
     * <p>
     * NEO max output velocity times unit conversion
     */
    public static final double MAX_ARM_VELOCITY_DEGREES_PER_SECOND = (5676. / 60.) * ARM_ROTATION_CONVERSION;

    /**
     * Unit conversion from motor rotation to arm extension inches
     * <p>
     * 1 rotation divided by output stackup
     * <p>
     * Output stackup: Gear ratio times pitch circumference
     */
    public static final double ARM_EXTENSION_CONVERSION = (1. / (4. * 4. * 3.)) * (3. * Math.PI);
    /**
     * Maximum possible arm extension velocity in inches per second
     * <p>
     * NEO max output velocity times unit conversion
     */
    public static final double MAX_ARM_VELOCITY_INCHES_PER_SECOND = (5676. / 60.) * ARM_EXTENSION_CONVERSION;

    // Arm Rotation PID and Feed Forward constants
    public static final double ARM_ROTATION_P = 0.88301;
    public static final double ARM_ROTATION_I = 0.;
    public static final double ARM_ROTATION_D = 0.094929;
    public static final double ARM_ROTATION_S = 0.36254;
    public static final double ARM_ROTATION_G = 0.61796;
    public static final double ARM_ROTATION_V = 0.032231;
    public static final double ARM_ROTATION_A = 0.0031809;
    /** Tolerance on arm angle for PID control */
    public static final double ARM_ROTATION_P_TOLERANCE = 2.; // TODO: Tune based of geometry/testing.

    // Arm Extension PID and Feed Forward constants
    public static final double ARM_EXTENSION_P = 4.1535;
    public static final double ARM_EXTENSION_I = 0.;
    public static final double ARM_EXTENSION_D = 0.37499;
    public static final double ARM_EXTENSION_S = 0.20016;
    public static final double ARM_EXTENSION_G = 0.022996;
    public static final double ARM_EXTENSION_V = 0.63034;
    public static final double ARM_EXTENSION_A = 0.015941;
    /** Tolerance on arm extension for PID control */
    public static final double ARM_EXTENSION_P_TOLERANCE = 1.; // TODO: Tune based of geometry/testing.

    // Claw constants
    // CAN IDs
    public static final int RIGHT_INTAKE_MOTOR = 40;
    public static final int LEFT_INTAKE_MOTOR = 41;

    // Max rotational speed of the claw in radians per second
    // Free motor speed multiplied by gear ratio
    public static final double CLAW_ROTATION_CONVERSION = (360. / (4. * 4. * 4.));
    public static final double MAX_CLAW_VELOCITY_DEGREES_PER_SECOND = (11000. / 60.) * CLAW_ROTATION_CONVERSION;

    // Camera constants
    public static final double CAMERA_HEIGHT = 1.;
    public static final double SCORE_TAG_HEIGHT_METERS = 18.25 * 0.0254;
    public static final double HUMAN_PLAYER_TAG_HEIGHT_METERS = 27.375 * 0.0254;
    public static final double[] TAG_HEIGHT = {
            SCORE_TAG_HEIGHT_METERS,
            SCORE_TAG_HEIGHT_METERS,
            SCORE_TAG_HEIGHT_METERS,
            HUMAN_PLAYER_TAG_HEIGHT_METERS,
            HUMAN_PLAYER_TAG_HEIGHT_METERS,
            SCORE_TAG_HEIGHT_METERS,
            SCORE_TAG_HEIGHT_METERS,
            SCORE_TAG_HEIGHT_METERS
    };
    public static final double CAMERA_PITCH = 0.;

    public static final double SCORE_LOCATION_X_METERS = 1.;
    public static final double[] SCORE_LOCATION_Y_METERS = {
            -22. * 0.0254, // Left column
            0. * 0.0254, // Center column
            22. * 0.0254 // Right column
    };

    public static final double HUMAN_PLAYER_STATION_X_METERS = 1.;
    public static final double HUMAN_PLAYER_STATION_Y_METERS = 0.;

    public static final double POINT_OF_DECREASE_METERS = 2;
}
