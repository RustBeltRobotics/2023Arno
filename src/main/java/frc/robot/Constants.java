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
    public static final int DRIVE_SMART_CURRENT_LIMIT = 30;
    /** Smart steer motor current */
    public static final int STEER_SMART_CURRENT_LIMIT = 40;
    /** Secondary drive motor current */
    public static final int DRIVE_SECONDARY_CURRENT_LIMIT = 80;
    /** Secondary steer motor current */
    public static final int STEER_SECONDARY_CURRENT_LIMIT = 80;
    /** Smart current limit applied to NEOs */
    public static final int NEO_SMART_CURRENT_LIMIT = 60;
    /** Secondary current limit applied to NEOs */
    public static final int NEO_SECONDARY_CURRENT_LIMIT = 80;
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
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.47;

    /**
     * The front-to-back distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.47;

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

    public static final double MAX_VELOCITY_PRECISION_MODE_METERS_PER_SECOND = 0.5;

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

    // Steer PID Constants
    public static final double STEER_P = 0.008;
    public static final double STEER_I = 0.;
    public static final double STEER_D = 0.0002;
    
    /** Max acceleration while balancing autonomously. Meters per second */
    public static final double MAX_BALANCE_VELOCITY = 0.11;

    /** FIXME: What are the units?? */
    public static final double BALANCE_VELOCITY_TOLERANCE = 7; // FIXME: Confirm value

    /** FIXME: What are the units?? */
    public static final double BALANCE_ANGLE_TOLERANCE = 13; // FIXME: Confirm value

    // Balance PID Constants
    public static final double BALANCE_P = 0.25;

    /** Max velocity while following a trajectory. Meters per second */
    public static final double MAX_TRAJECTORY_VELOCITY = 3.5;

    /** Max acceleration while following a trajectory. Meters per second per second */
    public static final double MAX_TRAJECTORY_ACCELERATION = 1.25;

    // Translation PID Constants
    public static final double TRANSLATION_P = 2.;

    // Rotation PID Constants
    public static final double ROTATION_P = 2.;

    /** Factor applied to maximum drive velocity in manual mode, high speed mode */
    public static final double MAX_SPEED_FACTOR_HIGH = 0.8;
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
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.62;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -246.09;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -288.37;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -345.23;

    // Arm Constants
    // CAN IDs
    public static final int LEFT_ARM_MOTOR = 19;
    public static final int RIGHT_ARM_MOTOR = 20;
    public static final int ARM_EXTENSION_MOTOR = 21;
    public static final int ARM_ABSOLUTE_ENCODER = 22;

    /** The max allowable angle of the arm */
    public static final double MAX_ARM_ANGLE_DEGREES = 120.;
    /** The min allowable angle of the arm */
    public static final double MIN_ARM_ANGLE_DEGREES = -120.;

    /** The max allowable extension of the arm */
    public static final double MAX_ARM_EXTENSION_INCHES = 24.;
    /** The min allowable extension of the arm */
    public static final double MIN_ARM_EXTENSION_INCHES = 0.;
    /** The angle offset in degress for the arm's absolute encoder */
    public static final double ARM_ABSOLUTE_OFFSET = 138.427734375;

    /**
     * Unit conversion from motor rotation to arm rotation degrees
     * <p>
     * 360 degrees divided by output stackup
     * <p>
     * Output stackup: Gear ratio times pulley ratio
     */
    public static final double ARM_ROTATION_CONVERSION = (360. / ((4. * 4. * 3.) * (36./20.)));
    
    /**
     * Maximum possible arm rotation velocity in degrees per second
     * <p>
     * NEO max output velocity times unit conversion
     */
    public static final double MAX_ARM_VELOCITY_DEGREES_PER_SECOND = (5676. / 60.) * ARM_ROTATION_CONVERSION;

    public static final double ARM_ROTATION_TRAINING_WHEELS = 1.;

    public static final double ARM_EXTENSION_TRAINING_WHEELS = 0.25;

    /**
     * Unit conversion from motor rotation to arm extension inches
     * <p>
     * 1 rotation divided by output stackup
     * <p>
     * Output stackup: Gear ratio times pitch circumference
     */
    public static final double ARM_EXTENSION_CONVERSION = (1. / (4. * 4.)) * (3. * Math.PI);
    /**
     * Maximum possible arm extension velocity in inches per second
     * <p>
     * NEO max output velocity times unit conversion
     */
    public static final double MAX_ARM_VELOCITY_INCHES_PER_SECOND = (5676. / 60.) * ARM_EXTENSION_CONVERSION;

    // Arm Rotation PID and Feed Forward constants
    public static final double ARM_ROTATION_P = 0.24725;
    public static final double ARM_ROTATION_I = 0.;
    public static final double ARM_ROTATION_D = 0.00623311;
    public static final double ARM_ROTATION_S = 0.39307;
    public static final double ARM_ROTATION_G = 0.026756;
    public static final double ARM_ROTATION_V = 0.03304;
    public static final double ARM_ROTATION_A = 0.0014601;

    /** Degrees */
    public static final double ARM_ROTATION_TOLERANCE = 1.;

    // Arm Extension PID and Feed Forward constants
    public static final double ARM_EXTENSION_P = 2.5145;
    public static final double ARM_EXTENSION_I = 0.;
    public static final double ARM_EXTENSION_D = 0.060068;
    public static final double ARM_EXTENSION_S = 0.20017;
    public static final double ARM_EXTENSION_G = 0.;
    public static final double ARM_EXTENSION_V = 0.63212;
    public static final double ARM_EXTENSION_A = 0.016752;

    /** Inches */
    public static final double ARM_EXTENSION_TOLERANCE = .5;

    // Intake constants
    // CAN IDs
    public static final int RIGHT_INTAKE_MOTOR = 40;
    public static final int LEFT_INTAKE_MOTOR = 41;

    // Camera constants
    public static final double CAMERA_X = ((24./2.) - 10.) * 0.0254;
    public static final double CAMERA_Y = (-((24./2.) - 8.75)) * 0.0254;
    public static final double CAMERA_Z = (25.5625) * 0.0254;
    public static final double[] TAG_X = {
            610.77 * .0254,
            610.77 * .0254,
            610.77 * .0254,
            636.96 * .0254,
            14.25 * .0254,
            40.45 * .0254,
            40.45 * .0254,
            40.45 * .0254
    };
    public static final double[] TAG_Y = {
            42.19 * .0254,
            108.19 * .0254,
            174.19 * .0254,
            265.74 * .0254,
            265.74 * .0254,
            174.19 * .0254,
            108.19 * .0254,
            42.19 * .0254
    };
    public static final double[] TAG_Z = {
            18.22 * 0.0254,
            18.22 * 0.0254,
            18.22 * 0.0254,
            27.38 * 0.0254,
            27.38 * 0.0254,
            18.22 * 0.0254,
            18.22 * 0.0254,
            18.22 * 0.0254
    };
    public static final double CAMERA_PITCH = 0.;

    public static final double POINT_OF_DECREASE_METERS = 2;

    // Location Presets
    /** Distance in meters. frame plus bumpers */
    public static final double ROBOT_WIDTH = (24. + 6.) * .0254;

    /** Distance in meters. */
    public static final double FIELD_LENGTH = 651.2225 * .0254;

    /** Distance in meters. Half the length of the robot */
    public static final double DRIVE_X_PRESET_SCORE_BLUE = 54.25 * .0254 + ROBOT_WIDTH / 2.;
    /** Distance in meters. Half the length of the robot */
    public static final double DRIVE_X_PRESET_SCORE_RED = FIELD_LENGTH - DRIVE_X_PRESET_SCORE_BLUE;

    /** Distance in meters. */
    public static final double[] DRIVE_Y_PRESET_SCORE_BLUE = {
        20.185 * .0254,
        42.185 * .0254,
        64.185 * .0254,
        86.185 * .0254,
        108.185 * .0254,
        130.185 * .0254,
        152.185 * .0254,
        174.185 * .0254,
        196.185 * .0254
    };
    /** Distance in meters. */
    public static final double[] DRIVE_Y_PRESET_SCORE_RED = DRIVE_Y_PRESET_SCORE_BLUE;

    /** Distance in meters. 0 -> chute station, 1 -> double station left, 2 -> double station right */
    public static final double[] DRIVE_X_PRESET_HUMANPLAYER_BLUE = {
        91.2685 * .0254,
        14. * .0254 + ROBOT_WIDTH / 2.,
        14. * .0254 + ROBOT_WIDTH / 2.
    };
    /** Distance in meters. 0 -> chute station, 1 -> double station left, 2 -> double station right */
    public static final double[] DRIVE_X_PRESET_HUMANPLAYER_RED = {
        FIELD_LENGTH - DRIVE_X_PRESET_HUMANPLAYER_BLUE[0],
        FIELD_LENGTH - DRIVE_X_PRESET_HUMANPLAYER_BLUE[1],
        FIELD_LENGTH - DRIVE_X_PRESET_HUMANPLAYER_BLUE[2],
    };

    /** Distance in meters. 0 -> chute station, 1 -> double station left, 2 -> double station right */
    public static final double[] DRIVE_Y_PRESET_HUMANPLAYER_BLUE = {
        315.5975 * .0254 - ROBOT_WIDTH / 2.,
        233.6331 * .0254,
        297.1737 * .0254
    };
    /** Distance in meters. 0 -> chute station, 1 -> double station left, 2 -> double station right */
    public static final double[] DRIVE_Y_PRESET_HUMANPLAYER_RED = {
        DRIVE_Y_PRESET_HUMANPLAYER_BLUE[0],
        DRIVE_Y_PRESET_HUMANPLAYER_BLUE[2],
        DRIVE_Y_PRESET_HUMANPLAYER_BLUE[1]
    };

    public static final double CHARGE_STATION_X_BLUE = 153. * .0254;

    public static final double CHARGE_STATION_X_RED = FIELD_LENGTH - CHARGE_STATION_X_BLUE;

    public static final double CHARGE_STATION_Y_BLUE = 108.015 * .0254;

    public static final double CHARGE_STATION_Y_RED = CHARGE_STATION_Y_BLUE;

    public static final double MOBILITY_X_BLUE = 193.125 * .0254;
    
    public static final double MOBILITY_X_RED = FIELD_LENGTH - MOBILITY_X_BLUE;
    
    public static final double MOBILITY_Y_BLUE = 108.015 * .0254;
    
    public static final double MOBILITY_Y_RED = MOBILITY_Y_BLUE;

    public static final double GROUND_PIECE_X_BLUE = 278.25 * .0254;

    public static final double GROUND_PIECE_X_RED = FIELD_LENGTH - GROUND_PIECE_X_BLUE;

    public static final double[] GROUND_PIECE_Y_BLUE = {
        36.185 * .0254,
        84.185 * .0254,
        132.185 * .0254,
        180.185 * .0254
    };

    public static final double[] GROUND_PIECE_Y_RED = GROUND_PIECE_Y_BLUE;

    /** Angles are in degrees. Left column is for cone, right column for cube */
    public static final double[] ARM_ANGLE_PRESET_SCORE = {
        100.,
        90.,
        45.
    };

    /** Extensions are in inches. Left column is for cone, right column for cube */
    public static final double[] ARM_EXTENSION_PRESET_SCORE = {
        22.,
        2.75,
        0.
    };

    /** Angles are in degrees. 0 -> chute station, 1 -> double station */
    public static final double[] ARM_ANGLE_PRESET_HUMANPLAYER = {
        -65.,
        92.5
    };
        
    /** Extensions are in inches. 0 -> chute station, 1 -> double station */
    public static final double[] ARM_EXTENSION_PRESET_HUMANPLAYER = {
        0.,
        0.
    };

    /** Angles are in degrees. 0 -> back, 1 -> front */
    public static final double[] ARM_ANGLE_PRESET_GROUND_PICKUP = {
        -30.,
        35.
    };
        
    /** Extensions are in inches. 0 -> back, 1 -> front */
    public static final double[] ARM_EXTENSION_PRESET_GROUND_PICKUP = {
        16.5,
        15.5
    };
}
