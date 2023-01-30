// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Maximum battery voltage
    public static final double MAX_VOLTAGE = 12.0;

    // The left-to-right distance between the drivetrain wheels
    // Should be measured from center to center
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.47; // FIXME: confirm value for new robot

    // The front-to-back distance between the drivetrain wheels.
    // Should be measured from center to center
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.47; // FIXME: confirm value for new robot

    // The maximum linear velocity of the robot in meters per second.
    // This is a measure of how fast the robot can move linearly.
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676 / 60
            * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
            * SdsModuleConfigurations.MK4I_L2.getWheelDiameter()
            * Math.PI;

    // The maximum angular velocity of the robot in radians per second.
    // This is a measure of how fast the robot can rotate in place.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    // The max allowable angle of the arm
    // This should be smaller then the angle at which it hits a hardstop/limitswitch
    public static final double MAX_ARM_ANGLE_DEGREES = 105; // FIXME: Confirm this value is samller than hardstop/limitswitch

    // This is the max allowable extenstion of the arm
    // This should be smaller than the value at which it hits a hardstop/limitswitch
    public static final double MAX_ARM_EXTENSION_METERS = 25 * 0.0254; // FIXME: Confirm this value is samller than hardstop/limitswitch

    // This is the minimum angle at which the arm will hit the robot if it is extended
    public static final double ARM_FRAME_PERIMITER_ANGLE_DEGRESS = 30; // FIXME: Confirm this value once funnel is built

    // This is the angle at which the arm hits the ground if the arm is fully extended
    public static final double ARM_GROUND_ANGLE_DEGRESS = 45; // FIXME: Confirm this value from CAD

    // Max rotational speed of the arm in radians per second
    // Free motor speed multiplied by gear ratio, multiplied by pully ratio
    public static final double MAX_ARM_VELOCITY_RADIANS_PER_SECOND = (5676 / 60) * (4 * 4 * 3) * (1.64) * (2 * Math.PI);
    
    // Max extension speed of the arm in meters per second.
    // Free motor speed multiplied by gear ratio, multiplied by pitch diameter
    public static final double MAX_ARM_VELOCITY_METERS_PER_SECOND = (11000 / 60) * (4 * 4 * 3) * (3 * .0254);

    // CAN ID's are specific to the indivual CAN device (motor, controller, encoder, etc.)
    // Steer offsets are set such that the encoders read zero with all wheels facing forward
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 15; // FIXME: need CAN ID for new robot
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 16; // FIXME: need CAN ID for new robot
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 32; // FIXME: need CAN ID for new robot
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(268.4); // FIXME: need offsets for new robot

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12; // FIXME: need CAN ID for new robot
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11; // FIXME: need CAN ID for new robot
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 34; // FIXME: need CAN ID for new robot
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(129.1); // FIXME: need offsets for new robot

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 17; // FIXME: need CAN ID for new robot
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 18; // FIXME: need CAN ID for new robot
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 31; // FIXME: need CAN ID for new robot
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(84.3); // FIXME: need offsets for new robot

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 14; // FIXME: need CAN ID for new robot
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 13; // FIXME: need CAN ID for new robot
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 33; // FIXME: need CAN ID for new robot
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(1.9); // FIXME: need offsets for new robot

    public static final int LEFT_ARM_MOTOR = 19;
    public static final int RIGHT_ARM_MOTOR = 20;
    public static final int ARM_EXTENSION_MOTOR = 21;
}
