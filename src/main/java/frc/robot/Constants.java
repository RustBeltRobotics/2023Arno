// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.47; //FIXME: confirm value for new robot
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.47; //FIXME: confirm value for new robot

    /**
     * CAN ID's are specific to the indivual SPARK MAXs for each individual CAN device (motor, encoder, etc.)
     * Steer offsets are set such that the encoders read zero with all wheels facing forward
     */
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 15; //FIXME: need CAN ID for new robot
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 16; //FIXME: need CAN ID for new robot
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 32; //FIXME: need CAN ID for new robot
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(268.4); //FIXME: need offsets for new robot

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12; //FIXME: need CAN ID for new robot
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11; //FIXME: need CAN ID for new robot
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 34; //FIXME: need CAN ID for new robot
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(129.1); //FIXME: need offsets for new robot

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 17; //FIXME: need CAN ID for new robot
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 18; //FIXME: need CAN ID for new robot
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 31; //FIXME: need CAN ID for new robot
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(84.3); //FIXME: need offsets for new robot

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 14; //FIXME: need CAN ID for new robot
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 13; //FIXME: need CAN ID for new robot
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 33; //FIXME: need CAN ID for new robot
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(1.9); //FIXME: need offsets for new robot
}
