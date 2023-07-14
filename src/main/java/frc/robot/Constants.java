// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.lib.util.PIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /*
     * Swerve Constants (newly added ones)
     */
    public static final class Swerve {
        /* Module Specific Constants */
        public static final String[] MODULE_NAMES = { "Front Left", "Front Right", "Back Left", "Back Right" }; // module
                                                                                                                // #0,
        // #1, #2, #3

        public static int PEAK_CURRENT_LIMIT = 50;
        public static int CONTINUOUS_CURRENT_LIMIT = 40;
        public static boolean ANGLE_INVERT = true;
        public static boolean DRIVE_INVERT = true;
        public static IdleMode IDLE_MODE = IdleMode.kBrake;

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.61;
        public static final double WHEEL_BASE = 0.61;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final PIDConstants DRIVE_PID = new PIDConstants(0.1, 0.0, 0.0); // you need to tune me yay :D

        /* Drive Motor Conversion Factors */
        public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0); // 6.75:1
        public static final double ANGLE_GEAR_RATIO = 150.0 / 7.0; // 150/7:1
        public static final double DRIVE_CONVERSION_POSITION_FACTOR = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
        public static final double DRIVE_CONVERSION_VELOCITY_FACTOR = DRIVE_CONVERSION_POSITION_FACTOR / 60.0;
        public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;
        public static final double MAX_SPEED = 14.5 / 3.28084;

    }

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.61; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.61; // FIXME Measure and set wheelbase

    public static final double ROBOT_MAX_VELOCITY_METERS_PER_SECOND = 14.5 / 3.28084; // ft/s divide ft/m to convert to
                                                                                      // m/s

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = ROBOT_MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    // SDS Module Configurations
    public static final double SWERVE_WHEEL_DIAMETER = 0.10033; // in meters
    public static final double SWERVE_DRIVE_GEAR_RATIO = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double SWERVE_STEER_GEAR_RATIO = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double SWERVE_DRIVE_MOTOR_RAMP_RATE = 1.0; // Time is seconds for acceleration from 0 to full
                                                                   // speed
    public static final double SWERVE_STEER_MOTOR_RAMP_RATE = 1.0;
    public static final int SWERVE_ENCODER_PULSE_PER_REV = 1;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 0; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -(218.4075 + 90.0); // FIXME Measure and set front left
                                                                                    // steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 20; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 21; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -(310.588 + 90.0); // FIXME Measure and set front right
                                                                                    // steer offset
    // 2, 72.098
    // 1, 40.341

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 30; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 31; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 2; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -(342.0 + 90.0); // FIXME Measure and set back left steer
                                                                                // offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 40; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 41; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -(54.767 + 180.0 + 90.0); // FIXME Measure and set back
                                                                                          // right steer offset

    public static final class Drivetrain {
        public static final double DRIVE_MOTION_PROFILE_MAX_VELOCITY = 4.000;
        public static final double DRIVE_MOTION_PROFILE_MAX_ACCELERATION = 3.550;
        public static final double ROTATION_MOTION_PROFILE_MAX_VELOCITY = 180.0;
        public static final double ROTATION_MOTION_PROFILE_MAX_ACCELERATION = 180.0;
    }

    // GENERAL
    public static double NEO_ENCODER_TICKS_PER_REV = 42;

    // SHOULDER
    public static final int SHOULDER_MOTOR_MASTER = 50;
    public static final int SHOULDER_MOTOR_SLAVE = 51;
    public static final int SHOULDER_ENCODER = 0;
    // public static final double SHOULDER_ENCODER_OFFSET = 0.542;
    public static final double SHOULDER_ENCODER_OFFSET = 0.095;
    public static final double SHOULDER_GEAR_RATIO = (11.0 / 52.0) * (30.0 / 68.0) * (12.0 / 60.0);
    public static final double SHOULDER_MANUAL_OVERRIDE_RANGE = 20.0;
    public static final double MAX_SHOULDER_ANGLE = 90.0;
    public static final double MIN_SHOULDER_ANGLE = -150.0;

    // WRIST
    public static final int WRIST_MOTOR = 54;
    public static final double WRIST_MANUAL_OVERRIDE_RANGE = 20.0;
    public static final int WRIST_ENCODER = 1;
    public static final double WRIST_ENCODER_OFFSET = 0.521;
    public static final double WRIST_GEAR_RATIO = (1.0 / 64.0) * (35.0 / 60.0);
    public static final double WRIST_MAX_ANGLE = 200.0;
    public static final double WRIST_MIN_ANGLE = -165.0;

    // ARM
    public static final int ARM_MOTOR_MASTER = 52;
    public static final int ARM_MOTOR_SLAVE = 53;
    public static final double ARM_GEAR_RATIO = (11.0 / 52.0) * (30.0 / 68.0) * (18.0 / 36.0);
    public static final double ARM_DRUM_DIAMETER = 2.5; // Arm Drum Diameter in inches
    public static final double ARM_PULLEY_BLOCK_RATIO = 1.0;
    public static final double ARM_MANUAL_OFFSET_RANGE = 5.0;
    public static final double ARM_MAX_DISTANCE = 40.0;

    // Intake
    public static final int LEAD_INTAKE_MOTOR_ID = 60;
    public static final int FOLLOW_INTAKE_MOTOR_ID = 61;
    public static final int INTAKE_SENSOR_PORT = 6;

    public static final int LIMIT_SWITCH_ARM = 4; // TODO SET THE CORRECT INPUT

    // HOME_POSITION
    public static final double HOME_POSITION_WRIST = 175.0; // Originally 170.0
    public static final double HOME_POSITION_ARM = 0.0;
    public static final double HOME_POSITION_SHOULDER = -90.0;

    // CONE_INTAKE_POSITION
    public static final double CONE_INTAKE_POSITION_WRIST = 53.0;
    public static final double CONE_INTAKE_POSITION_ARM = 5.8;
    public static final double CONE_INTAKE_POSITION_SHOULDER = -48.0;

    // CUBE_INTAKE_POSITION
    public static final double CUBE_INTAKE_POSITION_WRIST = 95.0;
    public static final double CUBE_INTAKE_POSITION_ARM = 5.5;
    public static final double CUBE_INTAKE_POSITION_SHOULDER = -52.0;

    // CONE_SCORE_HIGH_POSITION
    public static final double CONE_SCORE_HIGH_POSITION_WRIST = -40.0;
    public static final double CONE_SCORE_HIGH_POSITION_ARM = 20.0;
    public static final double CONE_SCORE_HIGH_POSITION_SHOULDER = 37.0;

    // CUBE_SCORE_HIGH_POSITION
    public static final double CUBE_SCORE_HIGH_POSITION_WRIST = 130.0;
    public static final double CUBE_SCORE_HIGH_POSITION_ARM = 5.0;
    public static final double CUBE_SCORE_HIGH_POSITION_SHOULDER = -7.5;

    // SCORE_MID_POSITION
    public static final double CONE_SCORE_MID_POSITION_WRIST = -35.0;
    public static final double CONE_SCORE_MID_POSITION_ARM = HOME_POSITION_ARM;
    public static final double CONE_SCORE_MID_POSITION_SHOULDER = 30.0;

    // CUBE_SCORE_MID_POSITION
    public static final double CUBE_SCORE_MID_POSITION_WRIST = 170.0;
    public static final double CUBE_SCORE_MID_POSITION_ARM = 0.0;
    public static final double CUBE_SCORE_MID_POSITION_SHOULDER = -48.0;

    // SINGLE_SUBSTATION_POSITION
    public static final double SINGLE_SUBSTATION_POSITION_WRIST = 150.0;
    public static final double SINGLE_SUBSTATION_POSITION_ARM = HOME_POSITION_ARM;
    public static final double SINGLE_SUBSTATION_POSITION_SHOULDER = -48.0;

}
