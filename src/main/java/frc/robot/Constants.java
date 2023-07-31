// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 0; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(218.4075 + 90.0); // FIXME Measure and
                                                                                                  // set front left
                                                                                                  // steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 20; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 21; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(310.588 + 90.0); // FIXME Measure and
                                                                                                  // set front right
                                                                                                  // steer offset
    // 2, 72.098
    // 1, 40.341

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 30; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 31; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 2; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(342.0 + 90.0); // FIXME Measure and set
                                                                                              // back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 40; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 41; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(54.767 + 180.0 + 90.0); // FIXME Measure
                                                                                                        // and set back
                                                                                                        // right steer
                                                                                                        // offset

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
    public static final double SHOULDER_ENCODER_OFFSET = 0.045 - 0.027 - 0.102777 - 0.02777 + 0.05 + 0.03888 + 0.02777
            - 0.02
            - 0.07222 + 0.041666; // Measure
    // when
    // arm
    // is
    // parallel to
    // floor
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
    public static final double HOME_POSITION_WRIST = 170.0; // Originally 175.0
    public static final double HOME_POSITION_ARM = 0.0;
    public static final double HOME_POSITION_SHOULDER = -70.0; // -90.0, changed it for ribfest so that the neo doesn't
                                                               // hit the superstructure

    // CONE_INTAKE_POSITION
    public static final double CONE_INTAKE_POSITION_WRIST = 53.0;
    public static final double CONE_INTAKE_POSITION_ARM = 4.0;
    public static final double CONE_INTAKE_POSITION_SHOULDER = -48.0; // -48

    // CUBE_INTAKE_POSITION
    public static final double CUBE_INTAKE_POSITION_WRIST = 95.0;
    public static final double CUBE_INTAKE_POSITION_ARM = 5.5;
    public static final double CUBE_INTAKE_POSITION_SHOULDER = -52.0;

    // CONE_SCORE_HIGH_POSITION
    public static final double CONE_SCORE_HIGH_POSITION_WRIST = -40.0;
    public static final double CONE_SCORE_HIGH_POSITION_ARM = 17.5; // Originally 20
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
