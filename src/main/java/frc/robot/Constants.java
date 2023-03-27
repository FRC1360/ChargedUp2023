// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

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
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(218.4075 + 90.0); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 20; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 21; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(310.588 + 90.0); // FIXME Measure and set front right steer offset
    // 2, 72.098
    // 1, 40.341

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 30; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 31; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 2; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(342.0 + 90.0); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 40; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 41; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(54.767 + 180.0 + 90.0); // FIXME Measure and set back right steer offset


    //GENERAL
    public static double NEO_ENCODER_TICKS_PER_REV = 42;

    //SHOULDER
    public static final int SHOULDER_MOTOR_MASTER = 50;
    public static final int SHOULDER_MOTOR_SLAVE = 51;
    public static final int SHOULDER_ENCODER = 0;
    //public static final double SHOULDER_ENCODER_OFFSET = 0.542;
    public static final double SHOULDER_ENCODER_OFFSET = 0.534;
    public static final double SHOULDER_GEAR_RATIO = (11.0 / 52.0) * (30.0 / 68.0) * (12.0 / 60.0);
    public static final double SHOULDER_MANUAL_OVERRIDE_RANGE = 20.0;
    public static final double SHOULDER_HOME_ANGLE = -90.0;  

    // WRIST
    public static final int WRIST_MOTOR = 54;
    public static final double WRIST_MANUAL_OVERRIDE_RANGE = 20.0;
    public static final int WRIST_ENCODER = 1;
    public static final double WRIST_ENCODER_OFFSET = 0.642;
    public static final double WRIST_GEAR_RATIO = (1.0 / 64.0) * (35.0 / 60.0);
    //public static final double WRIST_HOME_ANGLE = 140.0; 
    public static final double WRIST_HOME_ANGLE = 140.0;  // Originally 170.0
    

    // ARM
    public static final int ARM_MOTOR_MASTER = 52;
    public static final int ARM_MOTOR_SLAVE = 53;
    public static final double ARM_GEAR_RATIO = (11.0 / 52.0) * (30.0 / 68.0) * (18.0 / 36.0);
    public static final double ARM_DRUM_DIAMETER = 2.5;  // Arm Drum Diameter in inches
    public static final double ARM_PULLEY_BLOCK_RATIO = 1.0;
    public static final double ARM_MANUAL_OFFSET_RANGE = 5.0; 

    //Intake
    public static final int LEAD_INTAKE_MOTOR_ID = 60; 
    public static final int FOLLOW_INTAKE_MOTOR_ID = 61;
    public static final int INTAKE_SENSOR_PORT = 0; 


    public final class ARM_POSITION { 
        public static final double HIGH_GOAL = -25.0; 
        public static final double MID_GOAL = -15.0; 
        public static final double LOW_GOAL = -5.0; 
    }

    public static final int LIMIT_SWITCH_ARM = 4; // TODO SET THE CORRECT INPUT

}
