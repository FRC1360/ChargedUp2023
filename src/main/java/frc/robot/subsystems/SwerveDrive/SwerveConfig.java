package frc.robot.subsystems.SwerveDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;

import static frc.robot.Constants.*;
import swervelib.encoders.DIOAbsoluteEncoderSwerve;
import swervelib.imu.NavXSwerve;
import swervelib.motors.SparkMaxSwerve;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveModulePhysicalCharacteristics;

public final class SwerveConfig {

    public static CANSparkMax frontLeftDriveMotor = new CANSparkMax(FRONT_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless); 
    public static CANSparkMax frontLeftTurnMotor = new CANSparkMax(FRONT_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless); 
    public static CANSparkMax frontRightDriveMotor = new CANSparkMax(FRONT_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless); 
    public static CANSparkMax frontRightTurnMotor = new CANSparkMax(FRONT_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless); 
    public static CANSparkMax backLeftDriveMotor = new CANSparkMax(FRONT_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless); 
    public static CANSparkMax backLeftTurnMotor = new CANSparkMax(FRONT_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless); 
    public static CANSparkMax backRightDriveMotor = new CANSparkMax(FRONT_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless); 
    public static CANSparkMax backRightTurnMotor = new CANSparkMax(FRONT_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless); 


    public static final SwerveModuleConfiguration[] moduleConfigs 
        = new SwerveModuleConfiguration[] {
            // FRONT-LEFT
            new SwerveModuleConfiguration(new SparkMaxSwerve(frontLeftDriveMotor, true), new SparkMaxSwerve(frontLeftTurnMotor, false), 
                new DIOAbsoluteEncoderSwerve(FRONT_LEFT_MODULE_STEER_ENCODER), FRONT_LEFT_MODULE_STEER_OFFSET, 
                DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0, new PIDFConfig(1.0, 0.0, 0.0, 0.0), new PIDFConfig(1.0, 0.0, 0.0, 0.0), 
                ROBOT_MAX_VELOCITY_METERS_PER_SECOND, new SwerveModulePhysicalCharacteristics(SWERVE_DRIVE_GEAR_RATIO, SWERVE_STEER_GEAR_RATIO, SWERVE_WHEEL_DIAMETER, SWERVE_DRIVE_MOTOR_RAMP_RATE, SWERVE_STEER_MOTOR_RAMP_RATE, 
                                                        SWERVE_ENCODER_PULSE_PER_REV, SWERVE_ENCODER_PULSE_PER_REV), 
                "Front Left Module"), 
            
            // FRONT-RIGHT
            new SwerveModuleConfiguration(new SparkMaxSwerve(frontRightDriveMotor, true), new SparkMaxSwerve(frontRightTurnMotor, false), 
                new DIOAbsoluteEncoderSwerve(FRONT_RIGHT_MODULE_STEER_ENCODER), FRONT_RIGHT_MODULE_STEER_OFFSET, 
                DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0, new PIDFConfig(1.0, 0.0, 0.0, 0.0), new PIDFConfig(1.0, 0.0, 0.0, 0.0), 
                ROBOT_MAX_VELOCITY_METERS_PER_SECOND, new SwerveModulePhysicalCharacteristics(SWERVE_DRIVE_GEAR_RATIO, SWERVE_STEER_GEAR_RATIO, SWERVE_WHEEL_DIAMETER, SWERVE_DRIVE_MOTOR_RAMP_RATE, SWERVE_STEER_MOTOR_RAMP_RATE, 
                                                        SWERVE_ENCODER_PULSE_PER_REV, SWERVE_ENCODER_PULSE_PER_REV), 
                "Front Right Module"), 

            // BACK-LEFT
            new SwerveModuleConfiguration(new SparkMaxSwerve(backLeftDriveMotor, true), new SparkMaxSwerve(backLeftTurnMotor, false), 
                new DIOAbsoluteEncoderSwerve(BACK_LEFT_MODULE_STEER_ENCODER), BACK_LEFT_MODULE_STEER_OFFSET, 
                -DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0, new PIDFConfig(1.0, 0.0, 0.0, 0.0), new PIDFConfig(1.0, 0.0, 0.0, 0.0), 
                ROBOT_MAX_VELOCITY_METERS_PER_SECOND, new SwerveModulePhysicalCharacteristics(SWERVE_DRIVE_GEAR_RATIO, SWERVE_STEER_GEAR_RATIO, SWERVE_WHEEL_DIAMETER, SWERVE_DRIVE_MOTOR_RAMP_RATE, SWERVE_STEER_MOTOR_RAMP_RATE, 
                                                        SWERVE_ENCODER_PULSE_PER_REV, SWERVE_ENCODER_PULSE_PER_REV), 
                "Back Left Module"),
            
            // BACK-RIGHT
            new SwerveModuleConfiguration(new SparkMaxSwerve(backRightDriveMotor, true), new SparkMaxSwerve(backRightTurnMotor, false), 
                new DIOAbsoluteEncoderSwerve(BACK_RIGHT_MODULE_STEER_ENCODER), BACK_RIGHT_MODULE_STEER_OFFSET, 
                -DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0, new PIDFConfig(1.0, 0.0, 0.0, 0.0), new PIDFConfig(1.0, 0.0, 0.0, 0.0), 
                ROBOT_MAX_VELOCITY_METERS_PER_SECOND, new SwerveModulePhysicalCharacteristics(SWERVE_DRIVE_GEAR_RATIO, SWERVE_STEER_GEAR_RATIO, SWERVE_WHEEL_DIAMETER, SWERVE_DRIVE_MOTOR_RAMP_RATE, SWERVE_STEER_MOTOR_RAMP_RATE, 
                                                        SWERVE_ENCODER_PULSE_PER_REV, SWERVE_ENCODER_PULSE_PER_REV), 
                "Back Right Module"), 

            
        }; 
    
    public static final SwerveDriveConfiguration DRIVE_CONFIGURATION 
        = new SwerveDriveConfiguration(
            moduleConfigs, new NavXSwerve(Port.kMXP, (byte) 200), ROBOT_MAX_VELOCITY_METERS_PER_SECOND, true
        );  

    public static final SwerveControllerConfiguration CONTROLLER_CONFIGURATION 
        = new SwerveControllerConfiguration(
            DRIVE_CONFIGURATION, new PIDFConfig(1.0, 0.0, 0.0, 0.0)
            ); 
}
