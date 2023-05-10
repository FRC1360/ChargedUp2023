// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OrbitPID;
import swervelib.SwerveDrive;
import swervelib.math.SwerveModuleState2;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DrivetrainSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive; 

  private final ChassisSpeeds STOPPING_SPEEDS = new ChassisSpeeds(0.0, 0.0, 0.0); 
  
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public boolean lockWheels;

  //private SwerveDriveOdometry2022 odometry = new SwerveDriveOdometry2022(m_kinematics, getGyroscopeRotation(), new Pose2d(2.3, 4.5, Rotation2d.fromDegrees(0)));

  public OrbitPID driveRotPID; 

  public DrivetrainSubsystem() {
    lockWheels = false;

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    this.swerveDrive = new SwerveDrive(SwerveConfig.DRIVE_CONFIGURATION, SwerveConfig.CONTROLLER_CONFIGURATION); 

    this.driveRotPID = new OrbitPID(0.1, 0.0, 0.0); 
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    this.swerveDrive.zeroGyro();
  }

  public Rotation2d getGyroscopeRotation() {
    // Inversion has been done in the config
    return this.swerveDrive.getYaw(); 
  }

  /** 
   * @param chassisSpeeds
   * A {@link ChassisSpeeds} object representing: 
   * x is forward (positve)
   * y is to the left (positive)
   * and rotation is counter clockwise (positive)
  */

  public void drive(ChassisSpeeds chassisSpeeds, boolean isFieldRelative) {
    Translation2d xAndYVelocity 
        = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond); 
    
    this.swerveDrive.drive(xAndYVelocity, chassisSpeeds.omegaRadiansPerSecond, 
                              isFieldRelative, true); 
  }

  public void stop() { 
    m_chassisSpeeds = STOPPING_SPEEDS; 
  }

  public double getRollDeg() { 
    return this.swerveDrive.getRoll().getDegrees(); 
  }

  public Pose2d getPose() {
    return this.swerveDrive.getPose();
  }

  public Translation2d getTranslation() { 
    return this.getPose().getTranslation(); 
  }

  public void resetOdometry(Pose2d newPose) { 
    this.swerveDrive.resetOdometry(newPose);
  }

  public void resetDriveEncoders() { 
    this.swerveDrive.resetEncoders();
  }

  public void lockWheels() { 
    SwerveModuleState2[] states = new SwerveModuleState2[4]; 

    for(int i = 0; i < 4; i++) {
      states[i] = new SwerveModuleState2(); 
      states[i].speedMetersPerSecond = 0.0; 
      states[i].omegaRadPerSecond = 0.0; 
    }

    states[0].angle = Rotation2d.fromDegrees(45.0);
    states[1].angle = Rotation2d.fromDegrees(-45.0);
    states[2].angle = Rotation2d.fromDegrees(-45.0);
    states[3].angle = Rotation2d.fromDegrees(45.0);

    this.swerveDrive.setModuleStates(states, false);
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drivetrain_gyro", getGyroscopeRotation().getDegrees());  
    SmartDashboard.putNumber("NavX pitch", this.swerveDrive.getPitch().getDegrees()); 
    SmartDashboard.putNumber("NavX roll", this.getRollDeg()); 
    SmartDashboard.putNumber("Drivetrain_Speed_X", m_chassisSpeeds.vxMetersPerSecond); 
    SmartDashboard.putNumber("Drivetrain_Speed_Y", m_chassisSpeeds.vyMetersPerSecond);
    
    this.swerveDrive.updateOdometry();
  } 
}
