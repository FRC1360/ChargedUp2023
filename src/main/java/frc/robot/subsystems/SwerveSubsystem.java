package frc.robot.subsystems;

import java.lang.reflect.Field;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.SwerveModuleCustom;
import frc.lib.util.NavX;
import frc.lib.util.PIDConstants;
import frc.robot.Constants;
import swervelib.math.SwerveModuleState2;

public class SwerveSubsystem extends SubsystemBase {
  public final NavX navX;
  private SwerveModuleCustom[] swerveModules;
  private Translation2d centerOfRotation = new Translation2d();
  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private final Field2d field = new Field2d();

  private Pose2d lastPose = new Pose2d(0, 0, new Rotation2d());
  private long lastPoseTimestamp = System.currentTimeMillis();
  public Translation2d currentSpeed = new Translation2d(0, 0);
  private PIDConstants anglePID = Constants.Swerve.anglePID;

  public SwerveSubsystem() {
    // Gyro setup
    navX = new NavX();
    navX.setInverted(Constants.Swerve.isGyroInverted);

    // Swerve module setup
    swerveModules = new SwerveModuleCustom[] {
        new SwerveModuleCustom(0, Constants.Swerve.Mod0.constants),
        new SwerveModuleCustom(1, Constants.Swerve.Mod1.constants),
        new SwerveModuleCustom(2, Constants.Swerve.Mod2.constants),
        new SwerveModuleCustom(3, Constants.Swerve.Mod3.constants),
    };

    // Pose estimator
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, navX.getYaw(),
        getPositions(), new Pose2d());

    this.anglePID.sendDashboard("angle pid");
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(),
                rotation,
                navX.getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
        this.centerOfRotation);
    // Get rid of tiny tiny movements in the wheels to have more consistent driving
    // experience
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

    // set the states for each module
    for (SwerveModuleCustom mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

    for (SwerveModuleCustom mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void setModuleStatesDuringAuto(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.AutoConstants.maxSpeed);

    for (SwerveModuleCustom mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void setModuleStates(SwerveModuleState2[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

    for (SwerveModuleCustom mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void resetModuleZeros() {
    for (SwerveModuleCustom mod : swerveModules) {
      mod.resetToAbsolute();
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModuleCustom mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModuleCustom mod : swerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }

    return positions;
  }

  public void brake() {
    SwerveModuleState2[] states = new SwerveModuleState2[4];

    for (int i = 0; i < 4; i++) {
      states[i] = new SwerveModuleState2();
      states[i].speedMetersPerSecond = 0.0;
      states[i].omegaRadPerSecond = 0.0;
    }

    states[0].angle = Rotation2d.fromDegrees(45.0);
    states[1].angle = Rotation2d.fromDegrees(-45.0);
    states[2].angle = Rotation2d.fromDegrees(-45.0);
    states[3].angle = Rotation2d.fromDegrees(45.0);

    setModuleStates(states);
  }

  public void setCoR(Translation2d centerOfRotation) {
    this.centerOfRotation = centerOfRotation;
  }

  public void resetCoR() {
    setCoR(new Translation2d());
  }

  public Pose2d currentPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public Translation2d currentTranslation() {
    return currentPose().getTranslation();
  }

  public void setCurrentPose(Pose2d newPose) {
    swerveDrivePoseEstimator.resetPosition(
        navX.getYaw(),
        getPositions(),
        newPose);
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public String getFormattedPose() {
    var pose = currentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  /** Stolen from photonlib's examples */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    swerveDrivePoseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    swerveDrivePoseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  @Override
  public void periodic() {
    // Data
    SmartDashboard.putNumber("NavX Yaw", navX.getYaw().getDegrees());
    SmartDashboard.putNumber("NavX pitch", navX.getPitch().getDegrees());
    SmartDashboard.putNumber("NavX roll", navX.getRoll().getDegrees());

    for (SwerveModuleCustom module : swerveModules) {
      SmartDashboard.putNumber("Swerve Module #" + module.moduleNumber + " angle", module.getMagEncoder().getDegrees());
      SmartDashboard.putNumber("Swerve Module #" + module.moduleNumber + " target  ", module.targetAngle);
      SmartDashboard.putNumber("Swerve Module #" + module.moduleNumber + "speed", module.getSpeed());
      SmartDashboard.putNumber("Swerve Module #" + module.moduleNumber + "target speed", module.targetSpeed);
    }

    // Estimator update
    swerveDrivePoseEstimator.update(navX.getYaw(), getPositions());
    field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());

    // get speed
    long currentTime = System.currentTimeMillis();
    long deltaTime = currentTime - lastPoseTimestamp;
    currentSpeed = new Translation2d((currentPose().getX() - lastPose.getX()) / (deltaTime / 1000d),
        (currentPose().getY() - lastPose.getY()) / (deltaTime / 1000d));
    lastPose = swerveDrivePoseEstimator.getEstimatedPosition();
    lastPoseTimestamp = System.currentTimeMillis();

    SmartDashboard.putNumber("Speed X", currentSpeed.getX());
    SmartDashboard.putNumber("Speed Y", currentSpeed.getY());

    SmartDashboard.putString("Estimated Pose", this.getFormattedPose());
  }

}
