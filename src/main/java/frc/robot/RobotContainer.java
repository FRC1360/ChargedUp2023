// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.AutoBalance;
import frc.robot.autos.CubeHighAndBalanceAuto;
import frc.robot.autos.DriveStraightAuto;
import frc.robot.autos.pathplanner_autos.DriveForwardAuto;
import frc.robot.autos.ConeHighAndDriveAuto;
import frc.robot.autos.procedures.ConeScoreHighAuto;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.arm.ArmGoToPositionCommand;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.arm.ArmHomeCommand;
import frc.robot.commands.assembly.AssemblyGoToConeIntakeCommand;
import frc.robot.commands.assembly.AssemblyGoToCubeIntakeCommand;
import frc.robot.commands.assembly.AssemblyHighScoreCommand;
import frc.robot.commands.assembly.AssemblyHomePositionCommand;
import frc.robot.commands.assembly.AssemblyMidScoreCommand;
import frc.robot.commands.assembly.AssemblyPickUpSingleSubstationCommand;
import frc.robot.commands.assembly.autoAssembly.AutoAssemblyConeHighScoreCommand;
import frc.robot.commands.intake.IntakeHoldCommand;
import frc.robot.commands.intake.ManualIntakeCommand;
import frc.robot.commands.intake.ManualPutdownCommand;
import frc.robot.commands.shoulder.ShoulderGoToPositionCommand;
import frc.robot.commands.shoulder.ShoulderHoldCommand;
import frc.robot.commands.shoulder.ShoulderMoveManual;
import frc.robot.commands.vision.StrafeAlign;
import frc.robot.commands.wrist.WristGoToPositionCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.simulation.Simulator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.StateMachine;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.SwerveDrive.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandJoystick left_controller = new CommandJoystick(0);
  private final CommandJoystick right_controller = new CommandJoystick(1);
  private final CommandXboxController operatorController = new CommandXboxController(2);

  // The robot's subsystems and commands are defined here...
  // public final DrivetrainSubsystem m_drivetrainSubsystem = new
  // DrivetrainSubsystem();
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  // public final DrivetrainSubsystem m_drivetrainSubsystem = new
  // DrivetrainSubsystem(
  // new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  public final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem(
      () -> operatorController.getRightY() * Constants.SHOULDER_MANUAL_OVERRIDE_RANGE,
      operatorController.rightBumper());
  private final ShoulderSubsystem.ShoulderWristMessenger shoulderMessenger = shoulderSubsystem.new ShoulderWristMessenger();
  public final WristSubsystem wristSubsystem = new WristSubsystem(shoulderMessenger,
      () -> operatorController.getLeftY() * Constants.WRIST_MANUAL_OVERRIDE_RANGE, () -> false);
  public final ArmSubsystem armSubsystem = new ArmSubsystem(
      () -> operatorController.getLeftY() * Constants.ARM_MANUAL_OFFSET_RANGE, operatorController.rightBumper());
  private final ArmSubsystem.ArmShoulderMessenger armMessenger = armSubsystem.new ArmShoulderMessenger();
  private final Vision vision = new Vision();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final StateMachine sm = new StateMachine();

  public final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final CubeHighAndBalanceAuto highConeAndBalanceAuto = new CubeHighAndBalanceAuto(swerveSubsystem,
      shoulderSubsystem, shoulderMessenger,
      wristSubsystem, armSubsystem, intakeSubsystem, armMessenger, ledSubsystem, sm);
  private final ConeHighAndDriveAuto highConeAndDriveAuto = new ConeHighAndDriveAuto(swerveSubsystem,
      shoulderSubsystem, shoulderMessenger,
      wristSubsystem, armSubsystem, intakeSubsystem, armMessenger, ledSubsystem, sm);

  private final ConeScoreHighAuto highConeAuto = new ConeScoreHighAuto(swerveSubsystem, shoulderSubsystem,
      shoulderMessenger, wristSubsystem,
      armSubsystem, intakeSubsystem, armMessenger, ledSubsystem, sm);

  private final DriveStraightAuto driveStraightAuto = new DriveStraightAuto(swerveSubsystem);

  private final Simulator sim = new Simulator(swerveSubsystem);

  // private final DriveStraightAuto driveStraightAuto = new
  // DriveStraightAuto(m_drivetrainSubsystem, wristSubsystem);
  // private final EngageStationAuto engageStationAuto = new
  // EngageStationAuto(m_drivetrainSubsystem);
  // private final Simulator sim = new Simulator(m_drivetrainSubsystem);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    swerveSubsystem.setDefaultCommand(new DefaultDriveCommand(
        swerveSubsystem,
        () -> -modifyAxis(left_controller.getY()) * Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(left_controller.getX()) * Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND,
        () -> modifyAxis(right_controller.getX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        right_controller));

    shoulderSubsystem.setDefaultCommand(new ShoulderHoldCommand(shoulderSubsystem, armMessenger,
        () -> this.operatorController.getLeftTriggerAxis()));
    // shoulderSubsystem.setDefaultCommand(new ShoulderMoveManual(shoulderSubsystem,
    // () -> this.operatorController.getLeftY()));
    wristSubsystem.setDefaultCommand(
        new WristHoldCommand(wristSubsystem, () -> this.operatorController.getLeftTriggerAxis()));
    armSubsystem.setDefaultCommand(new ArmHoldCommand(this.armSubsystem));
    intakeSubsystem.setDefaultCommand(new IntakeHoldCommand(this.intakeSubsystem));

    initializeRobot();
    // Configure the button bindings
    configureButtonBindings();
  }

  public void initializeRobot() {
    // autoChooser.setDefaultOption("One side, two cargo, balance", leftConeAuto);
    autoChooser.setDefaultOption("No auto", new WaitCommand(15));
    autoChooser.addOption("High cube and balance", highConeAndBalanceAuto);
    autoChooser.addOption("High cone and drive straight", highConeAndDriveAuto);
    autoChooser.addOption("Only high cone score", highConeAuto);
    autoChooser.addOption("Only drive straight", driveStraightAuto);
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    /*
     * new Trigger(m_controller::getBackButton)
     * // No requirements because we don't need to interrupt anything
     * .onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope));
     */

    left_controller.button(1).onTrue(new InstantCommand(swerveSubsystem.navX::resetGyro));

    operatorController.a().and(() -> sm.getAtHome()).onTrue((new AssemblyGoToCubeIntakeCommand(shoulderSubsystem,
        shoulderMessenger, wristSubsystem, armSubsystem, armMessenger, intakeSubsystem, ledSubsystem, sm)));
    operatorController.y().and(() -> sm.getAtHome()).onTrue(new AssemblyGoToConeIntakeCommand(shoulderSubsystem,
        shoulderMessenger, wristSubsystem, armSubsystem, armMessenger, intakeSubsystem, ledSubsystem, sm));
    operatorController.b().and(() -> sm.getAtHome())
        .onTrue(new AssemblyMidScoreCommand(shoulderSubsystem, shoulderMessenger, wristSubsystem, armSubsystem,
            armMessenger, ledSubsystem, () -> operatorController.leftBumper().getAsBoolean(), sm));
    operatorController.x().and(() -> !sm.getAtHome()).onTrue(new AssemblyHomePositionCommand(shoulderSubsystem,
        shoulderMessenger, wristSubsystem, armSubsystem, armMessenger, ledSubsystem, sm));
    operatorController.povUp().and(() -> sm.getAtHome())
        .onTrue(new AssemblyHighScoreCommand(shoulderSubsystem, shoulderMessenger, wristSubsystem, armSubsystem,
            armMessenger, () -> operatorController.leftBumper().getAsBoolean(), ledSubsystem, sm));
    operatorController.povDown().and(() -> sm.getAtHome())
        .onTrue(new AssemblyPickUpSingleSubstationCommand(shoulderSubsystem, wristSubsystem, armSubsystem,
            shoulderMessenger, armMessenger, intakeSubsystem, ledSubsystem, sm));

    new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.05)
        .whileTrue(new ManualIntakeCommand(intakeSubsystem, () -> operatorController.getLeftTriggerAxis()));
    new Trigger(() -> operatorController.getRightTriggerAxis() > 0.05)
        .whileTrue(new ManualPutdownCommand(intakeSubsystem, () -> operatorController.getRightTriggerAxis()));

    // left_controller.button(2).whileTrue(new StrafeAlign(m_drivetrainSubsystem,
    // vision, left_controller::getX, left_controller::getY));
    right_controller.button(1).whileTrue(new ManualPutdownCommand(intakeSubsystem, () -> 1.0));
    left_controller.button(3).whileTrue(new InstantCommand(swerveSubsystem::brake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return null;
    // return autoChooser.getSelected();
    // return highConeAndBalanceAuto;

    // return driveStraightAuto;
    return new DriveForwardAuto(swerveSubsystem).getCommand();
  }

  public Command getArmHomeCommand() {
    return new ArmHomeCommand(armSubsystem);
  }

  public Command getShoulderZeroCommand() {
    return new ShoulderGoToPositionCommand(shoulderSubsystem, Constants.HOME_POSITION_SHOULDER);
  }

  public Command getGoToZeroWristCommand() {
    return new WristGoToPositionCommand(wristSubsystem, Constants.HOME_POSITION_WRIST);
  }

  public Command getIntakeHoldCommand() {
    return new IntakeHoldCommand(intakeSubsystem);
  }

  public Command setSMHomeCommand() {
    return new InstantCommand(() -> sm.setAtHome(true));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
