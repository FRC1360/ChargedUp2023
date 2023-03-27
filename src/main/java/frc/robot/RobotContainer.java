// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.autos.AutoSequence;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.autos.DriveStraightAuto;
import frc.robot.autos.EngageStationAuto;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.arm.ArmGoToPositionCommand;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.arm.ArmHomeCommand;
import frc.robot.commands.arm.ArmTestTuningCommand;
import frc.robot.commands.assembly.AssemblyGoToConeIntakeCommand;
import frc.robot.commands.assembly.AssemblyGoToCubeIntakeCommand;
import frc.robot.commands.assembly.AssemblyHighScoreCommand;
import frc.robot.commands.assembly.AssemblyHomePositionCommand;
import frc.robot.commands.assembly.AssemblyMidScoreCommand;
import frc.robot.commands.intake.IntakeHoldCommand;
import frc.robot.commands.intake.ManualIntakeCommand;
import frc.robot.commands.intake.ManualPutdownCommand;
import frc.robot.commands.shoulder.ShoulderGoToPositionCommand;
import frc.robot.commands.shoulder.ShoulderHoldCommand;
import frc.robot.commands.shoulder.ShoulderMoveManual;
import frc.robot.commands.vision.RotateAlign;
import frc.robot.commands.vision.StrafeAlign;
// import frc.robot.commands.vision.DriveToTarget;
// import frc.robot.commands.vision.RotateAlign;
// import frc.robot.commands.vision.StrafeAlign;
import frc.robot.commands.vision.TranslateAlign;
import frc.robot.commands.wrist.WristGoToPositionCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.simulation.Simulator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandJoystick left_controller = new CommandJoystick(0);
  private final CommandJoystick right_controller = new CommandJoystick(1);
  private final CommandXboxController operatorController = new CommandXboxController(2);

  // The robot's subsystems and commands are defined here...
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem(() -> operatorController.getRightY()*Constants.SHOULDER_MANUAL_OVERRIDE_RANGE, operatorController.leftBumper());
  private final ShoulderSubsystem.ShoulderWristMessenger shoulderMessenger = shoulderSubsystem.new ShoulderWristMessenger();

  public final WristSubsystem wristSubsystem = new WristSubsystem(shoulderMessenger, () -> operatorController.getLeftY()*Constants.WRIST_MANUAL_OVERRIDE_RANGE, operatorController.leftBumper());

  public final ArmSubsystem armSubsystem = new ArmSubsystem(/*() -> operatorController.getRightTriggerAxis()*Constants.ARM_MANUAL_OFFSET_RANGE, operatorController.leftBumper()*/);
  private final ArmSubsystem.ArmShoulderMessenger armMessenger = armSubsystem.new ArmShoulderMessenger(); 

  private final Vision vision = new Vision(); 

  ////private final TranslateAlign align = new TranslateAlign(m_drivetrainSubsystem, vision);
  
  //private final RotateAlign rotAlign = new RotateAlign(m_drivetrainSubsystem, vision); 

  //private final DriveToTarget driveToTarget = new DriveToTarget(m_drivetrainSubsystem, vision);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // private final ManualIntakeCommand ManualIntakeCommand = new ManualIntakeCommand(intake, 5);
  // private final ManualPutdownCommand ManualPutdownCommand = new ManualPutdownCommand(intake, 5);

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>(); 

  // private final DriveStraightAuto driveStraightAuto = new DriveStraightAuto(m_drivetrainSubsystem, wristSubsystem); 
  // private final EngageStationAuto engageStationAuto = new EngageStationAuto(m_drivetrainSubsystem); 
  //private final Simulator sim = new Simulator(m_drivetrainSubsystem); 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> modifyAxis(left_controller.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(left_controller.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(right_controller.getX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
     ));

    shoulderSubsystem.setDefaultCommand(new ShoulderHoldCommand(shoulderSubsystem, armMessenger, () -> this.operatorController.getLeftTriggerAxis()));
    /*shoulderSubsystem.setDefaultCommand(new ShoulderMoveManual(shoulderSubsystem,
      () -> modifyAxis(operatorController.getLeftY()) ));*/
    wristSubsystem.setDefaultCommand(new WristHoldCommand(wristSubsystem, () -> this.operatorController.getLeftTriggerAxis()));
    //wristSubsystem.setDefaultCommand(new InstantCommand(() -> wristSubsystem.setWristSpeed(/*operatorController.getLeftX()*/0.1), wristSubsystem));
    armSubsystem.setDefaultCommand(new ArmHoldCommand(this.armSubsystem));
    intakeSubsystem.setDefaultCommand(new IntakeHoldCommand(this.intakeSubsystem));

    initializeRobot();
    // Configure the button bindings
    configureButtonBindings();
  }

  public void initializeRobot() { 
    // autoChooser.addOption("Tip cone & drive straight auto", driveStraightAuto);
    // autoChooser.addOption("Engage charge station auto", engageStationAuto);
    autoChooser.setDefaultOption("No auto", new WaitCommand(15));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    /*new Trigger(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
            .onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope));*/
    
    // left_controller.button(1).onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope)); 
    // operatorController.back().onTrue(new InstantCommand(armSubsystem::resetEncoder));

    //PAST WORK!
    operatorController.x().onTrue(new AssemblyHomePositionCommand(shoulderSubsystem, shoulderMessenger, wristSubsystem, armSubsystem, armMessenger)); 
    //  operatorController.b().onTrue(new ArmGoToPositionCommand(armSubsystem, messenger, 10.0));
    //  operatorController.a().onTrue(new ArmGoToPositionCommand(armSubsystem, messenger, 0.0));
    // operatorController.y().onTrue(new ArmGoToPositionCommand(armSubsystem, messenger, 15.0));
    // operatorController.povUp().onTrue(new ShoulderGoToPositionCommand(shoulderSubsystem, 0.0)); 
    // operatorController.povRight().onTrue(new ShoulderGoToPositionCommand(shoulderSubsystem, -45.0)); 
    // operatorController.povDown().onTrue(new ShoulderGoToPositionCommand(shoulderSubsystem, -90.0)); 
    // operatorController.povDown().onTrue(new WristGoToPositionCommand(wristSubsystem, 0.0));
    // operatorController.povUp().onTrue(new WristGoToPositionCommand(wristSubsystem, 90));
    // operatorController.povLeft().onTrue(new WristGoToPositionCommand(wristSubsystem, -45.0));
    // operatorController.povRight().onTrue(new WristGoToPositionCommand(wristSubsystem, 45.0));

    operatorController.a().onTrue((new AssemblyGoToCubeIntakeCommand(shoulderSubsystem, shoulderMessenger, wristSubsystem, armSubsystem, armMessenger)));
     operatorController.y().onTrue(new AssemblyGoToConeIntakeCommand(shoulderSubsystem, shoulderMessenger, wristSubsystem, armSubsystem, armMessenger));
    operatorController.b().onTrue(new AssemblyMidScoreCommand(shoulderSubsystem, shoulderMessenger, wristSubsystem, armSubsystem, armMessenger)); 

    operatorController.povUp().onTrue(new AssemblyHighScoreCommand(shoulderSubsystem, shoulderMessenger, wristSubsystem, armSubsystem, armMessenger)); 
     


    new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.05)
     .whileTrue(new ManualIntakeCommand(intakeSubsystem, () -> operatorController.getLeftTriggerAxis()));
    new Trigger(() -> operatorController.getRightTriggerAxis() > 0.05)
     .whileTrue(new ManualPutdownCommand(intakeSubsystem, () -> operatorController.getRightTriggerAxis())); 
    //operatorController.b().onTrue(new ArmGoToPositionCommand(armSubsystem, shoulderSubsystem, Constants.ARM_POSITION.MID_GOAL));
    // operatorController.x().onTrue(new ArmGoToPositionCommand(armSubsystem, shoulderSubsystem, Constants.ARM_POSITION.LOW_GOAL));

    // operatorController.y().onTrue(getRetractArmCommand()); 

    //operatorController.a().onTrue(new InstantCommand(shoulderSubsystem::resetMotorRotations));

    //operatorController.rightBumper().onTrue(new ShoulderGoToPositionCommand(shoulderSubsystem, -90.0));
    // // operatorController.y().onTrue(new ShoulderGoToPositionCommand(shoulderSubsystem, 90.0));
    //operatorController.leftBumper().onTrue(new ShoulderGoToPositionCommand(shoulderSubsystem, -45.0));
    /*operatorController.y().onTrue(new AssemblyGoToPositionCommand(shoulderSubsystem, wristSubsystem, 90.0));
    operatorController.b().onTrue(new
     AssemblyGoToPositionCommand(shoulderSubsystem, wristSubsystem, 150.0));
    operatorController.rightBumper().onTrue(new AssemblyGoToPositionCommand(shoulderSubsystem, wristSubsystem, -50.0));*/


    // // operatorController.povRight().onTrue(new WristGoToPositionCommand(wristSubsystem, 135));
   
    // operatorController.povUp().onTrue(new WristGoToPositionCommand(wristSubsystem, 90.0)); 
    
    /* 
    new Trigger(operatorController::getBackButton)
            .onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope));

    /* 
    new Trigger(m_controller::getAButton)
            .whileTrue(new StrafeAlign(
              m_drivetrainSubsystem, 
              vision, 
              () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 4, 
              () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 4)); // limit speed (for now)

    new Trigger(() -> m_controller.getPOV() == 0)
            .whileTrue(new TranslateAlign(
              m_drivetrainSubsystem,
              vision,
              Vision.Pipeline.CENTER_TARGET
            ));

    new Trigger(() -> m_controller.getPOV() == 90)
            .whileTrue(new TranslateAlign(
              m_drivetrainSubsystem,
              vision,
              Vision.Pipeline.RIGHT_TARGET
            ));

    new Trigger(() -> m_controller.getPOV() == 270)
            .whileTrue(new TranslateAlign(
              m_drivetrainSubsystem,
              vision,
              Vision.Pipeline.LEFT_TARGET
            ));
            */

    left_controller.button(2).whileTrue(new StrafeAlign(m_drivetrainSubsystem, vision, left_controller::getX, left_controller::getY));
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
    // return new ArmTestTuningCommand(armSubsystem);
    //return autoChooser.getSelected();  
  }

  

  public Command getArmHomeCommand() { 
    return new ArmHomeCommand(armSubsystem); 
  }

  public Command getShoulderZeroCommand() { 
    return new ShoulderGoToPositionCommand(shoulderSubsystem, Constants.SHOULDER_HOME_ANGLE); 
  }

  public Command getGoToZeroWristCommand() { 
    return new WristGoToPositionCommand(wristSubsystem, Constants.WRIST_HOME_ANGLE); 
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
