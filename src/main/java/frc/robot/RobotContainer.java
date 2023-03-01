// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.assembly.AssemblyGoToPositionCommand;
import frc.robot.commands.shoulder.ShoulderGoToPositionCommand;
import frc.robot.commands.shoulder.ShoulderHoldCommand;
import frc.robot.commands.wrist.WristGoToPositionCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController m_controller = new XboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem(() -> operatorController.getRightY()*Constants.SHOULDER_MANUAL_OVERRIDE_RANGE, operatorController.leftBumper());
  //private final ShoulderSubsystem.ShoulderWristMessenger messenger = shoulderSubsystem.new ShoulderWristMessenger();

  //public final WristSubsystem wristSubsystem = new WristSubsystem(messenger, () -> operatorController.getLeftY()*Constants.WRIST_MANUAL_OVERRIDE_RANGE, operatorController.leftBumper());

  //public final ArmSubsystem armSubsystem = new ArmSubsystem();


  /*private final AutoSequence auto = new AutoSequence(m_drivetrainSubsystem); 


  private final Simulator sim = new Simulator(m_drivetrainSubsystem); */
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
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    shoulderSubsystem.setDefaultCommand(new ShoulderHoldCommand(shoulderSubsystem));
    //wristSubsystem.setDefaultCommand(new WristHoldCommand(wristSubsystem));

    //armSubsystem.setDefaultCommand(new ArmHoldCommand(this.armSubsystem));

    // Configure the button bindings
    configureButtonBindings();
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

    /*operatorController.a().onTrue(new ArmGoToPositionCommand(armSubsystem, ARM_POSITION.HIGH_GOAL));
    operatorController.b().onTrue(new ArmGoToPositionCommand(armSubsystem, ARM_POSITION.MID_GOAL));
    operatorController.x().onTrue(new ArmGoToPositionCommand(armSubsystem, ARM_POSITION.LOW_GOAL));*/

    //operatorController.a().onTrue(new InstantCommand(shoulderSubsystem::resetMotorRotations));

    operatorController.x().onTrue(new ShoulderGoToPositionCommand(shoulderSubsystem, 0.0));
    operatorController.y().onTrue(new ShoulderGoToPositionCommand(shoulderSubsystem, 90.0));
    operatorController.b().onTrue(new ShoulderGoToPositionCommand(shoulderSubsystem, 120.0));
    /*operatorController.y().onTrue(new AssemblyGoToPositionCommand(shoulderSubsystem, wristSubsystem, 90.0));
    operatorController.b().onTrue(new AssemblyGoToPositionCommand(shoulderSubsystem, wristSubsystem, 150.0));
    operatorController.rightBumper().onTrue(new AssemblyGoToPositionCommand(shoulderSubsystem, wristSubsystem, -50.0));

    operatorController.povDown().onTrue(new InstantCommand(wristSubsystem::resetMotorRotations));
    operatorController.povUp().onTrue(new WristGoToPositionCommand(wristSubsystem, 90));
    operatorController.povLeft().onTrue(new WristGoToPositionCommand(wristSubsystem, 45));
    operatorController.povRight().onTrue(new WristGoToPositionCommand(wristSubsystem, 135));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
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
