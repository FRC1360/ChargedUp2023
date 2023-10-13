// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.intake.IntakeHoldCommand;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();hy8ogukjvdfknjleaioptg;uilh vckj,/ed'afopgiu chdslkmwl;'afgvyu jgcbaxknjopdgti9uojvknf ,m'
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    CommandScheduler.getInstance().run();

    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.wristSubsystem.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.shoulderSubsystem.updateSmartDashboard();
    m_robotContainer.wristSubsystem.updateSmartDashboard();
    m_robotContainer.armSubsystem.updateSmartDashboard();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.wristSubsystem.setIdleMode(IdleMode.kCoast);
    m_robotContainer.shoulderSubsystem.resetMotorRotations();
    m_robotContainer.wristSubsystem.resetMotorRotations();
    
    /*m_robotContainer.getArmHomeCommand().schedule(); 
    m_robotContainer.getGoToZeroWristCommand().schedule(); 
    m_robotContainer.getShoulderZeroCommand().schedule();*/

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      (((m_robotContainer.getArmHomeCommand()
      .andThen(m_robotContainer.getGoToZeroWristCommand())
      .andThen(m_robotContainer.getShoulderZeroCommand())))
        .raceWith(m_robotContainer.getIntakeHoldCommand()))
      .andThen(m_robotContainer.setSMHomeCommand())
      .andThen(m_autonomousCommand).schedule();
    } else {
      (m_robotContainer.getArmHomeCommand()
      .andThen(m_robotContainer.getGoToZeroWristCommand())
      .andThen(m_robotContainer.getShoulderZeroCommand()))
      .andThen(m_robotContainer.setSMHomeCommand()).schedule(); 
    }

   // m_robotContainer.getArmHomeCommand().schedule(); 
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // m_robotContainer.shoulderSubsystem.updateSmartDashboard();
    // m_robotContainer.wristSubsystem.updateSmartDashboard();
    // m_robotContainer.armSubsystem.updateSmartDashboard();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

      m_robotContainer.wristSubsystem.setIdleMode(IdleMode.kCoast);
      m_robotContainer.shoulderSubsystem.resetMotorRotations();
      m_robotContainer.wristSubsystem.resetMotorRotations();
      m_robotContainer.wristSubsystem.holdPIDController.reset();
    } else {
      m_robotContainer.wristSubsystem.setIdleMode(IdleMode.kCoast);
      m_robotContainer.shoulderSubsystem.resetMotorRotations();
      m_robotContainer.wristSubsystem.resetMotorRotations();
      m_robotContainer.wristSubsystem.holdPIDController.reset();

      (m_robotContainer.getArmHomeCommand()
        .andThen(m_robotContainer.getGoToZeroWristCommand())
        .andThen(m_robotContainer.getShoulderZeroCommand())
        .andThen(m_robotContainer.setSMHomeCommand())).schedule(); 
    }

    // For Tuning
    /*m_robotContainer.wristSubsystem.setIdleMode(IdleMode.kCoast);
    m_robotContainer.shoulderSubsystem.resetMotorRotations();
    m_robotContainer.wristSubsystem.resetMotorRotations();
    m_robotContainer.wristSubsystem.holdPIDController.reset();
    m_robotContainer.getGoToZeroWristCommand().schedule();*/

    
    // m_robotContainer.shoulderSubsystem.holdPIDController.reset();
    
    /*m_robotContainer.getGoToZeroWristCommand().schedule(); 
    m_robotContainer.getShoulderZeroCommand().schedule();*/
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.shoulderSubsystem.updateSmartDashboard();
    m_robotContainer.wristSubsystem.updateSmartDashboard();
    m_robotContainer.armSubsystem.updateSmartDashboard();

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    
  }
}
