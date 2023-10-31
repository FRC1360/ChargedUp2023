package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.autos.basic.Drive;
import frc.robot.autos.procedures.ConeScoreMidAuto;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.shoulder.ShoulderHoldCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.StateMachine;

public class ConeMidAndDriveAuto extends SequentialCommandGroup {
    
    public ConeMidAndDriveAuto(DrivetrainSubsystem dt, ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                WristSubsystem wrist, ArmSubsystem arm, IntakeSubsystem intake, 
                                ArmShoulderMessenger armMessenger,
                                LEDSubsystem ledSubsystem,
                                StateMachine sm) { 
            
            addCommands(new ConeScoreMidAuto(dt, shoulder, shoulderWristMessenger, wrist, arm, intake, armMessenger, ledSubsystem, sm), 
                    new Drive(dt, Constants.DRIVE_STRAIGHT_DISTANCE, 0.0)
                        .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                        .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                        .raceWith(new ArmHoldCommand(arm)));
        }
}
