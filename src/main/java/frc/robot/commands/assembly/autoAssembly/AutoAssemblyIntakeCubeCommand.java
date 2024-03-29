package frc.robot.commands.assembly.autoAssembly;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmGoToPositionCommand;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.intake.AutoIntakeCommand;
import frc.robot.commands.shoulder.ShoulderGoToPositionCommand;
import frc.robot.commands.shoulder.ShoulderHoldCommand;
import frc.robot.commands.wrist.WristGoToPositionCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;

public class AutoAssemblyIntakeCubeCommand extends SequentialCommandGroup {
    
    public AutoAssemblyIntakeCubeCommand(ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                                WristSubsystem wrist, ArmSubsystem arm, IntakeSubsystem intake, ArmShoulderMessenger armMessenger) { 
        
        addCommands(
                new InstantCommand( () -> shoulder.setInIntakePosition(false)),
                
                new ShoulderGoToPositionCommand(shoulder, Constants.CUBE_INTAKE_POSITION_SHOULDER)
                        .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                        .raceWith(new ArmHoldCommand(arm)), 
                                            
                new ArmGoToPositionCommand(arm, shoulderWristMessenger, Constants.CUBE_INTAKE_POSITION_ARM)
                        .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                        .raceWith(new WristHoldCommand(wrist, () -> 0.0)), 
                                            
                new WristGoToPositionCommand(wrist, Constants.CUBE_INTAKE_POSITION_WRIST)
                        .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                        .raceWith(new ArmHoldCommand(arm)),
                
                new InstantCommand(() -> shoulder.setInIntakePosition(true)), 

                new AutoIntakeCommand(intake, 0.15)
                    .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.15))
                    .raceWith(new WristHoldCommand(wrist, () -> 0.15))
                    .raceWith(new ArmHoldCommand(arm))
                
        );
    }
}

