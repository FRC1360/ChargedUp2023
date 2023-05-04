package frc.robot.commands.assembly;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmGoToPositionCommand;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.shoulder.ShoulderGoToPositionCommand;
import frc.robot.commands.shoulder.ShoulderHoldCommand;
import frc.robot.commands.wrist.WristGoToPositionCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;
import frc.robot.util.StateMachine;

public class AssemblyGoToCubeIntakeCommand extends SequentialCommandGroup {
    
    public AssemblyGoToCubeIntakeCommand(ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                                WristSubsystem wrist, ArmSubsystem arm, ArmShoulderMessenger armMessenger, 
                                                IntakeSubsystem intake,
                                                LEDSubsystem ledSubsystem,
                                                StateMachine sm) { 
        
        addCommands(
                new InstantCommand( () -> sm.setAtHome(false)),
                new InstantCommand( () -> shoulder.setInIntakePosition(false)),
                new InstantCommand( () -> intake.setAtSubstationState(false)), 
                new InstantCommand(ledSubsystem::setLEDDisable),
                
                new ShoulderGoToPositionCommand(shoulder, Constants.CUBE_INTAKE_POSITION_SHOULDER)
                        .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                        .raceWith(new ArmHoldCommand(arm)), 
                                            
                (new ArmGoToPositionCommand(arm, shoulderWristMessenger, Constants.CUBE_INTAKE_POSITION_ARM)
                        .alongWith(new WristGoToPositionCommand(wrist, Constants.CUBE_INTAKE_POSITION_WRIST)))
                        .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0)),
                
                new InstantCommand(() -> shoulder.setInIntakePosition(true)),
                new InstantCommand(ledSubsystem::setLEDEnable)
                
        );
    }
}
