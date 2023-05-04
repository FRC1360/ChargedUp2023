package frc.robot.commands.assembly;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;
import frc.robot.util.StateMachine;

public class AssemblyHighScoreCommand extends SequentialCommandGroup {
    
    public AssemblyHighScoreCommand(ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                                WristSubsystem wrist, ArmSubsystem arm, ArmShoulderMessenger armMessenger,
                                                BooleanSupplier scoreCube,
                                                LEDSubsystem ledSubsystem,
                                                StateMachine sm) { 
        addCommands(
            new InstantCommand( () -> sm.setAtHome(false)),
            new InstantCommand( () -> shoulder.setInIntakePosition(false)),
            new InstantCommand(ledSubsystem::setLEDDisable),
            
            new ConditionalCommand(
                // Score Cube
                (new ArmGoToPositionCommand(arm, shoulderWristMessenger, 0.0)
                        .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                        .raceWith(new WristHoldCommand(wrist, () -> 0.0)))
                    .andThen(new ShoulderGoToPositionCommand(shoulder, Constants.CUBE_SCORE_HIGH_POSITION_SHOULDER)
                        .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                        .raceWith(new ArmHoldCommand(arm)))
                    .andThen((new WristGoToPositionCommand(wrist, Constants.CUBE_SCORE_HIGH_POSITION_WRIST)
                        .alongWith(new ArmGoToPositionCommand(arm, shoulderWristMessenger, Constants.CUBE_SCORE_HIGH_POSITION_ARM)))
                        .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))),
                
                //Score Cone
                
                (new ShoulderGoToPositionCommand(shoulder, Constants.CONE_SCORE_HIGH_POSITION_SHOULDER)
                    .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                    .raceWith(new ArmHoldCommand(arm)))
                .andThen((new WristGoToPositionCommand(wrist, Constants.CONE_SCORE_HIGH_POSITION_WRIST)
                    .alongWith(new ArmGoToPositionCommand(arm, shoulderWristMessenger, Constants.CONE_SCORE_HIGH_POSITION_ARM)))
                    .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))),
                
                scoreCube),

                new InstantCommand(ledSubsystem::setLEDEnable)

            
        ); 
            
    
    }
}