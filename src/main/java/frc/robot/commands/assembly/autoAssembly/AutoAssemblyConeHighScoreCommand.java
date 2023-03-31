package frc.robot.commands.assembly.autoAssembly;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmGoToPositionCommand;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.intake.AutoPutDownCommand;
import frc.robot.commands.intake.IntakeHoldCommand;
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

public class AutoAssemblyConeHighScoreCommand extends SequentialCommandGroup {
    
    public AutoAssemblyConeHighScoreCommand(ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                                WristSubsystem wrist, ArmSubsystem arm, IntakeSubsystem intake, 
                                                    ArmShoulderMessenger armMessenger) { 
        addCommands(
            new InstantCommand( () -> shoulder.setInIntakePosition(false)),

            new ShoulderGoToPositionCommand(shoulder, Constants.CONE_SCORE_HIGH_POSITION_SHOULDER)
                .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                .raceWith(new ArmHoldCommand(arm))
                .raceWith(new IntakeHoldCommand(intake)),

            new WristGoToPositionCommand(wrist, Constants.CONE_SCORE_HIGH_POSITION_WRIST)
                .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                .raceWith(new ArmHoldCommand(arm))
                .raceWith(new IntakeHoldCommand(intake)), 
    
            new ArmGoToPositionCommand(arm, shoulderWristMessenger, Constants.CONE_SCORE_HIGH_POSITION_ARM)
                .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                .raceWith(new IntakeHoldCommand(intake)), 

            new AutoPutDownCommand(intake, 0.8)
                .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                .raceWith(new ArmHoldCommand(arm))
        ); 
            
    
    }
}
