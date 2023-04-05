package frc.robot.commands.assembly.autoAssembly;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmGoToPositionCommand;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.assembly.AssemblyHighScoreCommand;
import frc.robot.commands.intake.AutoPutDownCommand;
import frc.robot.commands.intake.IntakeHoldCommand;
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

public class AutoAssemblyConeHighScoreCommand extends SequentialCommandGroup {
    
    public AutoAssemblyConeHighScoreCommand(ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                                WristSubsystem wrist, ArmSubsystem arm, IntakeSubsystem intake,
                                                    LEDSubsystem ledSubsystem, ArmShoulderMessenger armMessenger) { 
        addCommands(
            new AssemblyHighScoreCommand(shoulder, shoulderWristMessenger, wrist, arm, armMessenger, () -> false, ledSubsystem), 

            new AutoPutDownCommand(intake, 0.8)
                .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                .raceWith(new ArmHoldCommand(arm))
        ); 
            
    
    }
}
