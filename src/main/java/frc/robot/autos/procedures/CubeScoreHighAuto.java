package frc.robot.autos.procedures;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.LockWheels;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.assembly.AssemblyHighScoreCommand;
import frc.robot.commands.assembly.AssemblyHomePositionCommand;
import frc.robot.commands.intake.AutoIntakeCommand;
import frc.robot.commands.intake.AutoPutDownCommand;
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

public class CubeScoreHighAuto extends SequentialCommandGroup {
    
    public CubeScoreHighAuto(DrivetrainSubsystem dt, ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                WristSubsystem wrist, ArmSubsystem arm, IntakeSubsystem intake, 
                                    ArmShoulderMessenger armMessenger, LEDSubsystem ledSubsystem) { 
            
            addCommands(

                new AutoIntakeCommand(intake, 0.25), 

                new AssemblyHighScoreCommand(shoulder, shoulderWristMessenger, wrist, arm, armMessenger, () -> true, ledSubsystem)
                    .raceWith(new LockWheels(dt, 0.0, 0.0005)), 

                new AutoPutDownCommand(intake, 0.8)
                    .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                    .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                    .raceWith(new ArmHoldCommand(arm)), 

                new AssemblyHomePositionCommand(shoulder, shoulderWristMessenger, wrist, arm, armMessenger, ledSubsystem)
            );
    }
}
