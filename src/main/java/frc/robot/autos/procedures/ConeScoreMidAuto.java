package frc.robot.autos.procedures;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.LockWheels;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.assembly.AssemblyHomePositionCommand;
import frc.robot.commands.assembly.AssemblyMidScoreCommand;
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
import frc.robot.util.StateMachine;

public class ConeScoreMidAuto extends SequentialCommandGroup {
    
    public ConeScoreMidAuto(DrivetrainSubsystem dt, ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                WristSubsystem wrist, ArmSubsystem arm, IntakeSubsystem intake, 
                                ArmShoulderMessenger armMessenger, LEDSubsystem ledSubsystem, StateMachine sm) 
        { 
            addCommands(
                new AssemblyMidScoreCommand(shoulder, shoulderWristMessenger, wrist, arm, armMessenger, ledSubsystem,
                () -> false,
                sm)
                    .raceWith(new LockWheels(dt)),

                new AutoPutDownCommand(intake, 0.8)
                    .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                    .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                    .raceWith(new ArmHoldCommand(arm)), 

                new AssemblyHomePositionCommand(shoulder, shoulderWristMessenger, wrist, arm, armMessenger, ledSubsystem, sm)
            );
        }
}
