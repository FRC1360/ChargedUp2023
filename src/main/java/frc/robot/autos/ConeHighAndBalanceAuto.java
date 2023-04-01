package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.DriveEncoder;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.shoulder.ShoulderHoldCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;
import frc.robot.subsystems.WristSubsystem;

public class ConeHighAndBalanceAuto extends SequentialCommandGroup {
    
    public ConeHighAndBalanceAuto(DrivetrainSubsystem dt, ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                    WristSubsystem wrist, ArmSubsystem arm, IntakeSubsystem intake, 
                                        ArmShoulderMessenger armMessenger) { 
        
        addCommands(//new ConeScoreHighAuto(shoulder, shoulderWristMessenger, wrist, arm, intake, armMessenger), 
                    new DriveEncoder(dt, 3.5, 0.0)
                        .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                        .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                        .raceWith(new ArmHoldCommand(arm))
                    // new AutoBalanceCommand(dt)
                    //     .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                    //     .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                    //     .raceWith(new ArmHoldCommand(arm))
                    ); 
    }
}
