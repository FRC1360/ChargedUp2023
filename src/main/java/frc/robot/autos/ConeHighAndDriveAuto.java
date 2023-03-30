package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.Drive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;
import frc.robot.subsystems.WristSubsystem;

public class ConeHighAndDriveAuto extends SequentialCommandGroup {
    
    public ConeHighAndDriveAuto(DrivetrainSubsystem dt, ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                    WristSubsystem wrist, ArmSubsystem arm, IntakeSubsystem intake, 
                                        ArmShoulderMessenger armMessenger) { 
        
        addCommands(new ConeScoreHighAuto(shoulder, shoulderWristMessenger, wrist, arm, intake, armMessenger), 
                    new Drive(dt, 7.0, -0.1)); 
    }
}
