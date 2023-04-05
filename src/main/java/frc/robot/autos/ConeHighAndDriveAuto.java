package frc.robot.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.Drive;
import frc.robot.autos.basic.DriveSpeed;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.assembly.AssemblyHomePositionCommand;
import frc.robot.commands.assembly.autoAssembly.AutoAssemblyConeHighScoreCommand;
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

public class ConeHighAndDriveAuto extends SequentialCommandGroup {
    
    public ConeHighAndDriveAuto(DrivetrainSubsystem dt, ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                    WristSubsystem wrist, ArmSubsystem arm, IntakeSubsystem intake, 
                                        ArmShoulderMessenger armMessenger,
                                        LEDSubsystem ledSubsystem) { 
        
        addCommands(new ConeScoreHighAuto(shoulder, shoulderWristMessenger, wrist, arm, intake, armMessenger, ledSubsystem), 
                    new Drive(dt, 8.0, 0.0)
                        .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                        .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                        .raceWith(new ArmHoldCommand(arm))
                        ); 
    }
}
