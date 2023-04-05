package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.assembly.AssemblyHomePositionCommand;
import frc.robot.commands.assembly.autoAssembly.AutoAssemblyConeHighScoreCommand;
import frc.robot.commands.shoulder.ShoulderHoldCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;
import frc.robot.subsystems.WristSubsystem;

public class ConeScoreHighAuto extends SequentialCommandGroup {
    
    public ConeScoreHighAuto(ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                WristSubsystem wrist, ArmSubsystem arm, IntakeSubsystem intake, 
                                    ArmShoulderMessenger armMessenger, LEDSubsystem ledSubsystem) 
    {
        addCommands( 
            new AutoAssemblyConeHighScoreCommand(shoulder, shoulderWristMessenger, wrist, 
                                                    arm, intake, ledSubsystem, armMessenger), 
                // .raceWith(new DriveSpeed(dt, -0.5, 0.0)),   // New - To be tested!!!!
            new AssemblyHomePositionCommand(shoulder, shoulderWristMessenger, 
                            wrist, arm, armMessenger, ledSubsystem)
            );
    }
}
