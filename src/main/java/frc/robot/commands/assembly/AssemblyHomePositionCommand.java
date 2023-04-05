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
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;

public class AssemblyHomePositionCommand extends SequentialCommandGroup {
    
    public AssemblyHomePositionCommand(ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                        WristSubsystem wrist, ArmSubsystem arm, ArmShoulderMessenger armMessenger, LEDSubsystem ledSubsystem) { 
        addCommands(
            new InstantCommand( () -> shoulder.setInIntakePosition(false)),
            new InstantCommand(ledSubsystem::setLEDDisable),
            
            (new ArmGoToPositionCommand(arm, shoulderWristMessenger, Constants.HOME_POSITION_ARM)
                .alongWith(new WristGoToPositionCommand(wrist, Constants.HOME_POSITION_WRIST - 20.0)))
                .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0)),
            new WristGoToPositionCommand(wrist, Constants.HOME_POSITION_WRIST)
                .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                .raceWith(new ArmHoldCommand(arm)),
            new ShoulderGoToPositionCommand(shoulder, Constants.HOME_POSITION_SHOULDER)
                .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                .raceWith(new ArmHoldCommand(arm)),

            new InstantCommand(ledSubsystem::setLEDEnable)
            );
        

    }
}
