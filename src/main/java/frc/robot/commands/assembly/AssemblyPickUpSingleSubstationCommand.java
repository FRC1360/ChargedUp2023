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
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;

public class AssemblyPickUpSingleSubstationCommand extends SequentialCommandGroup {
    
    public AssemblyPickUpSingleSubstationCommand(ShoulderSubsystem shoulder, WristSubsystem wrist, ArmSubsystem arm,
        ShoulderWristMessenger shoulderWristMessenger, ArmShoulderMessenger armMessenger) { 

        addCommands(
            new InstantCommand( () -> shoulder.setInIntakePosition(false)),

            new ArmGoToPositionCommand(arm, shoulderWristMessenger, Constants.SINGLE_SUBSTATION_POSITION_ARM)
                .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                .raceWith(new WristHoldCommand(wrist, () -> 0.0)), 

            new ShoulderGoToPositionCommand(shoulder, Constants.SINGLE_SUBSTATION_POSITION_SHOULDER)
                .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                .raceWith(new ArmHoldCommand(arm)), 
            
            new WristGoToPositionCommand(wrist, Constants.SINGLE_SUBSTATION_POSITION_WRIST)
            .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
            .raceWith(new ArmHoldCommand(arm))
        );
    }
}
