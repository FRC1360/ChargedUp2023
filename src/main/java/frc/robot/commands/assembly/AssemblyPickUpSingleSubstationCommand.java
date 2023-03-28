package frc.robot.commands.assembly;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.shoulder.ShoulderGoToPositionCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AssemblyPickUpSingleSubstationCommand extends SequentialCommandGroup {
    
    public AssemblyPickUpSingleSubstationCommand(ShoulderSubsystem shoulder, WristSubsystem wrist, ArmSubsystem arm) { 
        addCommands(
            new InstantCommand( () -> shoulder.setInIntakePosition(false)),

            new ShoulderGoToPositionCommand(shoulder, -48.0)
                .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                .raceWith(new ArmHoldCommand(arm))
        );
    }
}
