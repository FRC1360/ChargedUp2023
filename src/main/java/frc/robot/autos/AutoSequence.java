package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShoulderCommand;
import frc.robot.subsystems.ShoulderSubsystem;

public class AutoSequence extends SequentialCommandGroup {

    public AutoSequence(ShoulderSubsystem subsystem) { 
        addCommands(new ShoulderCommand(subsystem, 15));
    }   
}