package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotateShoulderCommand;
import frc.robot.subsystems.ShoulderSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoSequence extends SequentialCommandGroup {

    public AutoSequence(ShoulderSubsystem subsystem) { 
        addCommands(new RotateShoulderCommand(subsystem, 90));
        //addCommands(new Drive(dt, 2.0, 2.0), new Rotate(dt, Rotation2d.fromDegrees(45)));
    }   
}