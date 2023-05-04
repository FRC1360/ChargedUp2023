package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.Drive;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveStraightAuto extends SequentialCommandGroup {
    
    public DriveStraightAuto(DrivetrainSubsystem dt) { 

        addCommands(
            new Drive(dt, 8.0, 0.0)
        );
    }
}
