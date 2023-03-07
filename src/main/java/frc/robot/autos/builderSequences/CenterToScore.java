package frc.robot.autos.builderSequences;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.Direction;
import frc.robot.autos.basic.Drive;
import frc.robot.subsystems.DrivetrainSubsystem;

public class CenterToScore extends SequentialCommandGroup {
    
    public CenterToScore(DrivetrainSubsystem dt, Direction direction) { 
        if (direction == Direction.LEFT) { 
            addCommands(
                new Drive(dt, 0.0, -1.0) // Slide left
            );
        } 
        else if (direction == Direction.RIGHT) { 
            addCommands(
                new Drive(dt, 0.0, 1.0)
            );
        }
    }
}