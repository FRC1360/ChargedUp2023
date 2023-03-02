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
                new Drive(dt, -0.3, -0.3), // Slide left
                new Drive(dt, -0.1, 0.0)  // Slide in to node frame to score
            );
        } 
        else if (direction == Direction.RIGHT) { 
            addCommands(
                new Drive(dt, -0.3, 0.3), 
                new Drive(dt, -0.1, 0.0) // Slide in
            );
        }
        else if (direction == Direction.CENTER) { 
            addCommands(
                new Drive(dt, -0.3, 0.0), 
                new Drive(dt, -0.1, 0.0) 
            );
        }
    }
}
