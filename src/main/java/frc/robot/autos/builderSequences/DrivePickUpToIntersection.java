package frc.robot.autos.builderSequences;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.Direction;
import frc.robot.autos.basic.Drive;
import frc.robot.autos.basic.Rotate;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivePickUpToIntersection extends SequentialCommandGroup {
    
    public DrivePickUpToIntersection(DrivetrainSubsystem dt, Direction direction) { 

        if (direction == Direction.LEFT) addCommands(new Drive(dt, -1.5, 2.0, 0.0, 2.0)
                                                        ); 

        else if (direction == Direction.BACK) addCommands(new Drive(dt, -1.7, 0.0, 0.0, 2.0));
        
        else if (direction == Direction.RIGHT) addCommands(new Drive(dt, -1.5, 0.0, 0.0, 2.0)
                                                                .alongWith(new Rotate(dt, Rotation2d.fromDegrees(0), -0.01, -0.015, 6.2, 1.9, 45)));
    }
}
