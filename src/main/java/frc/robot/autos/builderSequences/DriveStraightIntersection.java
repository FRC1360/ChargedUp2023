package frc.robot.autos.builderSequences;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.Direction;
import frc.robot.autos.basic.Drive;
import frc.robot.autos.basic.DriveToPosition;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveStraightIntersection extends SequentialCommandGroup {
    
    public DriveStraightIntersection(DrivetrainSubsystem dt, Direction direction) { 
        if (direction == Direction.FRONT_CENTER) addCommands(new Drive(dt, 6.7, 0.0, 0.0, 2.0)
                                                                //new DriveToPosition(dt, new Translation2d(8.4, 1.5))
                                                            );

        else if (direction == Direction.BACK) addCommands(new Drive(dt, -6.7, 0.0, 2.0, 0.0)
        
                                                            );

    }
    
}
