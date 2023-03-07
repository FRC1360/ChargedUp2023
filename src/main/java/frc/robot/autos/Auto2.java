package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.Direction;
import frc.robot.autos.basic.Drive;
import frc.robot.autos.builderSequences.CenterToScore;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Auto2 extends SequentialCommandGroup {
    
    public Auto2(DrivetrainSubsystem dt) { 
        // Note: Rotation doesn't work in the simulator! But something to think out in real world
        // Relative to starting position
        addCommands(new Drive(dt, 8.0, 0.0), 
                    new Drive(dt, -8.0, 0.0), 
                    new CenterToScore(dt, Direction.RIGHT)
            
        /*new CenterToScore(dt, Direction.RIGHT),

                    new Drive(dt, 4.3, 0.0), 
                    new Drive(dt, -4.3, 0.0), 

                    new Drive(dt, 0.0, -0.2), 
                    new Drive(dt, -0.5, 0.0),

                    new Drive(dt, 3.3, 0.0), 
                    new Drive(dt, 1.3, -1.3), 
                    new Drive(dt, -1.3, 1.3), 
                    new Drive(dt, -2.7, 0.0), 
                    new Drive(dt, 0.0, -0.7)

                    */

                    // new Drive(dt, -0.5, 0.0), 
                    // new Drive(dt, 0.5, 0.0),

                    // new Drive(dt, 0.0, -1.0), 
                    // new Drive(dt, 1.5, 0.0)
                    

                    /* 
                    new Drive(dt, 0.0, -3.3, new Translation2d(2.5, 4.7)), 
                    new Drive(dt, 4.3, 0.0, new Translation2d(2.5, 1.4)), 
                    new Drive(dt, -4.8, 0.0, new Translation2d(6.8, 1.4)),
                    new Drive(dt, 0.5, 0.0, new Translation2d(2.0, 1.4)),
                    new Drive(dt, 0.0, 1.6, new Translation2d(2.5, 1.4)), 
                    new Drive(dt, 1.4, 0.0, new Translation2d(2.5, 3.0))
                    */
                    );
    }
}
