package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.Direction;
import frc.robot.autos.basic.Drive;
import frc.robot.autos.basic.Rotate; 
import frc.robot.autos.procedures.GetAndScoreCone;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RightSide2ConeAuto extends SequentialCommandGroup {

    public RightSide2ConeAuto(DrivetrainSubsystem dt) { 
        // Note: Rotation doesn't work in the simulator! But something to think out in real world
        // Relative to starting position
        
        addCommands(/* Command to score preload */
                    // new GetAndScoreCone(dt, Direction.FRONT_CENTER, Direction.RIGHT), 
                    // new CenterToScore(dt, Direction.LEFT), 
                    // new GetAndScoreCone(dt, Direction.LEFT, Direction.LEFT)
                    new Drive(dt, 7.0, 0.0)
                    //new Drive(dt, -7.0, 0.0), 
                    //new Drive(dt, 0.0, -1.0)
                    );
                    
    }
}