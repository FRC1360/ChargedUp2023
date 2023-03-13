package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.Direction;
import frc.robot.autos.builderSequences.CenterToScore;
import frc.robot.autos.procedures.GetAndScoreCone;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LeftSide2ConeAuto extends SequentialCommandGroup {
    
    public LeftSide2ConeAuto(DrivetrainSubsystem dt) { 
        // Note: Rotation doesn't work in the simulator! But something to think out in real world
        // Relative to starting position
        addCommands(/* Command to score preload */
                    new GetAndScoreCone(dt, Direction.FRONT_CENTER, Direction.LEFT), 
                    new CenterToScore(dt, Direction.RIGHT), // To reset
                    new GetAndScoreCone(dt, Direction.RIGHT, Direction.RIGHT)
                    );
    }
}
