package frc.robot.autos.procedures;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.Direction;
import frc.robot.autos.basic.Drive;
import frc.robot.autos.builderSequences.CenterToScore;
import frc.robot.autos.builderSequences.DriveBackFromPickUp;
import frc.robot.autos.builderSequences.DriveToPickUp;
import frc.robot.subsystems.DrivetrainSubsystem;

public class GetAndScoreCone extends SequentialCommandGroup {
    
    public GetAndScoreCone(DrivetrainSubsystem dt, Direction gameObjectPosition, Direction scorePosition) { 
        addCommands(new DriveToPickUp(dt, gameObjectPosition), 
                    /* Command to pickup */
                    new DriveBackFromPickUp(dt, gameObjectPosition.getOpposite()), 
                    new CenterToScore(dt, scorePosition)
                    /* Command to score */
                    /* Command to lower (?)*/
                    );
    }
}
