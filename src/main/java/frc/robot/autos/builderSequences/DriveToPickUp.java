package frc.robot.autos.builderSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.Direction;
import frc.robot.autos.basic.Drive;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToPickUp extends SequentialCommandGroup {
    
    public DriveToPickUp(DrivetrainSubsystem dt, Direction position) {
        addCommands(
            new DriveStraightIntersection(dt, Direction.FRONT_CENTER), 
            new DriveIntersectionToPickUp(dt, position)
        );
    }
}
