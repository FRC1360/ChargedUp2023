package frc.robot.autos.builderSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.Direction;
import frc.robot.autos.basic.Drive;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveBackFromPickUp extends SequentialCommandGroup {
    
    public DriveBackFromPickUp(DrivetrainSubsystem dt, Direction position) {
        addCommands(
            new DrivePickUpToIntersection(dt, position), 
            new DriveStraightIntersection(dt, Direction.BACK)
        );
    }
}

