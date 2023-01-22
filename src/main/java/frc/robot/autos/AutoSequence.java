package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoSequence extends SequentialCommandGroup {

    public AutoSequence(DrivetrainSubsystem dt) { 
        // Note: Rotation doesn't work in the simulator! But something to think out in real world
        
        addCommands(new Drive(dt, 2.0, 2.0), 
                    /*new Rotate(dt, Rotation2d.fromDegrees(45)), */
                    new Drive(dt, -2.0, 2.0)
                    );
    }
    
}
