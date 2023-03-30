package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class EngageStationAuto extends SequentialCommandGroup {
    
    public EngageStationAuto(DrivetrainSubsystem dt) { 
        addCommands(new Drive(dt, 4.5, 0));
    }
}
