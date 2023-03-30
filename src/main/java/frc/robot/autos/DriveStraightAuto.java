package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class DriveStraightAuto extends SequentialCommandGroup {

    public DriveStraightAuto(DrivetrainSubsystem dt, WristSubsystem wrist) { 
        addCommands(new WaitCommand(3), 
                    new Drive(dt, 5.8, 0.0) //From nodes blue line to first cube is 5.69 m 
                    );
    }
    
}