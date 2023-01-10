package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.Vision;

public class Align extends CommandBase {
    
    private Vision vision; 

    public Align(Vision vision) { 
        this.vision = vision; 
        addRequirements(vision); 
    }

    @Override
    public void execute() { 
        if (vision.hasTargets()) 
        {
        double xOffset = vision.getX(); 
        double yOffset = vision.getY(); 
        double distance = vision.getDistanceFromTarget();

        SmartDashboard.putNumber("X-Offset:", xOffset); 
        SmartDashboard.putNumber("Y-Offset:", yOffset);
        SmartDashboard.putNumber("Distance:", distance);
        }
        SmartDashboard.putBoolean("Running", true); 
    }

}
