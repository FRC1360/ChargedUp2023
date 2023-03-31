package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.vision.Vision;

// aligns crosshair to vision tape
public class TranslateAlign extends CommandBase {
    
    private Vision vision; 
    private DrivetrainSubsystem dt;

    public TranslateAlign(DrivetrainSubsystem dt, Vision vision) { 
        this.vision = vision; 
        this.dt = dt;

        addRequirements(dt, vision); 
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() { 
        SmartDashboard.putBoolean("Align Running", true); 

        if (vision.hasTargets()) 
        {
            double xOffset = vision.getX(); 
            double yOffset = vision.getY(); 
            // double distance = vision.getDistanceFromTarget();

            SmartDashboard.putNumber("X-Offset:", xOffset); 
            SmartDashboard.putNumber("Y-Offset:", yOffset);
            // SmartDashboard.putNumber("Distance:", distance);

            dt.drive(new ChassisSpeeds(yOffset/25, -xOffset/25, 0)); 
        }
    }

    @Override
    public void end(boolean interrupted) { 
        dt.stop();
    }

    @Override
    public boolean isFinished() { 
        return Math.abs(vision.getX()) < 1.5;  
    }

}
