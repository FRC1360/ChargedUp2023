package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.Vision;

public class RotateAlign extends CommandBase {
    
    private Vision vision; 
    private DrivetrainSubsystem dt; 
    private ChassisSpeeds speeds = new ChassisSpeeds(); 

    public RotateAlign(DrivetrainSubsystem dt, Vision vision) { 
        this.vision = vision; 
        this.dt = dt; 

        addRequirements(dt, vision); 
    }

    @Override
    public void execute() { 

        SmartDashboard.putBoolean("Align Running", true); 

        if (vision.hasTargets()) 
        {
            double xOffset = vision.getX(); 
            double yOffset = vision.getY(); 

            SmartDashboard.putNumber("X-Offset:", xOffset); 
            SmartDashboard.putNumber("Y-Offset:", yOffset);
            
            speeds.omegaRadiansPerSecond = Math.toRadians(xOffset * 2.5);

            dt.drive(speeds, false); 
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
