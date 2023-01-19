package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.vision.Vision;

public class TranslateAlign extends CommandBase {
    
    private Vision vision; 
    private DrivetrainSubsystem dt;  
    private final double DESIRED_POSITION; 
    private final ChassisSpeeds speeds = new ChassisSpeeds(); 

    public TranslateAlign(DrivetrainSubsystem dt, Vision vision, double DESIRED_POSITION) { 
        this.vision = vision; 
        this.dt = dt; 
        this.DESIRED_POSITION = DESIRED_POSITION; 
        
        addRequirements(dt, vision); 
    }

    @Override
    public void execute() { 

        SmartDashboard.putBoolean("Align Running", true); 

        if (vision.hasTargets()) 
        {
        double camX = vision.getCamTran()[0]; 


        if (camX < DESIRED_POSITION) { 
            speeds.vyMetersPerSecond = -0.2;
        }
        else if (camX > DESIRED_POSITION) { 
            speeds.vyMetersPerSecond = 0.2;
        }

        /* 
        double yOffset = vision.getY(); 
        double distance = vision.getDistanceFromTarget();

      

        SmartDashboard.putNumber("X-Offset:", xOffset); 
        SmartDashboard.putNumber("Y-Offset:", yOffset);
        SmartDashboard.putNumber("Distance:", distance);

        */

        dt.drive(speeds); 
        }
    }

    @Override
    public void end(boolean interrupted) { 
        dt.stop();
    }

    @Override
    public boolean isFinished() { 
        return Math.abs(vision.getCamTran[0] - DESIRED_POSITION) < 1.5;  
    }

}
