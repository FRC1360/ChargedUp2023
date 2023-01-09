package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Rotate extends CommandBase {

    private final Rotation2d targetAngle; 

    private final DrivetrainSubsystem dt; 

    private final Rotation2d angToTravel; 

    private final ChassisSpeeds speeds; 

    private final double scaleFactor = 2; 

  
    public Rotate(DrivetrainSubsystem dt, Rotation2d targetAngle) { 
        this.dt = dt; 
        this.targetAngle = targetAngle; 

        this.angToTravel = targetAngle.minus(dt.getGyroscopeRotation()); 

        this.speeds = new ChassisSpeeds(0.0, 0.0, angToTravel.getRadians()/scaleFactor); 

        addRequirements(dt);
        
    }

    @Override
    public void initialize() { 
        SmartDashboard.putBoolean("Rotating", true); 
    }

    @Override
    public void execute() { 
        dt.drive(speeds);
    }

    @Override
    public void end (boolean interrupted) { 
        dt.stop();  
        SmartDashboard.putBoolean("Rotating", false); 
    }

    @Override
    public boolean isFinished() { 
        // Stops when within 2 degrees -- TODO: Consider using PID to prevent overshoot
        return Math.abs(dt.getGyroscopeRotation().minus(targetAngle).getDegrees()) < 2; 
    }
    
}
