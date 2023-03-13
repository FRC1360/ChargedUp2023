package frc.robot.autos.basic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Rotate extends CommandBase {

    private final Rotation2d targetAngle; 

    private final DrivetrainSubsystem dt; 

    private Rotation2d angToTravel; 

    private final ChassisSpeeds speeds; 

    private final double scaleFactor = 20; 
    
    private double angle = 0.0; 

    private double curAngle; 

    double x = 5.8; 
    double y = 1.5; 

    double xIncre; 
    double yIncre; 
  
    public Rotate(DrivetrainSubsystem dt, Rotation2d targetAngle, double xIncre, double yIncre, double xPos, double yPos, double curAngle) { 
        this.dt = dt; 
        this.xIncre = xIncre; 
        this.yIncre = yIncre; 
        this.x = xPos; 
        this.y = yPos; 
        this.targetAngle = targetAngle; 

        this.curAngle = curAngle;

        this.angToTravel = targetAngle.minus(Rotation2d.fromDegrees(curAngle)); 

        this.speeds = new ChassisSpeeds(0.0, 0.0, angToTravel.getRadians()/scaleFactor); 

        //addRequirements(dt);
        
    }

    @Override
    public void initialize() { 
        SmartDashboard.putBoolean("Rotating", true); 
        
    }

    @Override
    public void execute() { 
        //this.angToTravel = targetAngle.minus(dt.getGyroscopeRotation());
        this.curAngle += this.angToTravel.getRadians()/scaleFactor; 
        this.x += this.xIncre; 
        this.y += this.yIncre; 
        dt.setPoseOdometry(new Pose2d(this.x, this.y, Rotation2d.fromRadians(this.curAngle)));
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
        return Math.abs(dt.getPose().getRotation().minus(targetAngle).getDegrees()) < 2;
        //return Math.abs(dt.getGyroscopeRotation().minus(targetAngle).getDegrees()) < 2; 
    }
    
}
