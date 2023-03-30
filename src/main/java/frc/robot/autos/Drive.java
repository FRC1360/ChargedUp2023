package frc.robot.autos; 

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
public class Drive extends CommandBase {

    private final DrivetrainSubsystem dt; 

    private ChassisSpeeds speeds; 

    // To scale down the meters for speed (m/sec)
    // Aka how much secs it takes to complete 
    private double scaleFactor = 6; 

    private Translation2d targetPose; 

    public Drive(DrivetrainSubsystem dt, double xMeters, double yMeters) { 
        // Note: positive xMeters means to upwards, positive yMeters is left 
        this.dt = dt;
        
        this.speeds = new ChassisSpeeds(xMeters/scaleFactor, yMeters/scaleFactor, 0); // 0 Rotation
        
        //TODO: Check if Translation2d measures in meters
        this.targetPose = dt.getTranslation().plus(new Translation2d(xMeters, yMeters)); 

        addRequirements(dt); 
    }

    public Drive(DrivetrainSubsystem dt, Translation2d targetPose) { 
        this.dt = dt; 
        this.targetPose = targetPose; 

        // Calculate x and y speeds 
        Translation2d disToTravel = targetPose.minus(dt.getTranslation()); 

        this.speeds = new ChassisSpeeds(disToTravel.getX()/scaleFactor, disToTravel.getY()/scaleFactor, 0); // 0 Rotation

        addRequirements(dt); 
    }

    @Override
    public void initialize() { 
        SmartDashboard.putBoolean("Driving", true);
    }

    @Override
    public void execute() { 
        dt.drive(speeds);
    }
    
    @Override
    public void end (boolean interrupted) { 
        dt.stop();
        SmartDashboard.putBoolean("Driving", false);   
    }

    @Override
    public boolean isFinished() { 
        // Drive stops when both x and y are sqrt(2) from target
        // getDistance() calculated by pythagorean theorem
        return Math.abs(dt.getTranslation().getDistance(targetPose)) < 2; 
    }



}
