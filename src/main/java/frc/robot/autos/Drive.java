package frc.robot.autos; 

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
public class Drive extends CommandBase {

    private final DrivetrainSubsystem dt; 

    private ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0); 

    // To scale down the meters for speed (m/sec)
    // Aka how much secs it takes to complete 
    private double scaleFactor = 1.1; 

    private Translation2d targetPose; 

    private Translation2d curPose; 
    
    public Drive(DrivetrainSubsystem dt, double xMeters, double yMeters, Translation2d curPose) { 
        // Note: positive xMeters means to upwards, positive yMeters is left 
        this.dt = dt;
        
        this.curPose = curPose; 
        //TODO: Deal with scaling when one of x and y are less than 0.5
        if (Math.abs(xMeters) < 0.5 && xMeters != 0.0) this.speeds.vxMetersPerSecond = Math.copySign(xMeters*2.5, xMeters); 
        if (Math.abs(yMeters) < 0.5 && yMeters != 0.0) this.speeds.vyMetersPerSecond = Math.copySign(yMeters*2.5, yMeters);
        else {
        this.speeds = new ChassisSpeeds(xMeters/scaleFactor, yMeters/scaleFactor, 0); // 0 Rotation
        }
        
        //TODO: Check if Translation2d measures in meters
        this.targetPose = curPose.plus(new Translation2d(xMeters, yMeters)); 
        SmartDashboard.putNumber("trans", dt.getTranslation().getY());
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
        dt.setPoseOdometry(new Pose2d(curPose, Rotation2d.fromDegrees(0)));
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
        return Math.abs(dt.getTranslation().getDistance(targetPose)) < 0.05;
    }

}
