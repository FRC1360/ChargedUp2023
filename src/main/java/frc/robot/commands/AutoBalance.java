package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.OrbitPID;

public class AutoBalance extends CommandBase {

    private DrivetrainSubsystem dt; 
    private ChassisSpeeds speeds = new ChassisSpeeds(0, -1, 0); 
    private OrbitPID pid = new OrbitPID(-0.035, 0, 0); 

    private boolean got; 
    private Translation2d targetPose; 

    private Translation2d curPose; 
    
    public AutoBalance(DrivetrainSubsystem dt) { 
        this.dt = dt; 
        this.got = false; 

        addRequirements(dt);
    }

    @Override
    public void execute() { 
        double curPitch = dt.getPitch(); 

        System.out.println(curPitch); 
        if (got) {
        System.out.println(dt.getTranslation().getY() - curPose.getY());
        }

        if (!got && Math.abs(curPitch) > 4) { 
            curPose = dt.getTranslation(); 
            targetPose = dt.getTranslation().plus(new Translation2d(0, -2)); 
            got = true; 
        }

        
        if (got && dt.getTranslation().getDistance(targetPose) < 0.1) { 
            dt.stop(); 
        }
        


        /* 
        double ySpeed = pid.calculate(0, curPitch);
        
        speeds.vyMetersPerSecond = ySpeed; 

        */

        
        dt.drive(speeds);
        
    }
}
