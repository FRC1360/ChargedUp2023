package frc.robot.autos.basic;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LockWheels extends CommandBase {

    private DrivetrainSubsystem dt; 
    private double xSpeed; 
    private double ySpeed; 
    private ChassisSpeeds speeds; 
    
    public LockWheels(DrivetrainSubsystem dt, double xSpeed, double ySpeed) { 
        this.dt = dt; 
        this.xSpeed = xSpeed; 
        this.ySpeed = ySpeed; 
        this.speeds = new ChassisSpeeds(0.0, 0.0, 0.0); 

        addRequirements(dt);
    }

    @Override
    public void execute() { 
        this.speeds.vxMetersPerSecond = this.xSpeed; 
        this.speeds.vyMetersPerSecond = this.ySpeed; 
        dt.drive(speeds); 
    }

    @Override
    public void end(boolean interrupt) { 
    }

    @Override
    public boolean isFinished() { 
        //return this.speeds.vyMetersPerSecond > 0.0; 
        return false; 
    }
}
