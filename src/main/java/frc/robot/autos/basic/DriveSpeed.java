package frc.robot.autos.basic;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveSpeed extends CommandBase {

    private DrivetrainSubsystem dt; 
    private double xSpeed; 
    private double ySpeed; 
    private ChassisSpeeds speeds; 
    
    public DriveSpeed(DrivetrainSubsystem dt, double xSpeed, double ySpeed) { 
        this.dt = dt; 
        this.xSpeed = xSpeed; 
        this.ySpeed = ySpeed; 
        this.speeds = new ChassisSpeeds(0.0, 0.0, 0.0); 

        addRequirements(dt);
    }

    @Override
    public void execute() { 
        speeds.vxMetersPerSecond = this.xSpeed; 
        speeds.vyMetersPerSecond = this.ySpeed; 
        dt.drive(speeds); 
    }
}
