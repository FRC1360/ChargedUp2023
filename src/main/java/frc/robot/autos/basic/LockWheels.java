package frc.robot.autos.basic;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LockWheels extends CommandBase {

    private DrivetrainSubsystem dt; 
    private ChassisSpeeds speeds; 
    
    public LockWheels(DrivetrainSubsystem dt) { 
        this.dt = dt; 
        this.speeds = new ChassisSpeeds(0.0, 0.0, 0.0); 

        addRequirements(dt);
    }

    @Override
    public void execute() { 
        this.speeds.omegaRadiansPerSecond = 0.0001; 
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
