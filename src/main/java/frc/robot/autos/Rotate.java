package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ShoulderCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Rotate extends CommandBase {
    public Rotate(ShoulderCommand dt, Rotation2d targetAngle) { 
        //addRequirements(dt);
    }

    @Override
    public void execute() { 
        
    }

    @Override
    public void end (boolean interrupted) { 
        SmartDashboard.putBoolean("Rotating", false); 
    }

    @Override
    public boolean isFinished() { 
        // Stops when within 2 degrees -- TODO: Consider using PID to prevent overshoot 
        return true; //Math.abs(dt.getGyroscopeRotation().minus(targetAngle).getDegrees()) < 2; 
    }
    
}
