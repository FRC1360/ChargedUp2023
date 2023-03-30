package frc.robot.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalanceCommand extends CommandBase {

    private AutoBalance autoBalanceFactory; 
    private ChassisSpeeds speeds; 

    private DrivetrainSubsystem dt; 

    public AutoBalanceCommand(DrivetrainSubsystem dt) { 
        this.dt = dt; 
        this.autoBalanceFactory = new AutoBalance(); 
        this.speeds = new ChassisSpeeds(); 
    }

    @Override
    public void execute() { 
        double xSpeed = this.autoBalanceFactory.autoBalanceRoutine()
                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND; 

        speeds.vxMetersPerSecond = xSpeed; 

        dt.drive(speeds); 
    }
    
}
