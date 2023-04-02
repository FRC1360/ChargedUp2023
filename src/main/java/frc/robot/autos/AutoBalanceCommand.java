package frc.robot.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalanceCommand extends CommandBase {

    private AutoBalance autoBalanceFactory; 
    private ChassisSpeeds speeds; 

    private DrivetrainSubsystem dt; 

    public AutoBalanceCommand(DrivetrainSubsystem dt) { 
        this.dt = dt; 
        this.autoBalanceFactory = new AutoBalance(this.dt); 
        this.speeds = new ChassisSpeeds(0.0, 0.0, 0.0); 

        addRequirements(dt);
    }

    @Override
    public void execute() { 
        double xSpeed = this.autoBalanceFactory.autoBalanceRoutine()
                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND; 

        SmartDashboard.putNumber("Auto balance speed", xSpeed); 
        SmartDashboard.putNumber("Balance state", this.autoBalanceFactory.state); 
        SmartDashboard.putNumber("Debounce count", this.autoBalanceFactory.debounceCount); 

        SmartDashboard.putNumber("NavX Pitch Reading", this.autoBalanceFactory.getTilt()); 

        this.speeds.vxMetersPerSecond = xSpeed; 
        
        // if(xSpeed == 0.0) {
        //     speeds.vyMetersPerSecond = 0.01;
        // }

        dt.drive(this.speeds); 
    }

    @Override
    public void end(boolean interrupted) { 
        this.speeds.vyMetersPerSecond = 0.01; 
        dt.drive(this.speeds);
    }

    @Override
    public boolean isFinished() {
        return this.speeds.vxMetersPerSecond == 0.0; 
    }
    
}
