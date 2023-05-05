package frc.robot.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDrive.DrivetrainSubsystem;

public class Simulator extends SubsystemBase {

    // ONLY FOR TRANSLATION on swerve, ROTATION WON'T WORK!!!

    private DrivetrainSubsystem dt;

    private Field2d field = new Field2d();

    public Simulator(DrivetrainSubsystem dt) { 
        this.dt = dt;

        SmartDashboard.putData(field);
    }

    @Override 
    public void periodic() { 
        field.setRobotPose(dt.getPose());
    }
}
