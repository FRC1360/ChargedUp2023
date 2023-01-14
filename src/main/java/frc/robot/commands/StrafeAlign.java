package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OrbitPID;

public class StrafeAlign extends CommandBase {
    private DrivetrainSubsystem dt;
    private Vision vision;

    private DoubleSupplier ystrafe;
    private DoubleSupplier xStrafe; 

    private OrbitPID pid = new OrbitPID(-2.5, 0, 0);

    public StrafeAlign(DrivetrainSubsystem dt, Vision vision, DoubleSupplier ystrafe, DoubleSupplier xStrafe) {
        this.dt = dt;
        this.vision = vision;
        this.ystrafe = ystrafe;
        this.xStrafe = xStrafe; 
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Strafe Align Running", true);

        if(vision.hasTargets()) {
            double xOffset = Math.toRadians(vision.getX()); 
            double yOffset = Math.toRadians(vision.getY());
            double distance = vision.getDistanceFromTarget();

            SmartDashboard.putNumber("X-Offset:", xOffset); 
            SmartDashboard.putNumber("Y-Offset:", yOffset);
            SmartDashboard.putNumber("Distance:", distance);

            double ystr = ystrafe.getAsDouble();
            double xstr = xStrafe.getAsDouble(); 
            double rot = pid.calculate(0, xOffset);

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xstr, ystr, rot, dt.getGyroscopeRotation()); // field-relatives
            // ChassisSpeeds speeds = ChassisSpeeds(0, str, rot); // robot-relative

            dt.drive(speeds);
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Strafe Align Running", false);
        dt.stop();
    }
}
