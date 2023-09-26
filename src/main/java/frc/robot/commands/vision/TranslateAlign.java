package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;

// aligns crosshair to vision tape
public class TranslateAlign extends CommandBase {

    private Vision vision;
    private SwerveSubsystem dt;

    public TranslateAlign(SwerveSubsystem dt, Vision vision) {
        this.vision = vision;
        this.dt = dt;

        addRequirements(dt, vision);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Align Running", true);

        if (vision.hasTargets()) {
            double xOffset = vision.getX();
            double yOffset = vision.getY();
            // double distance = vision.getDistanceFromTarget();

            SmartDashboard.putNumber("X-Offset:", xOffset);
            SmartDashboard.putNumber("Y-Offset:", yOffset);
            // SmartDashboard.putNumber("Distance:", distance);

            dt.drive(new Translation2d(-xOffset / 25, yOffset / 25), 0, true, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        dt.drive(new Translation2d(0, 0), 0, true, true);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(vision.getX()) < 1.5;
    }

}
