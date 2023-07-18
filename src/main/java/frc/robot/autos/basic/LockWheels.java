package frc.robot.autos.basic;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveDrive.DrivetrainSubsystem;

public class LockWheels extends CommandBase {

    private SwerveSubsystem dt;
    private ChassisSpeeds speeds;

    public LockWheels(SwerveSubsystem dt) {
        this.dt = dt;
        this.speeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        addRequirements(dt);
    }

    @Override
    public void execute() {
        dt.drive(new Translation2d(), 0, true, true);
        dt.brake();
    }

    @Override
    public void end(boolean interrupt) {
    }

    @Override
    public boolean isFinished() {
        // return this.speeds.vyMetersPerSecond > 0.0;
        return false;
    }
}
