package frc.robot.autos.pathplanner_autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.intake.AutoPutDownCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveForwardAuto extends AutoBase {
    public DriveForwardAuto(SwerveSubsystem swerve, IntakeSubsystem intake) {
        pathGroup = PathPlanner.loadPathGroup("driveforward", new PathConstraints(
                Constants.Swerve.AutoConstants.maxSpeed,
                Constants.Swerve.AutoConstants.maxAcceleration));

        eventMap = new HashMap<>();
        eventMap.put("event", new AutoPutDownCommand(intake, 0.5)); 
        // no events for now

        swerveAutoBuilder = new SwerveAutoBuilder(swerve::currentPose, swerve::setCurrentPose,
                Constants.Swerve.swerveKinematics,
                new PIDConstants(Constants.Swerve.AutoConstants.translation.p,
                        Constants.Swerve.AutoConstants.translation.i,
                        Constants.Swerve.AutoConstants.translation.d),
                new PIDConstants(Constants.Swerve.AutoConstants.rotation.p,
                        Constants.Swerve.AutoConstants.rotation.i,
                        Constants.Swerve.AutoConstants.rotation.d),
                swerve::setModuleStatesDuringAuto,
                eventMap,
                true,
                swerve);

    }

    @Override
    public Command getCommand() {
        return swerveAutoBuilder.fullAuto(pathGroup);
    }
}
