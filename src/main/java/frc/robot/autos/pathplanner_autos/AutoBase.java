package frc.robot.autos.pathplanner_autos;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBase {
    public SwerveSubsystem swerve;
    public SwerveAutoBuilder swerveAutoBuilder;
    public List<PathPlannerTrajectory> pathGroup;
    public HashMap<String, Command> eventMap;

    public AutoBase updateAutoBuilder() {
        swerveAutoBuilder = new SwerveAutoBuilder(swerve::currentPose,
                swerve::setCurrentPose,
                Constants.Swerve.swerveKinematics,
                new PIDConstants(Constants.Swerve.AutoConstants.translation.p,
                        Constants.Swerve.AutoConstants.translation.i,
                        Constants.Swerve.AutoConstants.translation.d),
                new PIDConstants(Constants.Swerve.AutoConstants.rotation.p, Constants.Swerve.AutoConstants.rotation.i,
                        Constants.Swerve.AutoConstants.rotation.d),
                swerve::setModuleStates,
                eventMap,
                true,
                swerve);
        return this;

    }

    public Pose2d getInitialHolonomicPose() {
        return pathGroup.get(0).getInitialHolonomicPose();
    }

    public Command getCommand() {
        return Commands.none();
    }
}
