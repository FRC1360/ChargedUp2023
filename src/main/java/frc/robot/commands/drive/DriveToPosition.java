package frc.robot.commands.drive;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.OrbitTimer;
import frc.robot.util.pathfinding.AStarNode;
import frc.robot.util.pathfinding.Field;
import frc.robot.util.pathfinding.Node;
import frc.robot.util.pathfinding.constraints.RectangularFieldConstraint;

public class DriveToPosition extends CommandBase {

    Field field;
    TrajectoryConfig trajectoryConfig;
    DrivetrainSubsystem dt;
    Trajectory trajectory;

    HolonomicDriveController driveController;

    PIDController xPidController;
    PIDController yPidController;
    ProfiledPIDController rotationController;
    TrapezoidProfile.Constraints rotationConstraints;

    OrbitTimer timer;

    Field2d simField;

    Thread generationThread;

    PathPlannerTrajectory pathPlannerTrajectory;

    public DriveToPosition(DrivetrainSubsystem dt) {

        trajectoryConfig = new TrajectoryConfig(3.0, 3.0);

        RectangularFieldConstraint constraint = new RectangularFieldConstraint(3.0, 1.0, 5.0, 3.5);

        field = new Field(0.5, 10.0, 10.0, constraint);

        this.dt = dt;

        xPidController = new PIDController(0.1, 0, 0);
        yPidController = new PIDController(0.1, 0, 0);
        rotationConstraints = new TrapezoidProfile.Constraints(6.28, 3.14);
        rotationController = new ProfiledPIDController(0.1, 0, 0, rotationConstraints);
        driveController = new HolonomicDriveController(xPidController, yPidController, rotationController);

        timer = new OrbitTimer();

        simField = new Field2d();

        addRequirements(dt);
    }

    @Override
    public void initialize() {

        generationThread = new Thread() {
            public void run() {
                Node start = field.getClosestNode(dt.getPose().getX(), dt.getPose().getY());
                Node end = field.getClosestNode(6.0, 1.5);

                System.out.println("Begining Waypoint Generation");

                ArrayList<AStarNode> waypoints = field.generateWaypoints(start, end);

                // The following is for cubic splines
                // Remove start and end waypoint from waypoints
                /*
                 * waypoints.remove(0);
                 * waypoints.remove(waypoints.size()-1);
                 * 
                 * Pose2d startPose = new Pose2d(start.getX(), start.getY(),
                 * Rotation2d.fromDegrees(0.0));
                 * System.out.println("Start Pose of (" + startPose.getX() + ", " +
                 * startPose.getY() + ")");
                 * Pose2d endPose = new Pose2d(end.getX(), end.getY(),
                 * Rotation2d.fromDegrees(0.0));
                 * 
                 * 
                 * ArrayList<Translation2d> translationWaypoints = new ArrayList<>();
                 */

                /*
                 * slation2d translation = new Translation2d(waypoint.getX(), waypoint.getY());
                 * System.out.println("Waypoint at (" + translation.getX() + ", " +
                 * translation.getY() + ")");
                 * translationWaypoints.add(translation);
                 * }
                 */

                /*
                 * int smoothingFactor = 5;
                 * for(int i = smoothingFactor-1; i < waypoints.size(); i+=smoothingFactor) {
                 * Translation2d translation = new Translation2d(waypoints.get(i).getX(),
                 * waypoints.get(i).getY());
                 * System.out.println("Waypoint at (" + translation.getX() + ", " +
                 * translation.getY() + ")");
                 * translationWaypoints.add(translation);
                 * }
                 */

                // trajectory = TrajectoryGenerator.generateTrajectory(startPose,
                // translationWaypoints, endPose, trajectoryConfig);

                ArrayList<PathPoint> pathPoints = new ArrayList<>();

                for (AStarNode node : waypoints) {
                    Translation2d translation = new Translation2d(node.getX(), node.getY());
                    Rotation2d heading = Rotation2d.fromDegrees(0.0);
                    Rotation2d rotation = Rotation2d.fromDegrees(0.0);

                    System.out.println("Waypoint at (" + translation.getX() + ", " +
                    translation.getY() + ")");

                    PathPoint point = new PathPoint(translation, heading, rotation);
                    pathPoints.add(point);
                }

                pathPlannerTrajectory = PathPlanner.generatePath(new PathConstraints(3, 3), pathPoints);

                System.out.println("Ending Waypoint Generation");
                timer.start();
            }
        };

        generationThread.start();

    }

    @Override
    public void execute() {
        if(generationThread.isAlive())
            return;

        Trajectory.State goal = pathPlannerTrajectory.sample(timer.getTimeDeltaSec());

        ChassisSpeeds speeds = driveController.calculate(dt.getPose(), goal, Rotation2d.fromDegrees(0.0));

        SmartDashboard.putNumber("Goal_X", goal.poseMeters.getX());
        SmartDashboard.putNumber("Goal_Y", goal.poseMeters.getY());
        SmartDashboard.putNumber("DT_X", dt.getPose().getX());
        SmartDashboard.putNumber("DT_Y", dt.getPose().getY());

        //PathPlannerState state = pathPlannerTrajectory.sample(timer.getTimeDeltaSec());


        //SmartDashboard.putData(simField);
        //simField.getObject("traj").setTrajectory(pathPlannerTrajectory);

        dt.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
