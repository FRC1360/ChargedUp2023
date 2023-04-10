package frc.robot.commands.drive;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory.Waypoint;

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
    DrivetrainSubsystem dt;

    HolonomicDriveController driveController;

    PIDController xPidController;
    PIDController yPidController;
    ProfiledPIDController rotationController;
    TrapezoidProfile.Constraints rotationConstraints;

    OrbitTimer timer;

    Field2d simField;

    Thread generationThread;

    PathPlannerTrajectory pathPlannerTrajectory;

    double xTarget;
    double yTarget;

    public DriveToPosition(DrivetrainSubsystem dt, double xTarget, double yTarget) {

        RectangularFieldConstraint constraint = new RectangularFieldConstraint(3.183-0.5, 1.935-0.5, 4.84+0.5, 4.08+0.5);
        RectangularFieldConstraint constraint2 = new RectangularFieldConstraint(0.0, 0.0, 1.75, 5.25);

        field = new Field(0.25, constraint, constraint2);

        this.dt = dt;

        xPidController = new PIDController(0.1, 0, 0);
        yPidController = new PIDController(0.1, 0, 0);
        rotationConstraints = new TrapezoidProfile.Constraints(6.28, 3.14);
        rotationController = new ProfiledPIDController(0.1, 0, 0, rotationConstraints);
        driveController = new HolonomicDriveController(xPidController, yPidController, rotationController);

        timer = new OrbitTimer();

        simField = new Field2d();

        this.xTarget = xTarget;
        this.yTarget = yTarget;

        addRequirements(dt);
    }

    @Override
    public void initialize() {

        generationThread = new Thread() {
            public void run() {
                Node start = field.getClosestNode(dt.getPose().getX(), dt.getPose().getY());
                //Node end = field.getClosestNode(5.4, 3.1);
                Node end = field.getClosestNode(xTarget, yTarget);

                System.out.println(start.getIsValid());

                System.out.println("Begining Waypoint Generation");

                ArrayList<AStarNode> waypoints = field.generateWaypoints(start, end);

                ArrayList<PathPoint> pathPoints = new ArrayList<>();

                int smoothFactor = 3;
                for (int i = 0; i < waypoints.size(); i+=smoothFactor) {
                    Translation2d translation = new Translation2d(waypoints.get(i).getX(), waypoints.get(i).getY());

                    Rotation2d heading;
                    if(i + smoothFactor > waypoints.size()) {
                        heading = new Rotation2d(waypoints.get(i).getX()-waypoints.get(i-smoothFactor).getX(), waypoints.get(i).getY()-waypoints.get(i-smoothFactor).getY());
                    } else {
                        heading = new Rotation2d(waypoints.get(i+smoothFactor).getX()-waypoints.get(i).getX(), waypoints.get(i+smoothFactor).getY()-waypoints.get(i).getY());
                    }

                    //Rotation2d heading = Rotation2d.fromRadians(0.0);


                    Rotation2d rotation = Rotation2d.fromDegrees(0.0);

                    System.out.println("Waypoint at (" + translation.getX() + ", " +
                    translation.getY() + ") and valid = " + waypoints.get(i).getIsValid());

                    PathPoint point = new PathPoint(translation, heading, rotation);
                    //PathPoint point = new PathPoint(translation, rotation);
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
        if(generationThread == null || generationThread.isAlive())
            return false;

        return pathPlannerTrajectory.getEndState() == (PathPlannerState)pathPlannerTrajectory.sample(timer.getTimeDeltaSec());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Command Ended");
    }

}
