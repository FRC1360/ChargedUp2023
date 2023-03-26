package frc.robot.autos.basic; 

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.OrbitPID;
import frc.robot.util.OrbitTimer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
public class Drive extends CommandBase {

    private final DrivetrainSubsystem dt; 

    private ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0); 

    // To scale down the meters for speed (m/sec)
    // Aka how much secs it takes to complete 
    private double scaleFactor = 1.1; 

    private Translation2d targetPose; 


    private TrapezoidProfile xMotionProfile; 
    private TrapezoidProfile yMotionProfile; 

    private OrbitTimer timer;  

    // Two PIDs used as values for x and y need to be independently calculated (one might be at target, other not)
    private OrbitPID driveXPID = new OrbitPID(0.5, 0.0, 0.0); 

    private OrbitPID driveYPID = new OrbitPID(0.5, 0.0, 0.0); 

    private double kF_X = 0.0; 
    private double kF_Y = 0.0;

    private double endSpeed = 0.0; 

    // private final Rotation2d targetAngle;

    // private Rotation2d angToTravel;

    // private double curAngle;

    // private double angle; 

    public Drive(DrivetrainSubsystem dt, double xMeters, double yMeters) { 
        this.dt = dt;

        this.speeds = new ChassisSpeeds(0.0, 0.0, 0.0); // 0 Rotation

        Translation2d curPose = dt.getTranslation();  
        this.targetPose = curPose.plus(new Translation2d(xMeters, yMeters));
        
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.Drivetrain.MOTION_PROFILE_MAX_VELOCITY, 
                                                                                    Constants.Drivetrain.MOTION_PROFILE_MAX_ACCELERATION); 

        TrapezoidProfile.State xStart = new TrapezoidProfile.State(curPose.getX(), 0.0); 
        TrapezoidProfile.State xEnd = new TrapezoidProfile.State(targetPose.getX(), 0.0);

        TrapezoidProfile.State yStart = new TrapezoidProfile.State(curPose.getY(), 0.0);
        TrapezoidProfile.State yEnd = new TrapezoidProfile.State(targetPose.getY(), 0.0);

        this.xMotionProfile = new TrapezoidProfile(constraints, xEnd, xStart); 
        this.yMotionProfile = new TrapezoidProfile(constraints, yEnd, yStart); 

        // double xDifference = targetPose.getX() - curPose.getX(); 
        // double yDifference = targetPose.getY() - curPose.getY(); 

        // if (xDifference != 0.0) kF_X = Math.copySign(0.3, xDifference); 
        // if (yDifference != 0.0) kF_Y = Math.copySign(0.3, yDifference); 

        this.timer = new OrbitTimer(); 

        addRequirements(dt); 
    }

    public Drive(DrivetrainSubsystem dt, double xMeters, double yMeters, double startSpeed, double endSpeed) { 
        this.dt = dt;

        this.speeds = new ChassisSpeeds(0.0, 0.0, 0.0); // 0 Rotation

        Translation2d curPose = dt.getTranslation();  
        this.targetPose = curPose.plus(new Translation2d(xMeters, yMeters));
        
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.Drivetrain.MOTION_PROFILE_MAX_VELOCITY, 
                                                                                    Constants.Drivetrain.MOTION_PROFILE_MAX_ACCELERATION); 

        TrapezoidProfile.State xStart; 
        TrapezoidProfile.State xEnd; 

        this.endSpeed = endSpeed; 

        if (xMeters != 0.0) xStart = new TrapezoidProfile.State(curPose.getX(), Math.copySign(startSpeed, xMeters)); 
        else xStart = new TrapezoidProfile.State(curPose.getX(), 0.0); 
        if (xMeters != 0.0) xEnd = new TrapezoidProfile.State(targetPose.getX(), Math.copySign(endSpeed, xMeters)); 
        else xEnd = new TrapezoidProfile.State(targetPose.getX(), 0.0);

        TrapezoidProfile.State yStart; 
        TrapezoidProfile.State yEnd;

        if (yMeters != 0.0) yStart = new TrapezoidProfile.State(curPose.getY(), Math.copySign(startSpeed, yMeters)); 
        else yStart = new TrapezoidProfile.State(curPose.getY(), 0.0);
        if (yMeters != 0.0) yEnd = new TrapezoidProfile.State(targetPose.getY(), Math.copySign(endSpeed, yMeters)); 
        else yEnd = new TrapezoidProfile.State(targetPose.getY(), 0.0);


        this.xMotionProfile = new TrapezoidProfile(constraints, xEnd, xStart); 
        this.yMotionProfile = new TrapezoidProfile(constraints, yEnd, yStart); 

        // double xDifference = targetPose.getX() - curPose.getX(); 
        // double yDifference = targetPose.getY() - curPose.getY(); 

        // if (xDifference != 0.0) kF_X = Math.copySign(0.3, xDifference); 
        // if (yDifference != 0.0) kF_Y = Math.copySign(0.3, yDifference); 

        this.timer = new OrbitTimer(); 

        addRequirements(dt); 

        // this.targetAngle = targetAngle; 

        // this.angToTravel = targetAngle.minus(dt.getPose().getRotation());
    }


    @Override
    public void initialize() { 
        SmartDashboard.putBoolean("Driving", true);

        this.timer.start(); 
        this.driveXPID.reset();
        this.driveYPID.reset();
    }

    @Override
    public void execute() { 
        SmartDashboard.putNumber("time", this.timer.getTimeDeltaSec());

        SmartDashboard.putNumber("P XDrive Term", this.driveXPID.getPTerm()); 
        SmartDashboard.putNumber("I XDrive Term", this.driveXPID.getITerm()); 
        SmartDashboard.putNumber("D XDrive Term", this.driveXPID.getDTerm());


        TrapezoidProfile.State xPosition = this.xMotionProfile.calculate(this.timer.getTimeDeltaSec()); 
        TrapezoidProfile.State yPosition = this.yMotionProfile.calculate(this.timer.getTimeDeltaSec()); 

        SmartDashboard.putNumber("x pos calc", xPosition.velocity); 
        SmartDashboard.putNumber("y pos calc", yPosition.velocity); 

        SmartDashboard.putNumber("curXSpeed", speeds.vxMetersPerSecond); 
        SmartDashboard.putNumber("curYSpeed", speeds.vyMetersPerSecond); 

        double xSpeed = driveXPID.calculate(xPosition.velocity, speeds.vxMetersPerSecond); 
        double ySpeed = driveYPID.calculate(yPosition.velocity, speeds.vyMetersPerSecond); 

        double time = this.timer.getTimeDeltaSec(); 

        if (xMotionProfile.isFinished(time) && !yMotionProfile.isFinished(time)) xSpeed = 0.0; 
        if (yMotionProfile.isFinished(time) && !xMotionProfile.isFinished(time)) ySpeed = 0.0;
        
        SmartDashboard.putNumber("x speed", xSpeed + kF_X); 
        SmartDashboard.putNumber("y speed", ySpeed + kF_Y); 

        speeds.vxMetersPerSecond = xSpeed + kF_X; 
        speeds.vyMetersPerSecond = ySpeed + kF_Y;

        dt.drive(speeds);

        // this.angle = this.angToTravel.getRadians()/scaleFactor; 
        // this.curAngle += this.angle; 
        // dt.setPoseOdometry(new Pose2d(dt.getTranslation().getX(), dt.getTranslation().getY(), Rotation2d.fromRadians(this.curAngle)));
    }
    
    @Override
    public void end (boolean interrupted) { 
        if (this.endSpeed == 0.0) dt.stop(); 
        SmartDashboard.putBoolean("Driving", false);
        //dt.setPoseOdometry(new Pose2d(targetPose.getX(), targetPose.getY(), dt.getPose().getRotation())); 
    }

    @Override
    public boolean isFinished() { 
        // Drive stops when both x and y are sqrt(2) from target
        // getDistance() calculated by pythagorean theorem
        //return Math.abs(dt.getTranslation().getDistance(targetPose)) < 0.05;
        return this.xMotionProfile.isFinished(this.timer.getTimeDeltaSec()) 
               && this.yMotionProfile.isFinished(this.timer.getTimeDeltaSec()); 
            //    && Math.abs(dt.getPose().getRotation().minus(targetAngle).getDegrees()) < 2; 
    }

}
