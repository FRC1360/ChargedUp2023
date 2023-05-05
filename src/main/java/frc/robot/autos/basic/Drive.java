package frc.robot.autos.basic; 

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive.DrivetrainSubsystem;
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

    private double xMeters; 
    private double yMeters; 

    private ChassisSpeeds speeds; 

    private TrapezoidProfile.State xStart; 
    private TrapezoidProfile.State xEnd; 
    private TrapezoidProfile.State yStart; 
    private TrapezoidProfile.State yEnd; 

    private TrapezoidProfile.Constraints driveConstraints; 

    private TrapezoidProfile xMotionProfile; 
    private TrapezoidProfile yMotionProfile; 

    private OrbitTimer timer;  

    private OrbitPID driveXPID;
    private OrbitPID driveYPID; 

    // private final Rotation2d targetAngle;

    // private Rotation2d angToTravel;

    // private double curAngle;

    // private double angle; 

    public Drive(DrivetrainSubsystem dt, double xMeters, double yMeters) { 
        this.dt = dt;
        this.xMeters = xMeters; 
        this.yMeters = yMeters; 

        this.speeds = new ChassisSpeeds(0.0, 0.0, 0.0); // 0 Rotation, left is positive y, forwards is positive x
        
        this.driveConstraints = new TrapezoidProfile.Constraints(Constants.Drivetrain.DRIVE_MOTION_PROFILE_MAX_VELOCITY, 
                                                                    Constants.Drivetrain.DRIVE_MOTION_PROFILE_MAX_ACCELERATION); 

        // Two PIDs used as values for x and y need to be independently calculated
        this.driveXPID = new OrbitPID(1.0, 0.0, 0.0); 
        this.driveYPID = new OrbitPID(1.0, 0.0, 0.0);

        this.timer = new OrbitTimer(); 

        addRequirements(dt); 
    }

    @Override
    public void initialize() { 
        SmartDashboard.putBoolean("Driving", true);

        Translation2d curPose = dt.getTranslation(); 
        Translation2d targetPose = curPose.plus(new Translation2d(xMeters, yMeters));

        this.xStart = new TrapezoidProfile.State(curPose.getX(), 0.0); 
        this.xEnd = new TrapezoidProfile.State(targetPose.getX(), 0.0);

        this.yStart = new TrapezoidProfile.State(curPose.getY(), 0.0);
        this.yEnd = new TrapezoidProfile.State(targetPose.getY(), 0.0);

        this.xMotionProfile = new TrapezoidProfile(this.driveConstraints, this.xEnd, this.xStart); 
        this.yMotionProfile = new TrapezoidProfile(this.driveConstraints, this.yEnd, this.yStart); 

        this.timer.start(); 
        this.driveXPID.reset();
        this.driveYPID.reset();
    }

    @Override
    public void execute() { 
        // SmartDashboard.putNumber("time", this.timer.getTimeDeltaSec());

        // SmartDashboard.putNumber("P XDrive Term", this.driveXPID.getPTerm()); 
        // SmartDashboard.putNumber("I XDrive Term", this.driveXPID.getITerm()); 
        // SmartDashboard.putNumber("D XDrive Term", this.driveXPID.getDTerm());


        TrapezoidProfile.State xPosition = this.xMotionProfile.calculate(this.timer.getTimeDeltaSec()); 
        TrapezoidProfile.State yPosition = this.yMotionProfile.calculate(this.timer.getTimeDeltaSec()); 

        // SmartDashboard.putNumber("x pos calc", xPosition.velocity); 
        // SmartDashboard.putNumber("y pos calc", yPosition.velocity); 

        // SmartDashboard.putNumber("curXSpeed", speeds.vxMetersPerSecond); 
        // SmartDashboard.putNumber("curYSpeed", speeds.vyMetersPerSecond);

        double xSpeed = driveXPID.calculate(xPosition.velocity, speeds.vxMetersPerSecond); 
        double ySpeed = driveYPID.calculate(yPosition.velocity, speeds.vyMetersPerSecond);
        
        // SmartDashboard.putNumber("x speed", xSpeed); 
        // SmartDashboard.putNumber("y speed", ySpeed); 

        speeds.vxMetersPerSecond = xSpeed; 
        speeds.vyMetersPerSecond = ySpeed;

        dt.drive(speeds, true);
    }
    
    @Override
    public void end (boolean interrupted) { 
        dt.stop(); 
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
