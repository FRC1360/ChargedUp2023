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
 
public class DriveAndRotate extends CommandBase {

    private final DrivetrainSubsystem dt; 

    private double xMeters; 
    private double yMeters; 

    private ChassisSpeeds speeds; 

    private TrapezoidProfile.State xStart; 
    private TrapezoidProfile.State xEnd; 
    private TrapezoidProfile.State yStart; 
    private TrapezoidProfile.State yEnd; 
    // private TrapezoidProfile.State rotStart;
    // private TrapezoidProfile.State rotEnd; 

    private TrapezoidProfile.Constraints driveConstraints; 
    // private TrapezoidProfile.Constraints rotationConstraints; 

    private TrapezoidProfile xMotionProfile; 
    private TrapezoidProfile yMotionProfile; 
    // private TrapezoidProfile rotationProfile; 

    private OrbitTimer timer;  

    private OrbitPID driveXPID;
    private OrbitPID driveYPID; 
    private OrbitPID driveRotPID; 

    private double targetAngle; 

    public DriveAndRotate(DrivetrainSubsystem dt, double xMeters, double yMeters, double targetAngle) { 
        this.dt = dt;
        this.xMeters = xMeters; 
        this.yMeters = yMeters; 
        this.targetAngle = targetAngle; 

        this.speeds = new ChassisSpeeds(0.0, 0.0, 0.0); // 0 Rotation
        
        this.driveConstraints = new TrapezoidProfile.Constraints(Constants.Drivetrain.DRIVE_MOTION_PROFILE_MAX_VELOCITY, 
                                                                    Constants.Drivetrain.DRIVE_MOTION_PROFILE_MAX_ACCELERATION); 
        
        // this.rotationConstraints = new TrapezoidProfile.Constraints(Constants.Drivetrain.ROTATION_MOTION_PROFILE_MAX_VELOCITY, 
        //                                                                 Constants.Drivetrain.ROTATION_MOTION_PROFILE_MAX_ACCELERATION); 

        // Two PIDs used as values for x and y need to be independently calculated
        this.driveXPID = new OrbitPID(1.0, 0.0, 0.0); 
        this.driveYPID = new OrbitPID(1.0, 0.0, 0.0);
        this.driveRotPID = new OrbitPID(4.0, 0.0, 3.0); 

        this.timer = new OrbitTimer();

        addRequirements(dt); 
    }
    

    @Override
    public void initialize() { 
        SmartDashboard.putBoolean("Driving", true);

        Translation2d curPose = dt.getTranslation(); 
        Translation2d targetPose = curPose.plus(new Translation2d(xMeters, yMeters));

        System.out.println("Target angle: " + this.targetAngle); 

        this.xStart = new TrapezoidProfile.State(curPose.getX(), 0.0); 
        this.xEnd = new TrapezoidProfile.State(targetPose.getX(), 0.0);

        this.yStart = new TrapezoidProfile.State(curPose.getY(), 0.0);
        this.yEnd = new TrapezoidProfile.State(targetPose.getY(), 0.0);

        // this.rotStart = new TrapezoidProfile.State(curAngle, 0.0); 
        // this.rotEnd = new TrapezoidProfile.State(targetAngle, 0.0); 

        this.xMotionProfile = new TrapezoidProfile(this.driveConstraints, this.xEnd, this.xStart); 
        this.yMotionProfile = new TrapezoidProfile(this.driveConstraints, this.yEnd, this.yStart);
        // this.rotationProfile = new TrapezoidProfile(this.rotationConstraints, this.rotEnd, this.rotStart); 

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

        TrapezoidProfile.State xProfileTarget = this.xMotionProfile.calculate(this.timer.getTimeDeltaSec()); 
        TrapezoidProfile.State yProfileTarget = this.yMotionProfile.calculate(this.timer.getTimeDeltaSec()); 
        // TrapezoidProfile.State rotationProfileTarget = this.rotationProfile.calculate(this.timer.getTimeDeltaSec()); 

        SmartDashboard.putNumber("x pos calc", xProfileTarget.velocity); 
        SmartDashboard.putNumber("y pos calc", yProfileTarget.velocity); 
        // SmartDashboard.putNumber("Rotation Motion Profile Velocity", rotationProfileTarget.velocity); 
        // SmartDashboard.putNumber("Rotation Motion Profile Position", rotationProfileTarget.position); 
        
        SmartDashboard.putNumber("curXSpeed", speeds.vxMetersPerSecond); 
        SmartDashboard.putNumber("curYSpeed", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("curRotSpeed", Math.toDegrees(speeds.omegaRadiansPerSecond) * 2.0);

        double xSpeed = driveXPID.calculate(xProfileTarget.velocity, speeds.vxMetersPerSecond); 
        double ySpeed = driveYPID.calculate(yProfileTarget.velocity, speeds.vyMetersPerSecond); 
        double curAngle = dt.getGyroscopeRotation().getDegrees() % 360.0; 

        double rotSpeed = Math.toRadians(driveRotPID.calculate(this.targetAngle, 
                                                    //dt.getGyroscopeRotation().getDegrees()));
                                                    curAngle)); 

        SmartDashboard.putNumber("Rot PID Out", rotSpeed); 

        SmartDashboard.putNumber("Gyro angle deg", curAngle); 
    

        this.speeds.vxMetersPerSecond = xSpeed; 
        this.speeds.vyMetersPerSecond = ySpeed;
        this.speeds.omegaRadiansPerSecond = -rotSpeed; 
        SmartDashboard.putNumber("Target angle", this.targetAngle); 
        
        //speeds.omegaRadiansPerSecond = -1.6; 

        ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(this.speeds, dt.getGyroscopeRotation()); 

        dt.drive(speed);
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
               && this.yMotionProfile.isFinished(this.timer.getTimeDeltaSec())
                // && this.rotationProfile.isFinished(this.timer.getTimeDeltaSec())
                && Math.abs(this.dt.getGyroscopeRotation().getDegrees() - this.targetAngle) < 2.0; 
    }
}
