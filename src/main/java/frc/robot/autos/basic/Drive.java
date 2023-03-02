package frc.robot.autos.basic; 

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

    private OrbitPID driveXPID = new OrbitPID(1.0, 0.0, 0.0); 

    private OrbitPID driveYPID = new OrbitPID(1.0, 0.0, 0.0); 

    public Drive(DrivetrainSubsystem dt, double xMeters, double yMeters) { 
        // Note: positive xMeters means to upwards, positive yMeters is left 
        this.dt = dt;
        /* 
        //TODO: Deal with scaling when one of x and y are less than 0.5
        if (Math.abs(xMeters) < 0.5 && xMeters != 0.0) this.speeds.vxMetersPerSecond = Math.copySign(xMeters*2.5, xMeters); 
        if (Math.abs(yMeters) < 0.5 && yMeters != 0.0) this.speeds.vyMetersPerSecond = Math.copySign(yMeters*2.5, yMeters);
        else {
        this.speeds = new ChassisSpeeds(xMeters/scaleFactor, yMeters/scaleFactor, 0); // 0 Rotation
        }

        */
        this.speeds = new ChassisSpeeds(0.0, 0.0, 0.0); // 0 Rotation
        //TODO: Check if Translation2d measures in meters

        Translation2d curPose = dt.getTranslation();  
        this.targetPose = curPose.plus(new Translation2d(xMeters, yMeters));
        
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(3.0, 12.0); 

        TrapezoidProfile.State xStart = new TrapezoidProfile.State(curPose.getX(), 0.0); 
        TrapezoidProfile.State xEnd = new TrapezoidProfile.State(targetPose.getX(), 0.0);

        TrapezoidProfile.State yStart = new TrapezoidProfile.State(curPose.getY(), 0.0);
        TrapezoidProfile.State yEnd = new TrapezoidProfile.State(targetPose.getY(), 0.0);

        this.xMotionProfile = new TrapezoidProfile(constraints, xEnd, xStart); 
        this.yMotionProfile = new TrapezoidProfile(constraints, yEnd, yStart);

        this.timer = new OrbitTimer(); 

        addRequirements(dt); 
    }


    @Override
    public void initialize() { 
        SmartDashboard.putBoolean("Driving", true);

        this.timer.start(); 
    }

    @Override
    public void execute() { 
        SmartDashboard.putNumber("time", this.timer.getTimeDeltaSec());

        TrapezoidProfile.State xPosition = this.xMotionProfile.calculate(this.timer.getTimeDeltaSec()); 
        TrapezoidProfile.State yPosition = this.yMotionProfile.calculate(this.timer.getTimeDeltaSec()); 

        SmartDashboard.putNumber("x pos calc", xPosition.velocity); 
        SmartDashboard.putNumber("y pos calc", yPosition.velocity); 

        SmartDashboard.putNumber("pid", driveXPID.getPTerm());
        SmartDashboard.putNumber("curXSpeed", speeds.vxMetersPerSecond); 
        SmartDashboard.putNumber("curYSpeed", speeds.vyMetersPerSecond); 

        double xSpeed = driveXPID.calculate(xPosition.velocity, speeds.vxMetersPerSecond); 
        double ySpeed = driveYPID.calculate(yPosition.velocity, speeds.vyMetersPerSecond); 

        SmartDashboard.putNumber("x speed", xSpeed); 
        SmartDashboard.putNumber("y speed", ySpeed); 

        speeds.vxMetersPerSecond = xSpeed; 
        speeds.vyMetersPerSecond = ySpeed;

        dt.drive(speeds);
    }
    
    @Override
    public void end (boolean interrupted) { 
        dt.stop(); 
        SmartDashboard.putBoolean("Driving", false);
        //dt.setPoseOdometry(new Pose2d(targetPose, Rotation2d.fromDegrees(0))); 
    }

    @Override
    public boolean isFinished() { 
        // Drive stops when both x and y are sqrt(2) from target
        // getDistance() calculated by pythagorean theorem
        //return Math.abs(dt.getTranslation().getDistance(targetPose)) < 0.2;
        return this.xMotionProfile.isFinished(this.timer.getTimeDeltaSec()) 
                && this.yMotionProfile.isFinished(this.timer.getTimeDeltaSec()); 
    }

}