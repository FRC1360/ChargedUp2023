package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Vision extends SubsystemBase {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // TODO: Update to new NT API (pub/sub system)

    private IntegerSubscriber tv, tid;
    private DoubleSubscriber tx, ty;
    private DoubleArraySubscriber botpose;

    public Vision() { 
        tv = table.getIntegerTopic("tv").subscribe(0);
        tid = table.getIntegerTopic("tid").subscribe(0);
        
        tx = table.getDoubleTopic("tx").subscribe(0);
        ty = table.getDoubleTopic("ty").subscribe(0);

        botpose = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    }

    public boolean hasTargets() { 
        return tv.get() > 0.0; 
    }

    public double getX() { 
        // Positive is to the left        
        return tx.get(); 
    }

    public double getY() { 
        return ty.get();
    }

    public int getTagID() { 
        // Get AprilTag ID, returns -1 if not found
        //TODO: if incompatible type, i.e., it returns -1 always, try getInteger()
        return (int) tid.get();
    }

    public Pose2d getPoseFromTarget() {
        //Pose returns [tx, ty, tz, rx, ry, rz]
        double[] pose = botpose.get();
        
        return new Pose2d(pose[0], pose[1], new Rotation2d(pose[3], pose[4])); 
    }

    public double getDistanceFromTarget() {
        double angleOffset = Constants.CAMERA_MOUNT_ANGLE_DEG + getY(); 
        double distance = (Constants.TARGET_HEIGHT_METERS - Constants.CAMERA_MOUNT_HEIGHT_METERS) 
                            / Math.tan(Math.toRadians(angleOffset)); 
        return distance;   
    }
}
