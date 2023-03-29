package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {

    // get the ll net table
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    /*
    * API reference:
    * 
    * Integers:
    * tv: is there a valid target (0 or 1)
    * tid: primary target ID
    * 
    * Doubles:
    * tx/ty: target pos in screen space
    * 
    * Arrays:
    * botpose: robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
    * targetpose_cameraspace: transform of primary target in camera space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
    */

    private DoubleSubscriber tv, tid;
    private DoubleSubscriber tx, ty;
    private DoubleArraySubscriber botpose, targetpose_cameraspace;

    private IntegerPublisher ledMode, cammMode, pipeline, stream, snapshot;

    public Vision() {
        // integer topics
        tv = table.getDoubleTopic("tv").subscribe(0);
        tid = table.getDoubleTopic("tid").subscribe(-1);
        
        // double topics
        tx = table.getDoubleTopic("tx").subscribe(0);
        ty = table.getDoubleTopic("ty").subscribe(0);

        // array topics
        botpose = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
        targetpose_cameraspace = table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});
    }

    public boolean hasTargets() { 
        return tv.get() > 0;
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

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("tid", table.getEntry("tid").getInteger(-1));
        SmartDashboard.putNumberArray("targetpose_cameraspace", targetpose_cameraspace.get());
    }

    @Override
    public void periodic() {
        // update SmartDashboard with numbers constantly
        updateSmartDashboard();
    }
}
