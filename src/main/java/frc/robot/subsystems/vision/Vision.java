package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Vision extends SubsystemBase {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // TODO: Update to new NT API (pub/sub system)

    public Vision() { 

    }

    public enum Pipeline {
        CENTER_TARGET(0),
        RIGHT_TARGET(1),
        LEFT_TARGET(2);

        private final int id;
        private
         Pipeline(int id) {
            this.id = id;
        }
        public int getId() {
            return this.id;
        }
    }

    public void setPipeline(Pipeline pipe) {
        table.getEntry("pipeline").setDouble(pipe.getId());
    }

    public boolean hasTargets() { 
        return table.getEntry("tv").getDouble(0.0) > 0.0 ? true : false; 
    }

    public double getX() { 
        //Positive is too left        
        return table.getEntry("tx").getDouble(0.0); 
    }

    public double getY() { 
        return table.getEntry("ty").getDouble(0.0); 
    }

    public double[] getCamTran() {
        Number[] cam_tran = table.getEntry("camtran").getNumberArray({0.0});
        if(cam_tran.length != 6) {
            return {0.0};
        }
        double[] out = new double[6];
        for(int i = 0; i < 6; i++) {
            out[i] = cam_tran[i].doubleValue();
        }
        return out;
    }

    public double getDistanceFromTarget() {
        double angleOffset = Constants.CAMERA_MOUNT_ANGLE_DEG + getY(); 
        double distance = (Constants.TARGET_HEIGHT_METERS - Constants.CAMERA_MOUNT_HEIGHT_METERS) 
                            / Math.tan(Math.toRadians(angleOffset)); 
        return distance;   
    }

    public int getTagID() { 
        // Get AprilTag ID, returns -1 if not found
        //TODO: if incompatible type, i.e., it returns -1 always, try getInteger()
        return (int) table.getEntry("tid").getDouble(-1); 
    }

    public Pose2d getPoseFromTarget() { 
        //TODO: Test

        //Pose returns [tx, ty, tz, rx, ry, rz]
        double[] pose = table.getEntry("botpose").getDoubleArray(new double[] {-1, -1, -1, -1, -1, -1}); 
        
        return new Pose2d(pose[0], pose[1], new Rotation2d(pose[3], pose[4])); 
    }


    
}
