package frc.robot.subsystems;

import java.util.NoSuchElementException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants; 

public class Vision extends SubsystemBase {

    public PhotonCamera camera; 
    public AprilTagFieldLayout aprilTagFieldLayout; 

    public Vision(int pipelineIndex) { 
        
        //If want to use NetworkTables, add an additional parameter, otherwise use PhotonLib
        this.camera = new PhotonCamera(NetworkTableInstance.getDefault(), "AprilTag"); 
        //this.aprilTagFieldLayout = new AprilTagFieldLayout() // Need to hold a list of apriltag poses, and field dimensions OR json file

        camera.setPipelineIndex(pipelineIndex);
        
    }

    public boolean hasTargets() { 
        return camera.getLatestResult().hasTargets(); 
    }

    
    public PhotonTrackedTarget getBestTarget() { 
        //Out of a list of targets, the "best" is choosen based on config
         return camera.getLatestResult().getBestTarget();
    }

    public double getX() {
        // Right offset positive; measured in degrees 
        return getBestTarget().getYaw(); 
    }

    public double getY() {
        // Positive is degrees offset up 
        return getBestTarget().getPitch();
    }

    public double getDistanceFromTarget() {
        // TODO: Figure out a way to adjust 2nd parameter of target height to encompass all nodes (enum?)
        return PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_MOUNT_HEIGHT_METERS, 
                                                            Constants.TARGET_HEIGHT_METERS, 
                                                            Math.toRadians(Constants.CAMERA_MOUNT_ANGLE_DEG), 
                                                            Math.toRadians(getY())); 
    }

    
    public Pose2d getFieldPoseAprilTag() { 
        try 
        {
            return PhotonUtils.estimateFieldToRobotAprilTag(getBestTarget().getBestCameraToTarget(), 
                                                        aprilTagFieldLayout.getTagPose(getTagID()).get(), // .get() is to remove the Optional wrapping
                                                        Constants.CAMERA_TO_ROBOT).toPose2d(); // Ideally, we won't need the z-axis
        }
        catch(NoSuchElementException e) 
        { 
            return null;
        }
    }

    public int getTagID() { 
        // Only applicable for AprilTags, -1 for N/A
        return getBestTarget().getFiducialId();
    }
    

    
}
