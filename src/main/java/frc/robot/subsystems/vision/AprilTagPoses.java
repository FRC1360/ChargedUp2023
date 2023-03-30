package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class AprilTagPoses {
    

    public static final AprilTag[] aprilTagPosesList = {new AprilTag(0, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))), 
                                                        new AprilTag(1, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))), 
                                                        new AprilTag(2, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))), 
                                                        new AprilTag(3, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))), 
                                                        new AprilTag(4, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))), 
                                                        new AprilTag(5, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))), 
                                                        new AprilTag(6, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))),
                                                        new AprilTag(7, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))),
                                                        new AprilTag(8, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)))
                                                    }; // FIXME Get measurements from manual
    
}
