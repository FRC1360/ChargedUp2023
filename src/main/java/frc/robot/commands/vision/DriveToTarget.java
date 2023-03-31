// package frc.robot.commands.vision;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.vision.Vision;

// public class DriveToTarget extends CommandBase {
    
//     private Vision vision; 
//     private DrivetrainSubsystem dt;
//     private ChassisSpeeds speeds = new ChassisSpeeds(); 

//     public DriveToTarget(DrivetrainSubsystem dt, Vision vision) { 
//         this.vision = vision; 
//         this.dt = dt; 

//         addRequirements(dt, vision); 
//     }

//     @Override
//     public void execute() { 
//         if (vision.hasTargets()) { 
//         double disToTarget = vision.getDistanceFromTarget(); 

//         speeds.vxMetersPerSecond = disToTarget; 

//         dt.drive(speeds);
//         }
//     }

//     @Override
//     public void end(boolean interrupted) { 
//         dt.stop();
//     }

//     public boolean isFinished() { 
//         return vision.getDistanceFromTarget() < 3; 
//     }


// }
