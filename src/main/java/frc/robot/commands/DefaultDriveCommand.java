package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.OrbitPID;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final CommandJoystick rotationJoystick; 

    private OrbitPID driveRotPID; 

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier, 
                               CommandJoystick rotationJoystick) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.rotationJoystick = rotationJoystick; 

        this.driveRotPID = new OrbitPID(0.1, 0.0, 0.0); 

        addRequirements(drivetrainSubsystem);
    }

    final double[] rotationalTargets = {
        0.0, 90.0, 180.0, 270.0
    };

    double[] rotationalError = new double[4];

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement

        double rotSpeed = m_rotationSupplier.getAsDouble(); 

        double curAngle = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() % 360.0;
        // rotationalError[0] = getDirection(rotationalTargets[0], curAngle);
        // rotationalError[1] = getDirection(rotationalTargets[1], curAngle);
        // rotationalError[2] = getDirection(rotationalTargets[2], curAngle);
        // rotationalError[3] = getDirection(rotationalTargets[3], curAngle);
        
        double curAngle2 = (m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + 180.0) % 360.0;
        // double direction = 0.0;

        SmartDashboard.putNumber("Default_Drive_Command_Cur_Angle", curAngle); 
        // 2 -> 0 (180deg in alternative frame of reference)
        // 5 -> 90
        // 3 -> 180
        // 4 -> 270

        
        if (this.rotationJoystick.button(3).getAsBoolean()) rotSpeed = -this.driveRotPID.calculate(180.0, curAngle); 
        else if (this.rotationJoystick.button(2).getAsBoolean()) rotSpeed = -this.driveRotPID.calculate(180.0, curAngle2); 
        else if (this.rotationJoystick.button(5).getAsBoolean()) rotSpeed = -this.driveRotPID.calculate(90.0, curAngle); 
        else if (this.rotationJoystick.button(4).getAsBoolean()) {
            rotSpeed = -this.driveRotPID.calculate(270.0, curAngle+(Math.abs(270-curAngle)>180?360:0));
        }
        

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            m_translationXSupplier.getAsDouble(),
            m_translationYSupplier.getAsDouble(),
            rotSpeed,
            m_drivetrainSubsystem.getGyroscopeRotation()
        );

        //System.out.println(speeds.toString());

        m_drivetrainSubsystem.drive(
                speeds
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    private double getDirection(double target, double current) {
        if (target==current) {
            return 0;
        }
        if (Math.abs(current - target) < 180.0) {
            if (current < target) {
                return 1.0;
            }
            else {
                return -1.0;
            } 
        }
        else {
            if (current < target) {
                return -1.0;
            }
            else {
                return 1.0;
            }
        }
    }
}
