package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;
import frc.robot.util.OrbitPID;

public class WristSubsystem extends SubsystemBase {

    private CANSparkMax wristMotor;
    private double wristOffset;  // Angle offset for the shoulder

    public OrbitPID holdPIDController;
    public OrbitPID movePIDController;
    public TrapezoidProfile.Constraints wristMotionProfileConstraints;

    private ShoulderWristMessenger shoulderWristMessenger;

    private double cacheOffset;

    public WristSubsystem(ShoulderWristMessenger shoulderWristMessenger) {
        this.wristMotor = new CANSparkMax(5, MotorType.kBrushless);
        this.wristOffset = 90.0;
        this.holdPIDController = new OrbitPID(0.004, 0.0000005, 0);
        this.movePIDController = new OrbitPID(0.004, 0.0, 0.0);  // TODO - Tune
        this.wristMotionProfileConstraints = new TrapezoidProfile.Constraints(2000.0, 1000.0);  // TODO - Tune

        this.shoulderWristMessenger = shoulderWristMessenger;

        this.wristMotor.restoreFactoryDefaults();
        this.wristMotor.setIdleMode(IdleMode.kBrake);
        this.wristMotor.setInverted(true);

        this.cacheOffset = 0.0;
    }

    public double getMotorRotations() {
        return this.wristMotor.getEncoder().getPosition();
    }

    
    // Returns the wrist GLOBAL angle. The global angle is the angle relative to the shoulder
    public double getWristAngle() {
        return this.encoderToAngleConversion(this.getMotorRotations());
    }

    public void setWristSpeed(double speed) {
        this.wristMotor.set(speed);
    }

    /*
     * Sets arm voltage based off 0.0 - 12.0
     */
    public void setWristVoltage(double voltage) {
        this.wristMotor.setVoltage(voltage);
    }

    /*
     * Sets arm voltage based off 0.0 - 1.0 
     */
    public void setWristNormalizedVoltage(double voltage) {
        this.setWristVoltage(voltage * 12.0);  // Should probably change this to a constant somewhere for ARM_VOLTAGE
    }

    // This return a GLOBAL angle. The global angle is the angle relative to the encoder
    public double getTargetAngle() {  // Use getTargetAngle() when doing commands to move the wrist
        
        return this.shoulderWristMessenger.getShoulderAngle() + this.getWristOffset();
    }
 
    public void setWristOffset(double offset) {
        this.wristOffset = offset;
    }

    // The offset is more akin to a LOCAL angle. The local angle is the angle relative to wrist starting position.
    // This angle ensures the wrist doesn't move relative to the starting location while the shoulder rotates
    // For example, if the wrist starts pointing directly up and the wrist offset is 0, the wrist will stay pointing up
    // regardless of the shoulder's orientation.  Change this value when you want to change the angle of the wrist
    public double getWristOffset() {
        return this.wristOffset;
    }

    public void setCacheOffset() {
        System.out.println("Setting cache offset to " + this.getWristOffset());
        this.cacheOffset = this.getWristOffset();
    }

    public double getCacheOffset() {
        return this.cacheOffset;
    }

    /*
     * Converts motor rotations to angle (0 - 360)
     */
    public double encoderToAngleConversion(double encoderPosition) {
        return (encoderPosition * 360.0 * (1.0/12.0)) % 360.0;
    }

    public void resetMotorRotations() {
        if(this.wristMotor.getEncoder().setPosition(0) == REVLibError.kOk) {
            System.out.println("Reset Shoulder Rotations to 0");
        } else {
            System.out.println("Failed to reset Shoulder Rotations");
        }
        
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Wrist_Hold_P_Gain", this.holdPIDController.getPTerm());
        SmartDashboard.putNumber("Wrist_Hold_I_Gain", this.holdPIDController.getITerm());
        SmartDashboard.putNumber("Wrist_Hold_D_Gain", this.holdPIDController.getDTerm());

        SmartDashboard.putNumber("Wrist_Target_Angle", this.getTargetAngle());
        SmartDashboard.putNumber("Wrist_Angle", this.getWristAngle());
        SmartDashboard.putNumber("Wrist_Motor_Rotations", this.getMotorRotations());
        SmartDashboard.putNumber("Wrist_Cache_Offset", this.getCacheOffset());
        SmartDashboard.putNumber("Wrist_Offset", this.getWristOffset());
    }
    
}
