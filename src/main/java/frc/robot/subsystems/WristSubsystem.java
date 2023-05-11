package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;

public class WristSubsystem extends SubsystemBase {

    private CANSparkMax wristMotor;
    private double wristOffset;  // Angle offset for the shoulder, should really be called angle 

    public SparkMaxPIDController pid;
    public int moveSlot = 0;
    public int holdSlot = 1;

    public ArmFeedforward wristFeedForward;

    private ShoulderWristMessenger shoulderWristMessenger;

    private DoubleSupplier manualOffset;
    private BooleanSupplier manualOffsetEnable;

    private AnalogEncoder absoluteEncoder;

    private double lastTime; 

    private Double lastAngle; 
    private Double angularVelocity; // degrees per second

    public WristSubsystem(ShoulderWristMessenger shoulderWristMessenger, DoubleSupplier manualOffset, BooleanSupplier manualOffsetEnable) {
        this.wristMotor = new CANSparkMax(Constants.WRIST_MOTOR, MotorType.kBrushless);
        
        this.wristOffset = 0.0;

        this.pid = wristMotor.getPIDController();
        setPIDValues(holdSlot, 0, 0, 0, 0, 0, 0); // SparkMAX needs much smaller constants than we had before, retune both of these
        setPIDValues(moveSlot, 0, 0, 0, 0, 0, 0);
        setSmartMotionValues(moveSlot, 0, 0); // and this!

        // SparkMAX can't do complex FF (gravity, etc.) so we keep this bit, dont need to retune i think
        this.wristFeedForward = new ArmFeedforward(0.0, 0.125, 0.0);

        this.shoulderWristMessenger = shoulderWristMessenger;

        this.wristMotor.restoreFactoryDefaults();
        this.wristMotor.setIdleMode(IdleMode.kBrake);
        this.wristMotor.setInverted(true);

        this.manualOffset = manualOffset;
        this.manualOffsetEnable = manualOffsetEnable;

        this.absoluteEncoder = new AnalogEncoder(Constants.WRIST_ENCODER);

        this.lastTime = -1; 
        this.lastAngle = Double.NaN; 
        this.angularVelocity = Double.NaN; 

        resetMotorRotations();
    }

    public void setIdleMode(IdleMode mode) { 
        this.wristMotor.setIdleMode(mode); 
    }

    public void resetMotorRotations() {
        double newPos = (this.absoluteEncoder.getAbsolutePosition() - Constants.WRIST_ENCODER_OFFSET) / Constants.WRIST_GEAR_RATIO;

        if(this.wristMotor.getEncoder().setPosition(newPos) == REVLibError.kOk) {
            System.out.println("Reset Shoulder Rotations to 0");
        } else {
            System.out.println("Failed to reset Shoulder Rotations");
        }
        
    }

    public double getMotorRotations() {
        return this.wristMotor.getEncoder().getPosition();
    }
    
    // Returns the wrist GLOBAL angle. The global angle is the angle relative to the shoulder
    public double getWristAngle() {
        return this.encoderToAngleConversion(this.getMotorRotations());
    }

    // This return a GLOBAL angle. The global angle is the angle relative to the shoulder
    public double getTargetAngle() {  // Use getTargetAngle() when doing commands to move the wrist
        return -this.shoulderWristMessenger.getShoulderAngle() + this.getWristOffset() + (manualOffsetEnable.getAsBoolean() ? -manualOffset.getAsDouble() : 0.0);
    }

    // The offset is more akin to a LOCAL angle. The local angle is the angle relative to wrist starting position.
    // This angle ensures the wrist doesn't move relative to the starting location while the shoulder rotates
    // For example, if the wrist starts pointing directly up and the wrist offset is 0, the wrist will stay pointing up
    // regardless of the shoulder's orientation.  Change this value when you want to change the angle of the wrist
    public void setWristOffset(double offset) {
        this.wristOffset = offset;
    }

    public double getWristOffset() {
        return this.wristOffset;
    }

    public double getWristAngleLocal() {
        return this.shoulderWristMessenger.getShoulderAngle() + this.getWristAngle();
    }

    /*
     * Converts motor rotations to angle (0 - 360)
     */
    public double encoderToAngleConversion(double encoderPosition) {
        return (encoderPosition * 360.0 * 2.0 * Constants.WRIST_GEAR_RATIO);
    }

    public void setPIDValues(int slot, double kP, double kI, double kD, double kFF, double kMax, double kMin) {
        pid.setP(kP, slot);
        pid.setI(kI, slot);
        pid.setD(kD, slot);
        pid.setFF(kFF, slot);
       
        pid.setOutputRange(kMin, kMax);  // handles speed limits, very important to prevent smokeys
    }

    public void setSmartMotionValues(int slot, double maxVel, double maxAccel) {
        pid.setSmartMotionMaxVelocity(maxVel, slot);
        pid.setSmartMotionMaxAccel(maxAccel, slot);
    }

    // Updates SmartMotion inputs
    // target should be encoder position 
    // FF is in % output, like before
    public void updateSmartMotion(int slot, double target, double ff) {
        pid.setReference(target, ControlType.kSmartMotion, slot, ff, ArbFFUnits.kPercentOut);
    }

    public void updateSmartDashboard() {
        // SmartDashboard.putNumber("Wrist_Target_Angle", this.getTargetAngle());
        SmartDashboard.putNumber("Wrist_Angle", this.getWristAngle());
        SmartDashboard.putNumber("Wrist_NEO_Encoder", this.getMotorRotations()); 
        SmartDashboard.putNumber("Wrist_Motor_Rotations", this.getMotorRotations());
        // SmartDashboard.putNumber("Wrist_Manual_Offset", this.manualOffset.getAsDouble());
        SmartDashboard.putNumber("Wrist_Offset", this.getWristOffset());
        SmartDashboard.putNumber("Wrist_Absolute_Encoder_Relative", this.absoluteEncoder.get());
        SmartDashboard.putNumber("Wrist_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Wrist_Angular_Velocity", this.getAngularVelocity().doubleValue()); 
    }

    public void updateAngularVelocity() { 
        double currentTime = (System.currentTimeMillis() / 1000.0); 
        double currentAngle = this.getWristAngle(); 

        if (this.lastTime != -1 && !this.lastAngle.isNaN()) {
            double deltaTime = currentTime - this.lastTime; 
            double deltaAngle = currentAngle - this.lastAngle.doubleValue(); 
            this.angularVelocity = deltaAngle / ((double) deltaTime); 
        }
        this.lastAngle = currentAngle; 
        this.lastTime = currentTime; 
    }

    public Double getAngularVelocity() {
        return this.angularVelocity;  
    }

    @Override
    public void periodic() { 
        updateAngularVelocity();
    }
    
}
