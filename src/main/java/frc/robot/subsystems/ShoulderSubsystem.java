package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OrbitPID;

public class ShoulderSubsystem extends SubsystemBase {
    
    private CANSparkMax shoulderMotor;
    private double targetAngle;
    public OrbitPID pidController;

    public ShoulderSubsystem() {
        this.pidController = new OrbitPID(0, 0, 0);
        this.targetAngle = 0;
        this.shoulderMotor = new CANSparkMax(0, MotorType.kBrushless);
        
    }

    public double getEncoderPosition() {
        return this.shoulderMotor.getEncoder().getPosition();
    }

    public double getShoulderAngle() {
        return this.encoderToAngleConversion(this.getEncoderPosition());
    }

    public void setShoulderSpeed(double speed) {
        this.shoulderMotor.set(speed);
    }

    /*
     * Sets arm voltage based off 0.0 - 12.0
     */
    public void setShoulderVoltage(double voltage) {
        this.shoulderMotor.setVoltage(voltage);
    }

    /*
     * Sets arm voltage based off 0.0 - 1.0 
     */
    public void setShoulderNormalizedVoltage(double voltage) {
        this.setShoulderVoltage(voltage * 12.0);  // Should probably change this to a constant somewhere for ARM_VOLTAGE
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public double getTargetAngle() {
        return this.targetAngle;
    }

    /*
     * Converts angle (0 - 360) to encoder ticks
     */
    public double angleToEncoderConversion(double angle) {
        return 0.0;
    }

    /*
     * Converts encoder ticks to angle (0 - 360)
     */
    public double encoderToAngleConversion(double encoderPosition) {
        return 0.0;
    }
}
