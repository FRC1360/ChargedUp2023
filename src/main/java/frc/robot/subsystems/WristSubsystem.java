package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;
import frc.robot.util.OrbitPID;

public class WristSubsystem extends SubsystemBase {

    private CANSparkMax wristMotor;
    private double wristOffset;  // Angle offset for the shoulder
    public OrbitPID pidController;

    private ShoulderWristMessenger shoulderWristMessenger;

    public WristSubsystem(ShoulderWristMessenger shoulderWristMessenger) {
        this.wristMotor = new CANSparkMax(1, MotorType.kBrushless);
        this.wristOffset = -90.0;
        this.pidController = new OrbitPID(0.0, 0.0, 0.0);

        this.shoulderWristMessenger = shoulderWristMessenger;
    }

    public double getMotorRotations() {
        return this.wristMotor.getEncoder().getPosition();
    }

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

    public double getTargetAngle() {
        return this.shoulderWristMessenger.getShoulderAngle() + this.wristOffset;
    }

    public void setWristOffset(double offset) {
        this.wristOffset = offset;
    }

    public double getWristOffset() {
        return this.wristOffset;
    }

    /*
     * Converts motor rotations to angle (0 - 360)
     */
    public double encoderToAngleConversion(double encoderPosition) {
        return 0.0;
    }
    
}
