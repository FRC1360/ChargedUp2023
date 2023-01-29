package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OrbitPID;

public class ShoulderSubsystem extends SubsystemBase {
    
    private CANSparkMax shoulderMotorMaster;
    private CANSparkMax shoulderMotorSlave;
    private double targetAngle;
    public OrbitPID pidController;

    public ShoulderSubsystem() {
        this.pidController = new OrbitPID(0.0055, 0, 0);
        this.targetAngle = 0;

        this.shoulderMotorMaster = new CANSparkMax(Constants.SHOULDER_MOTOR_MASTER, MotorType.kBrushless);
        this.shoulderMotorSlave = new CANSparkMax(Constants.SHOULDER_MOTOR_SLAVE, MotorType.kBrushless);

        this.shoulderMotorMaster.restoreFactoryDefaults();
        this.shoulderMotorSlave.restoreFactoryDefaults();

        this.shoulderMotorMaster.setIdleMode(IdleMode.kBrake);
        this.shoulderMotorSlave.setIdleMode(IdleMode.kBrake);

        this.shoulderMotorSlave.follow(this.shoulderMotorMaster);
    }


    public double getMotorRotations() {
        return this.shoulderMotorMaster.getEncoder().getPosition();
    }

    public double getShoulderAngle() {
        return this.rotationsToAngleConversion(this.getMotorRotations());
    }

    public void setShoulderSpeed(double speed) {
        this.shoulderMotorMaster.set(speed);
    }

    /*
     * Sets arm voltage based off 0.0 - 12.0
     */
    public void setShoulderVoltage(double voltage) {
        this.shoulderMotorMaster.setVoltage(voltage);
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
     * Converts motor rotations  to angle (0 - 360)
     */
    public double rotationsToAngleConversion(double encoderPosition) {
        // encoderPosition * 360.0 = angle of motor rotation
        // angle of motor rotation * GEAR_RATIO = shoulder angle
        // shoulder angle % 360 = keep range between 0-360
        return (encoderPosition * 360.0 * Constants.SHOULDER_GEAR_RATIO) % 360.0;
    }

    class ShoulderWristMessenger {
        public double getShoulderAngle() {
            return ShoulderSubsystem.this.getShoulderAngle();
        } 
    }
}
