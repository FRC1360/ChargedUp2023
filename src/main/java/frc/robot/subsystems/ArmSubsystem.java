package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OrbitPID;

public class ArmSubsystem extends SubsystemBase {
    
    private CANSparkMax armMotor;
    private RelativeEncoder encoder;
    private double encoderTargetPosition;

    public OrbitPID pidController;

    public enum ARM_POSITION {
        LOW_GOAL(0),
        MID_GOAL(0),
        HIGH_GOAL(0),
        INTAKE(0);

        // value represents the output of the encoder at that position
        private final double value;
        ARM_POSITION(final double value) {
            this.value = value;
        }

        public double getValue() {
            return this.value;
        }
    }

    public ArmSubsystem() {
        this.armMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
        this.pidController = new OrbitPID(0, 0, 0);
        // TODO - Initialize the encoder
        this.encoder = armMotor.getEncoder();
        this.encoderTargetPosition = 0;
    }

    public double getEncoderPosition() {
        return this.encoder.getPosition();
    }

    public double getMotorRotations() {
        return getEncoderPosition()/this.encoder.getCountsPerRevolution();
    }

    public void setArmSpeed(double speed) {
        this.armMotor.set(speed);
    }

    /*
     * Sets arm voltage based off 0.0 - 12.0
     */
    public void setArmVoltage(double voltage) {
        this.armMotor.setVoltage(voltage); // should we do input clamping here? what happens if you put in a negative voltage? or one greater than 12?
    }

    /*
     * Sets arm voltage based off 0.0 - 1.0 
     */
    public void setArmNormalizedVoltage(double voltage) {
        this.setArmVoltage(voltage * Constants.BATTERY_VOLTAGE);
    }

    public void setEncoderTargetPosition(double encoderTargetPosition) {
        this.encoderTargetPosition = encoderTargetPosition;
    }

    public void setEncoderTargetPosition(ARM_POSITION position) {
        this.setEncoderTargetPosition(position.getValue());
    }

    public double getEncoderTargetPosition() {
        return this.encoderTargetPosition;
    }
}
