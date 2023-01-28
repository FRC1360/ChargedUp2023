package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OrbitPID;

public class ArmSubsystem extends SubsystemBase {
    
    private CANSparkMax armMotor;
    private Encoder encoder;
    private int encoderTargetPosition;

    public OrbitPID pidController;

    public enum ARM_POSITION {
        LOW_GOAL(0),
        MID_GOAL(0),
        HIGH_GOAL(0),
        INTAKE(0);

        private final int value;
        ARM_POSITION(final int value) {
            this.value = value;
        }

        public int getValue() {
            return this.value;
        }
    }

    public ArmSubsystem() {
        this.armMotor = new CANSparkMax(0, MotorType.kBrushless);  // TODO - Add constant
        this.pidController = new OrbitPID(0, 0, 0);
        // TODO - Initialize the encoder
        this.encoderTargetPosition = 0;
    }

    public int getEncoderPosition() {
        return this.encoder.get();
    }

    public void setArmSpeed(double speed) {
        this.armMotor.set(speed);
    }

    public void setArmVoltage(double voltage) {
        this.armMotor.setVoltage(voltage);
    }

    public void setEncoderTargetPosition(int encoderTargetPosition) {
        this.encoderTargetPosition = encoderTargetPosition;
    }

    public void setEncoderTargetPosition(ARM_POSITION position) {
        this.setEncoderTargetPosition(position.getValue());
    }

    public int getEncoderTargetPosition() {
        return this.encoderTargetPosition;
    }
}