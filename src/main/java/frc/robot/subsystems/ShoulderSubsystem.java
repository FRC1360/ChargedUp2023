package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.util.OrbitPID;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;    
import frc.robot.Constants;

public class ShoulderSubsystem extends SubsystemBase {
    public static OrbitPID pid;
    private static CANSparkMax masterShoulder;
    private static CANSparkMax slaveShoulder;
    private static RelativeEncoder masterShoulderEncoder;
    private double encoderTargetPosition;

    public ShoulderSubsystem (int MasterShoulderID, int SlaveShoulderID) {
        masterShoulder = new CANSparkMax(MasterShoulderID, MotorType.kBrushless);
        slaveShoulder = new CANSparkMax(SlaveShoulderID, MotorType.kBrushless);
        
        masterShoulderEncoder = masterShoulder.getEncoder(); 

        // error checking to make sure motor config is successful
        // if the CAN bus starts getting weird, we'll notice it here
        SmartDashboard.putBoolean("ShoulderSlaveFollowSuccess:", 
                                    masterShoulder.setIdleMode(IdleMode.kBrake) == REVLibError.kOk);

        SmartDashboard.putBoolean("ShoulderSlaveFollowSuccess:", 
                                    slaveShoulder.follow(masterShoulder) == REVLibError.kOk);

        pid = new OrbitPID(0.1, 0, 0);
    }

    public void setZero() {
        masterShoulderEncoder.setPosition(0.0);
    }

    public void setSpeed(double speed) {
        masterShoulder.set(speed);
    }

    public double getAngle() {
        System.out.println(getPositionOfEncoder() / Constants.ROTATIONS_PER_ANGLE_PIVOT);
        return getPositionOfEncoder() / Constants.ROTATIONS_PER_ANGLE_PIVOT;
    }

    public double getStepsfromAngle(double degrees) {
        return Constants.ROTATIONS_PER_ANGLE_PIVOT * degrees;
    }

    public enum SHOULDER_POSITION {
        LOW_GOAL(5),
        MID_GOAL(20),
        HIGH_GOAL(45),
        INTAKE(0);

        private final double value;
        SHOULDER_POSITION(final double value) {
            this.value = value;
        }

        public double getValue() {
            return this.value;
        }
    }

    public double getPositionOfEncoder() {
        return masterShoulderEncoder.getPosition();
    }   

    public void setEncoderTargetPosition(double encoderTargetPosition) {
        this.encoderTargetPosition = encoderTargetPosition;
    }

    public void setEncoderTargetPosition(SHOULDER_POSITION position) {
        this.setEncoderTargetPosition(position.getValue());
    }
}
