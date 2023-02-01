package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.util.OrbitPID;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;    
import frc.robot.Constants;

public class ShoulderSubsystem extends SubsystemBase {
    private static OrbitPID pid;
    private static CANSparkMax masterShoulder;
    private static CANSparkMax slaveShoulder;
    private static RelativeEncoder masterShoulderEncoder;
    private static RelativeEncoder slaveShoulderEncoder;
    private static double shoulderSteps;

    public ShoulderSubsystem (int MasterShoulderID, int SlaveShoulderID) {
        masterShoulder = new CANSparkMax(MasterShoulderID, MotorType.kBrushless);
        slaveShoulder = new CANSparkMax(SlaveShoulderID, MotorType.kBrushless);

        masterShoulderEncoder = masterShoulder.getEncoder();
        slaveShoulderEncoder = slaveShoulder.getEncoder();

        masterShoulder.setIdleMode(IdleMode.kBrake); 
        slaveShoulder.setIdleMode(IdleMode.kBrake); 

        slaveShoulder.follow(masterShoulder); // FIXME should the motors be inverted?
    } 

    public void initialize() {

    }

    public void execute() {

    }

    public void setZero() {
        masterShoulderEncoder.setPosition(0.0);
        slaveShoulderEncoder.setPosition(0.0);
    }

    public void setSpeed(double speed) {
        masterShoulder.set(speed);
        slaveShoulder.set(speed);
    }

    public double getAngle() {
        return getPositionOfEncoder() / Constants.TICKS_PER_ANGLE_PIVOT;
    }

    public double getStepsfromAngle(double degrees) {
        return Constants.TICKS_PER_ANGLE_PIVOT * degrees;
    }

    public void setTargetLow() {
        setAngle(5); // TODO Enter low target angle
    }
    
    public void setTargetMiddle() {
        setAngle(20); // TODO Enter Middle target angle
    }

    public void setTargetHigh() {
        setAngle(45); // TODO Enter High target angle
    }

    public double getPositionOfEncoder() {
        return masterShoulderEncoder.getPosition();
    }   

    public void setAngle(double degrees) {
        shoulderSteps = getStepsfromAngle(degrees);
        double pidoutput = pid.calculate(shoulderSteps, getPositionOfEncoder());
        setSpeed(pidoutput);
        if (pidoutput < 0.1 && pidoutput > -0.1) setSpeed(0);
    }
}
