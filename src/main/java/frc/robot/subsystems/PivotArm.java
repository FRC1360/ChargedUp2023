package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

public class PivotArm extends SubsystemBase {

    private final CANSparkMax pivot;
    private final CANSparkMax arm;
    private final RelativeEncoder pivotEncoder;
    private final RelativeEncoder armEncoder;


    public PivotArm (int pivotID, int armID, MotorType type) {
        pivot = new CANSparkMax(pivotID, type);
        arm = new CANSparkMax(armID, type);

        pivotEncoder = pivot.getEncoder();
        armEncoder = arm.getEncoder();

        pivot.setIdleMode(IdleMode.kBrake); 
        arm.setIdleMode(IdleMode.kBrake); 

    } 

    public CANSparkMax getPivotMotor() {
        return pivot;
    } 

    public CANSparkMax getArmMotor() {
        return arm;
    } 

    public RelativeEncoder getPivotEncoder() {
        return pivotEncoder;
    }

    public int getDegrees(RelativeEncoder encoder) {
        return encoder / Constants.TICKS_PER_ANGLE_PIVOT;
    }

    public int getStepsfromDegrees(int degrees) {
        return Constants.TICKS_PER_ANGLE_PIVOT * degrees;
    }

    public int getCM(RelativeEncoder encoder) {
        return encoder / Constants.TICKS_PER_DISTANCE_CM;
    }

    public int getStepsfromCM(int cm) {
        return Constants.TICKS_PER_DISTANCE_CM * cm;
    }

    public void setZeroPivot()
    
}
