package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OrbitPID;

public class ShoulderSubsystem extends SubsystemBase {
    
    private CANSparkMax shoulderMotorMaster;
    private double targetAngle;

    public OrbitPID holdPIDController;  // PID Controller for HoldToTarget
    public OrbitPID movePIDController;  // PID Controller for following Trapazoid Motion Profile
    public TrapezoidProfile.Constraints shoulderMotionProfileConstraints;

    private double angularVelocity;  // angular velocity in deg / second
    private double lastAngle;
    private long lastTime;

    public ShoulderSubsystem() {
        this.holdPIDController = new OrbitPID(0.015, 0.00001, 0);
        this.movePIDController = new OrbitPID(0.015, 0.0, 0.0);  // TODO - Tune

        // This units are deg / second for velocity and deg / sec^2 for acceleration
        this.shoulderMotionProfileConstraints = new TrapezoidProfile.Constraints(500, 250);  // TODO - Tune.
        this.targetAngle = 0;

        this.shoulderMotorMaster = new CANSparkMax(Constants.SHOULDER_MOTOR_MASTER, MotorType.kBrushless);

        this.shoulderMotorMaster.restoreFactoryDefaults();

        this.shoulderMotorMaster.setIdleMode(IdleMode.kBrake);

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

    public void resetMotorRotations() {
        if(this.shoulderMotorMaster.getEncoder().setPosition(0) == REVLibError.kOk) {
            System.out.println("Reset Shoulder Rotations to 0");
        } else {
            System.out.println("Failed to reset Shoulder Rotations");
        }
        
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

    private void updateAngularVelocity() {
        long currentTime = System.currentTimeMillis();

        double deltaTime = (currentTime - lastTime) / 1000.0;

        this.angularVelocity = (this.getShoulderAngle() - lastAngle) / deltaTime;
        this.lastAngle = this.getShoulderAngle();
        this.lastTime = currentTime;
    }

    public double getAngluarVelocity() {
        return this.angularVelocity;
    }

    @Override
    public void periodic() {
        updateAngularVelocity();
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Shoulder_Hold_P_Gain", this.holdPIDController.getPTerm());
        SmartDashboard.putNumber("Shoulder_Hold_I_Gain", this.holdPIDController.getITerm());
        SmartDashboard.putNumber("Shoulder_Hold_D_Gain", this.holdPIDController.getDTerm());

        SmartDashboard.putNumber("Shoulder_Target_Angle", this.getTargetAngle());
        SmartDashboard.putNumber("Shoulder_Angle", this.getShoulderAngle());

        SmartDashboard.putNumber("Shoulder_Angular_Velocity", this.getAngluarVelocity());

        SmartDashboard.putNumber("Shoulder_Move_P_Gain", this.movePIDController.getPTerm());
        SmartDashboard.putNumber("Shoulder_Move_I_Gain", this.movePIDController.getITerm());
        SmartDashboard.putNumber("Shoulder_Move_D_Gain", this.movePIDController.getDTerm());
    }

    public class ShoulderWristMessenger {
        public double getShoulderAngle() {
            return ShoulderSubsystem.this.getShoulderAngle();
        } 

        public double getTargetAngle() {
            return ShoulderSubsystem.this.getTargetAngle();
        }
    }
}
