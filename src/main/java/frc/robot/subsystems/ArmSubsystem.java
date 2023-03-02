package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OrbitPID;

public class ArmSubsystem extends SubsystemBase {
    
    private CANSparkMax armMotorMaster;
    private CANSparkMax armMotorSlave;

    public OrbitPID holdPIDController;
    public OrbitPID movePIDController;
    public TrapezoidProfile.Constraints armMotionProfileConstraints;

    private double targetDistance;

    public ArmSubsystem() {
        this.armMotorMaster = new CANSparkMax(Constants.ARM_MOTOR_MASTER, MotorType.kBrushless);
        this.armMotorSlave = new CANSparkMax(Constants.ARM_MOTOR_SLAVE, MotorType.kBrushless);

        this.holdPIDController = new OrbitPID(0.01, 0.0, 0.0);
        this.movePIDController = new OrbitPID(0.01, 0.0, 0.0);

        /*\
         * Max velocity initial calculation:
         * NEO RPM / 60 * Gear Ratio * PI * Drum Diameter ~= 34.66 in/s
         * 
         * Max acceleration inital calculation:
         * max velocity / 2 ~= 17.33 in/s^2
         */
        this.armMotionProfileConstraints = new TrapezoidProfile.Constraints(34.66, 17.33);

        this.armMotorMaster.restoreFactoryDefaults();
        this.armMotorSlave.restoreFactoryDefaults();

        this.armMotorMaster.setIdleMode(IdleMode.kCoast);
        this.armMotorSlave.setIdleMode(IdleMode.kCoast);
        
        //this.armMotorSlave.follow(this.armMotorMaster);
    }

    public double getMotorRotations() {
        return this.armMotorMaster.getEncoder().getPosition();
    }

    public double getArmDistance() {
        return this.encoderToDistanceConversion(this.getMotorRotations());
    }

    // Returns distance arm has traveled in inches
    public double encoderToDistanceConversion(double encoderPosition) {
        return encoderPosition * Constants.ARM_GEAR_RATIO * Math.PI * Constants.ARM_DRUM_DIAMETER * Constants.ARM_PULLEY_BLOCK_RATIO;
    }

    public void setArmSpeed(double speed) {
        this.armMotorMaster.set(speed);
        this.armMotorSlave.set(speed);
    }

    public void setArmVoltage(double voltage) {
        this.armMotorMaster.setVoltage(voltage);
        this.armMotorSlave.setVoltage(voltage);
    }

    public void setArmNormalizedVoltage(double voltage) {
        this.setArmVoltage(voltage * 12.0);
    }

    public double getTargetDistance() {
        return this.targetDistance;
    }

    // Distance in inches
    public void setTargetDistance(double distance) {
        this.targetDistance = distance;
    }

    public void resetEncoder() {
        this.armMotorMaster.getEncoder().setPosition(0.0);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Arm_Hold_P_Gain", this.holdPIDController.getPTerm());
        SmartDashboard.putNumber("Arm_Hold_I_Gain", this.holdPIDController.getITerm());
        SmartDashboard.putNumber("Arm_Hold_D_Gain", this.holdPIDController.getDTerm());

        SmartDashboard.putNumber("Arm_Target_Distance", this.getTargetDistance());
        SmartDashboard.putNumber("Arm_Distance", this.getArmDistance());

        SmartDashboard.putNumber("Arm_Move_P_Gain", this.movePIDController.getPTerm());
        SmartDashboard.putNumber("Arm_Move_I_Gain", this.movePIDController.getITerm());
        SmartDashboard.putNumber("Arm_Move_D_Gain", this.movePIDController.getDTerm());
    }



}
