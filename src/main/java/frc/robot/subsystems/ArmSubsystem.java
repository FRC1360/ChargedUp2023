package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OrbitPID;

public class ArmSubsystem extends SubsystemBase {
    
    private CANSparkMax armMotorMaster;
    private CANSparkMax armMotorSlave;

    public OrbitPID holdPIDController;
    public OrbitPID movePIDController;
    public TrapezoidProfile.Constraints armMotionProfileConstraintsExtend;
    public TrapezoidProfile.Constraints armMotionProfileConstraintsRetract;
    public DigitalInput limitSwitch;

    private double targetDistance;

    public ArmFeedforward armFeedforward; 

    private double armVelocity; 
    private double lastTime; 
    private Double lastDistance; 

    private DoubleSupplier manualOffset; 
    private BooleanSupplier manualOffsetEnable; 

    public ArmSubsystem(DoubleSupplier manualOffset, BooleanSupplier manualOffsetEnable) {
        this.armMotorMaster = new CANSparkMax(Constants.ARM_MOTOR_MASTER, MotorType.kBrushless);
        this.armMotorSlave = new CANSparkMax(Constants.ARM_MOTOR_SLAVE, MotorType.kBrushless);

        // kP = 0.2, kI = 0.0, kD = 0.0
        this.holdPIDController = new OrbitPID(0.4, 0.00005, 0.0);
        this.movePIDController = new OrbitPID(0.25, 0.0, 0.0);
        
        this.armFeedforward = new ArmFeedforward(0.0, 0.00, 0.0); //ks, kg, kv

        /*\
         * Max velocity initial calculation:
         * NEO RPM / 60 * Gear Ratio * PI * Drum Diameter ~= 34.66 in/s
         * 
         * Max acceleration inital calculation:
         * max velocity / 2 ~= 17.33 in/s^2
         */
        this.armMotionProfileConstraintsExtend = new TrapezoidProfile.Constraints(25.00, 42.0);
        this.armMotionProfileConstraintsRetract = new TrapezoidProfile.Constraints(25.00, 42.0);

        this.armMotorMaster.restoreFactoryDefaults();
        this.armMotorSlave.restoreFactoryDefaults();

        this.armMotorMaster.setIdleMode(IdleMode.kBrake);
        this.armMotorSlave.setIdleMode(IdleMode.kBrake);

        this.armMotorMaster.setInverted(false);
        this.armMotorSlave.setInverted(false);
        
        this.limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_ARM);

        this.armVelocity = 0.0; 
        //this.armMotorSlave.follow(this.armMotorMaster);
        
        this.lastTime = -1.0; 
        this.lastDistance = Double.NaN; 

        this.targetDistance = Constants.HOME_POSITION_ARM; 

        this.manualOffset = manualOffset; 
        this.manualOffsetEnable = manualOffsetEnable; 
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
        if (this.getArmDistance() > Constants.ARM_MAX_DISTANCE) speed = 0.0; 
        this.armMotorMaster.set(speed);
        this.armMotorSlave.set(speed);
    }

    public void setArmVoltage(double voltage) {
        if (this.getArmDistance() > Constants.ARM_MAX_DISTANCE) voltage = 0.0; 
        this.armMotorMaster.setVoltage(voltage);
        this.armMotorSlave.setVoltage(voltage);
    }

    public void setArmNormalizedVoltage(double voltage) {
        this.setArmVoltage(voltage * 12.0);
    }

    public double getTargetDistance() {

        /*double manualOffsetDistance;

        if (this.manualOffsetEnable.getAsBoolean()) {
            manualOffsetDistance = manualOffset.getAsDouble();

            if(this.targetDistance + manualOffsetDistance < 0.0) {  // If manual offset is negative (retracting) and the new target distance < 0.0
                manualOffsetDistance = 
            }

        } else {
            manualOffsetDistance = 0.0;
        }*/

        if(this.manualOffsetEnable.getAsBoolean()) {
            double newDistance = this.targetDistance + manualOffset.getAsDouble();

            if(newDistance < 0.0) {
                return 0.0;
            } else {
                return newDistance;
            }
        } else {
            return this.targetDistance;
        }


        /*return this.targetDistance 
                    + (this.manualOffsetEnable.getAsBoolean() ? manualOffset.getAsDouble() : 0.0);*/
    }

    // Distance in inches
    public void setTargetDistance(double distance) {
        this.targetDistance = distance;
    }

    public void resetEncoder() {
        this.armMotorMaster.getEncoder().setPosition(0.0);
    }

    public void updateArmVelocity() { 
        double currentTime = ((double) System.currentTimeMillis() / 1000.0); 
        double currentDistance = this.getArmDistance(); 

        if (this.lastTime != -1.0 && !this.lastDistance.isNaN()) {
            double deltaTime = currentTime - this.lastTime; 

            double deltaDistance = currentDistance - this.lastDistance.doubleValue(); 

            this.armVelocity = deltaDistance / deltaTime; 
        }
        
        this.lastDistance = currentDistance; 
        this.lastTime = currentTime; 
    }

    public double getArmVelocity() { 
        return this.armVelocity; 
    }

    public void updateSmartDashboard() {
        // SmartDashboard.putNumber("Arm_Hold_P_Gain", this.holdPIDController.getPTerm());
        // SmartDashboard.putNumber("Arm_Hold_I_Gain", this.holdPIDController.getITerm());
        // SmartDashboard.putNumber("Arm_Hold_D_Gain", this.holdPIDController.getDTerm());

        SmartDashboard.putNumber("Arm_Target_Distance", this.getTargetDistance());
        SmartDashboard.putNumber("Arm_Distance", this.getArmDistance());
        SmartDashboard.putNumber("Arm_Encoder_Value", this.armMotorMaster.getEncoder().getPosition()); 

        SmartDashboard.putNumber("Arm_Move_P_Gain", this.movePIDController.getPTerm());
        SmartDashboard.putNumber("Arm_Move_I_Gain", this.movePIDController.getITerm());
        SmartDashboard.putNumber("Arm_Move_D_Gain", this.movePIDController.getDTerm());

        SmartDashboard.putBoolean("Arm_Limit_Switch_Status", this.limitSwitch.get()); 

        SmartDashboard.putNumber("Arm_Velocity", 
                                this.getArmVelocity());
    }

    @Override
    public void periodic() { 
        updateArmVelocity(); 
    }

    public class ArmShoulderMessenger { 
        public double getArmDistance() { 
            return ArmSubsystem.this.getArmDistance(); 
        }

        public double getTargetDistance() { 
            return ArmSubsystem.this.getTargetDistance(); 
        }
    }

      
}
