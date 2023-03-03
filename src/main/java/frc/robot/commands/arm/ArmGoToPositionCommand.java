package frc.robot.commands.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;
import frc.robot.util.OrbitTimer;

public class ArmGoToPositionCommand extends CommandBase {

    private ArmSubsystem arm;
    private double position;

    private ShoulderSubsystem shoulder; 

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;

    private OrbitTimer timer;

    public ArmGoToPositionCommand(ArmSubsystem arm, ShoulderSubsystem shoulder, double position) {
            this.arm = arm;
            this.shoulder = shoulder; 
            this.position = position;

            this.timer = new OrbitTimer();

            addRequirements(arm);
    }

    @Override
    public void initialize() {
        this.arm.movePIDController.reset();
        this.arm.setTargetDistance(this.position);
        
        this.startState = new TrapezoidProfile.State(this.arm.getArmDistance(), 0.0);
        this.endState = new TrapezoidProfile.State(this.arm.getTargetDistance(), 0.0);

        this.motionProfile = new TrapezoidProfile(this.arm.armMotionProfileConstraints, 
            endState,
            startState);
        
        this.timer.start();
    }

    @Override
    public void execute() {
        TrapezoidProfile.State profileTarget = this.motionProfile.calculate(this.timer.getTimeDeltaSec());

        double target = profileTarget.position;
        double input = this.arm.getArmDistance();

        SmartDashboard.putNumber("Arm_Move_Profile_Position", profileTarget.position);
        SmartDashboard.putNumber("Arm_Move_Profile_Velocity", profileTarget.velocity);

        double speed = this.arm.movePIDController.calculate(target, input);

        double kF = 0.18 * Math.cos(Math.toRadians(this.shoulder.getShoulderAngle())); 
       
        SmartDashboard.putNumber("Arm_Move_Speed", speed); 

        this.arm.setArmNormalizedVoltage(speed + kF);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return this.motionProfile.isFinished(this.timer.getTimeDeltaSec());
        //return false; 
    }


    
}
