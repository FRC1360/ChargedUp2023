package frc.robot.commands.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.OrbitTimer;

public class ArmGoToPositionCommand extends CommandBase {

    private ArmSubsystem arm;
    private double position;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;

    private OrbitTimer timer;

    public ArmGoToPositionCommand(ArmSubsystem arm, double position) {
            this.arm = arm;
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

        SmartDashboard.putNumber("Arm_Move_Speed", speed); 

        this.arm.setArmNormalizedVoltage(speed);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        //return this.motionProfile.isFinished(this.timer.getTimeDeltaSec());
        return false; 
    }


    
}
