package frc.robot.commands.shoulder;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.util.OrbitTimer;

public class ShoulderGoToPositionCommand extends CommandBase {
    
    private ShoulderSubsystem shoulder;
    private double angle;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;

    private OrbitTimer timer;

    public ShoulderGoToPositionCommand(ShoulderSubsystem shoulder, double angle) {
        this.shoulder = shoulder;
        this.angle = angle;
        this.timer = new OrbitTimer();
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        this.shoulder.movePIDController.reset();
        this.shoulder.setTargetAngle(angle);
        
        this.startState = new TrapezoidProfile.State(this.shoulder.getShoulderAngle(), 0.0);
        this.endState = new TrapezoidProfile.State(this.shoulder.getTargetAngle(), 0.0);

        this.motionProfile = new TrapezoidProfile(this.shoulder.shoulderMotionProfileConstraints,
             this.endState,
             this.startState);

        this.timer.start();

    }

    @Override
    public void execute() {

        TrapezoidProfile.State profileTarget = this.motionProfile.calculate(this.timer.getTimeDeltaSec());

        double target = profileTarget.position;

        SmartDashboard.putNumber("Shoulder_Move_Profile_Position", profileTarget.position);
        SmartDashboard.putNumber("Shoulder_Move_Profile_Velocity", profileTarget.velocity);

        double input = this.shoulder.getShoulderAngle();

        double speed = this.shoulder.movePIDController.calculate(target, input);
        this.shoulder.setShoulderNormalizedVoltage(speed);

    }

    @Override
    public boolean isFinished() {
        return this.motionProfile.isFinished(this.timer.getTimeDeltaSec());
        
    }
}
