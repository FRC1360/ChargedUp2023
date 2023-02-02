package frc.robot.commands.shoulder;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderGoToPositionCommand extends CommandBase {
    
    private ShoulderSubsystem shoulder;
    private double angle;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;

    private long startTime;

    public ShoulderGoToPositionCommand(ShoulderSubsystem shoulder, double angle) {
        this.shoulder = shoulder;
        this.angle = angle;
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

        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {

        long time = System.currentTimeMillis();
        double deltaTime = (time - startTime) / 1000.0;

        TrapezoidProfile.State profileTarget = this.motionProfile.calculate(deltaTime); // 0.02 is delta time = 20ms. Should probably fix this to actual time delta

        double target = profileTarget.position;

        SmartDashboard.putNumber("Shoulder_Move_Profile_Position", profileTarget.position);
        SmartDashboard.putNumber("Shoulder_Move_Profile_Velocity", profileTarget.velocity);

        double input = this.shoulder.getShoulderAngle();

        double speed = this.shoulder.movePIDController.calculate(target, input);
        this.shoulder.setShoulderNormalizedVoltage(speed);

    }

    @Override
    public boolean isFinished() {
        // TODO - Change to motionProfile.isFinished()
        //return Math.abs(this.shoulder.getShoulderAngle() - this.shoulder.getTargetAngle()) < 3;
        long time = System.currentTimeMillis();
        double deltaTime = (time - startTime) / 1000.0;

        return this.motionProfile.isFinished(deltaTime);
        
    }
}
