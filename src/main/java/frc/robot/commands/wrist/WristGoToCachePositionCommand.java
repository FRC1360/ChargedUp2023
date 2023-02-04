package frc.robot.commands.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.OrbitTimer;

public class WristGoToCachePositionCommand extends CommandBase {
    
    private WristSubsystem wrist;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;

    private OrbitTimer timer;

    public WristGoToCachePositionCommand(WristSubsystem wrist) {
        this.wrist = wrist;
        this.timer = new OrbitTimer();
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        this.wrist.movePIDController.reset();
        this.wrist.setWristOffset(this.wrist.getCacheOffset());

        this.startState = new TrapezoidProfile.State(this.wrist.getWristAngle(), 0.0);
        this.endState = new TrapezoidProfile.State(this.wrist.getTargetAngle(), 0.0);

        this.motionProfile = new TrapezoidProfile(this.wrist.wristMotionProfileConstraints, 
            this.endState,
            this.startState);
        
        this.timer.start();
    }

    @Override
    public void execute() {
        TrapezoidProfile.State profileTarget = this.motionProfile.calculate(this.timer.getTimeDeltaSec());

        double target = profileTarget.position;
        double input = this.wrist.getWristAngle();

        SmartDashboard.putNumber("Wrist_Move_Profile_Position", profileTarget.position);
        SmartDashboard.putNumber("Wrist_Move_Profile_Velocity", profileTarget.velocity);

        double speed = this.wrist.movePIDController.calculate(target, input);
        this.wrist.setWristNormalizedVoltage(speed);
    }

    @Override
    public boolean isFinished() {
        return this.motionProfile.isFinished(this.timer.getTimeDeltaSec());
    }
}
