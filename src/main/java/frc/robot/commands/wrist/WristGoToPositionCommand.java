package frc.robot.commands.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.OrbitTimer;

public class WristGoToPositionCommand extends CommandBase {
    
    private WristSubsystem wrist;
    private double angle;

    private OrbitTimer timer;

    public WristGoToPositionCommand(WristSubsystem wrist, double angle) {
        this.wrist = wrist;
        this.angle = angle;
        this.timer = new OrbitTimer();
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        this.wrist.setWristOffset(angle);
        this.timer.start();

        // TODO: calculate and store profile ETA
        double accel = this.wrist.pid.getSmartMotionMaxAccel(this.wrist.moveSlot);
        double vel = this.wrist.pid.getSmartMotionMaxVelocity(this.wrist.moveSlot);
        double accelTime = (vel - this.wrist.getAngularVelocity()) / accel; // time to reach top speed
        double stopTime = vel / accel; // time to stop
        double travelTime = 0; // TODO: finish trapezoid travel time calcs    

        System.out.println("Set Wrist Offset = " + this.angle);
    }

    @Override
    public void execute() {
        // FF needs to be done by RoboRIO
        double feedforwardOutput = 0.0;
        if (!this.wrist.getAngularVelocity().isNaN()) { 
            feedforwardOutput = this.wrist.wristFeedForward.calculate(
                Math.toRadians(this.wrist.getWristAngleLocal()),
                Math.toRadians(this.wrist.getAngularVelocity())); 
        }

        this.wrist.updateSmartMotion(this.wrist.moveSlot, this.wrist.getTargetAngle(), feedforwardOutput);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.wrist.getWristAngle() - this.wrist.getTargetAngle()) < 3.0; // TODO: calculate profile ETA and use that here
    }
}
