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
        System.out.println("Set Wrist Offset = " + this.angle);
    }

    @Override
    public void execute() {
        double input = this.wrist.getWristAngle();

        // FF needs to be done by RoboRIO
        double feedforwardOutput = 0.0;
        if (!this.wrist.getAngularVelocity().isNaN()) { 
            feedforwardOutput = this.wrist.wristFeedForward.calculate(
                Math.toRadians(this.wrist.getWristAngleLocal()),
                Math.toRadians(this.wrist.getAngularVelocity())); 
        }

        this.wrist.updateSmartMotion(wrist.moveSlot, this.wrist.getTargetAngle(), feedforwardOutput);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.wrist.getWristAngle() - this.wrist.getTargetAngle()) < 3.0; // TODO: calculate profile ETA and use that here
    }
}
