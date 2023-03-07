package frc.robot.commands.shoulder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderHoldCommand extends CommandBase {
    
    private ShoulderSubsystem shoulder;
    private DoubleSupplier intakeSpeed;

    public ShoulderHoldCommand(ShoulderSubsystem shoulder, DoubleSupplier intakeSpeed) {
        this.shoulder = shoulder;
        this.intakeSpeed = intakeSpeed;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        this.shoulder.holdPIDController.reset();
    }

    @Override
    public void execute() {

        double target = this.shoulder.getTargetAngle();
        double input = this.shoulder.getShoulderAngle();
        double speed = this.shoulder.holdPIDController.calculate(target, input);

        if (Math.abs(speed) > 0.25) speed = Math.copySign(0.25, speed); 

        // Remember to increase this value and also add kI please
        double kF;

        if(intakeSpeed.getAsDouble() > 0.05) {
           // kF = -0.2;
           kF = 0.18 * Math.sin(Math.toRadians(this.shoulder.getShoulderAngle()));
        } else {
            kF = 0.18 * Math.sin(Math.toRadians(this.shoulder.getShoulderAngle()));
        }

        this.shoulder.setShoulderNormalizedVoltage(speed + kF);
        //this.shoulder.setShoulderNormalizedVoltage(0.1); 

        SmartDashboard.putNumber("Shoulder_Hold_Output", speed + kF);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
