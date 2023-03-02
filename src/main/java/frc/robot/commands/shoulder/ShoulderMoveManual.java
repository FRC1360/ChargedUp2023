package frc.robot.commands.shoulder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderMoveManual extends CommandBase {

    DoubleSupplier joystick;
    ShoulderSubsystem shoulder;

    public ShoulderMoveManual(ShoulderSubsystem shoulder, DoubleSupplier joystick) {
        this.shoulder = shoulder;
        this.joystick = joystick;

        addRequirements(shoulder);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        this.shoulder.setShoulderNormalizedVoltage(joystick.getAsDouble());
        SmartDashboard.putNumber("Shoulder_Raw_Output", joystick.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
