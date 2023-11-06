package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class ManualIntakeCommand extends CommandBase{
    private IntakeSubsystem intake;
    private DoubleSupplier speed; // the speed given by the controller

    public ManualIntakeCommand(IntakeSubsystem intake, DoubleSupplier speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }
    @Override
    public void execute() {
        if (this.intake.getAtSubstationState()) intake.intake(speed.getAsDouble() * 0.25);  // Speed cap of 25%   
        else intake.intake(speed.getAsDouble() * 0.25);      
        SmartDashboard.putNumber("Intake speed", this.speed.getAsDouble()); 
    }

    @Override
    public boolean isFinished() { 
        return false;
    }
}