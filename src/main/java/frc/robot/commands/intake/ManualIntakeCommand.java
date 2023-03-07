package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;
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
        intake.intake(speed.getAsDouble());        
    }

    @Override
    public boolean isFinished() { 
        return false;
    }
}