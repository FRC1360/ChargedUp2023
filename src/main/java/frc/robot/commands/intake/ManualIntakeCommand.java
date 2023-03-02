package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;

public class ManualIntakeCommand extends CommandBase{
    private IntakeSubsystem intake; // that piston that open and closes the claw
    //make a command that will take in a value that will spin it in a certain speed
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