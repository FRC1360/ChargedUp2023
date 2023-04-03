package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualPutdownCommand extends CommandBase {
    private IntakeSubsystem intake;
    private DoubleSupplier speed;

    public ManualPutdownCommand(IntakeSubsystem intake, DoubleSupplier speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void execute() {      
        intake.intake(-speed.getAsDouble() * 0.8);
    }   

    @Override
    public boolean isFinished() { 
        return false;
    }
}
