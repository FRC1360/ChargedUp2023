package frc.robot.commands;

import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OpenCloseClaw extends CommandBase{
    private Claw motor; // that piston that open and closes the claw

    public OpenCloseClaw(Claw motor) {
        this.motor = motor;
        addRequirements(motor);
    }

    @Override
    public void execute() {
        motor.toggle();       
    }
}