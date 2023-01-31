package frc.robot.commands;

import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UserInput extends CommandBase{
    private Claw motor; // that piston that open and closes the claw

    public UserInput(Claw Motor) {
        this.motor = motor;
        addRequirements(motor);
    }

}