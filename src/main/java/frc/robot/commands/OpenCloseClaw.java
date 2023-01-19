package frc.robot.commands;

import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OpenCloseClaw extends CommandBase{
    private Claw piston; // that piston that open and closes the claw

    public OpenCloseClaw(Claw piston) {
        this.piston = piston;
        addRequirements(piston);
    }

    @Override
    public void execute() {
        piston.toggle();// toggle basically makes it do the opposite of whatever state it is in. Ex. if the claw is open it will close and vice versa. 
    }
}