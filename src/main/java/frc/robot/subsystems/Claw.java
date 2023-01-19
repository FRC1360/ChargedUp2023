package frc.robot.subsystems; // get rid of the main.java if this line of code is complaining

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Claw extends SubsystemBase {
    
    private Solenoid piston; // the piston to open and close the claw
    private int solenoidChannel = 5; //Idk the piston channel so let this be the placeholder for now

    public Claw() { 
        this.piston = new Solenoid(PneumaticsModuleType.CTREPCM, solenoidChannel); 
    }

    public void set(boolean on) { 
        piston.set(on);
    }

    public void toggle(){
        piston.toggle();
    }
}
