package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{

    private Spark LEDController;
    private double LEDColour;
    private boolean enableLED;
    
    
    public LEDSubsystem() {
        this.LEDController = new Spark(0);  // TODO - Change PWM port
        this.LEDColour = 0.0;
        this.enableLED = true;
    }

    public void setLEDEnable() {
        this.enableLED = true;
    }

    public void setLEDDisable() {
        this.enableLED = false;
    }

    public void setLEDCode(double value) {
        this.LEDColour = value;
    }

    @Override
    public void periodic() {
        /*if(this.enableLED) {
            this.LEDColour = 0.95;
        } else {
            this.LEDColour = 0.61;
        }*/

        this.LEDController.set(this.LEDColour);
    }

}
