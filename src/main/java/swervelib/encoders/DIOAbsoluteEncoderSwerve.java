package swervelib.encoders;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class DIOAbsoluteEncoderSwerve extends SwerveAbsoluteEncoder {

    private DigitalInput dio; 
    public DutyCycle encoder; 

    private boolean isInverted = false; 
    
    public DIOAbsoluteEncoderSwerve(int port) { 
        this.dio = new DigitalInput(port); 
        this.encoder = new DutyCycle(this.dio); 
    }

    @Override
    public void factoryDefault() {
    
    }

    @Override
    public void clearStickyFaults() {
        
    }

    @Override
    public void configure(boolean inverted) {
        this.isInverted = inverted; 
    }

    @Override
    /**
     * Get the absolute position of the encoder.
     *
     * @return Absolute position in degrees from [0, 360).
     */
    public double getAbsolutePosition() {
        //Offset is stored in the module config
        double angle = (this.isInverted ? -1.0 : 1.0)  * (360.0 * encoder.getOutput());

        angle %= 360.0;

        if (angle < 0.0) {
            angle += 360.0;
        }
        
        return angle;
    }

    @Override
    public Object getAbsoluteEncoder() {
        return this.encoder; 
    }
    
}
