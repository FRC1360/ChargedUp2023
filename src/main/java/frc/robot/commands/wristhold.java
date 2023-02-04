package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;




public class wristhold extends CommandBase {
    private static double degrees = 0.0;
    private static wristhold subsystem;
    private Object motor;




    public wristhold(wristhold ssystem, double degrees) {
        subsystem = ssystem;
        wristhold.degrees = degrees;
        addRequirements(ssystem);
    }




    public void stop(){
        ((Object) motor).stopMotor();
    }
   
   




    public void set(){
        Object speed;
        ((Object) motor).set(0);
    }






}
