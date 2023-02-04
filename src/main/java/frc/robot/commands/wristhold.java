package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;




public class wristhold extends CommandBase {
    private static double degrees = 0.0;
    private static wristhold wrist;




    public wristhold(wristhold wrist, double degrees) {
        this.wrist = wrist;
        wristhold.degrees = degrees;
        addRequirements(wrist);
    }




    private void addRequirements(wristhold subsystem2) {
    }

    
     



    






}

