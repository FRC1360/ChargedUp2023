package frc.robot.util;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class DashboardTuning {
    private String subsystem;
    private double defaultValue;

    private double[] constantsArray;
    
    private String value1; 
    private String value2; 
    private String value3; 

    /** Constructor
     *  @param subsystem Name of the subsystem used
     *  @param type What controller is being used, and application
     *      seperated by underscore (in the form: Move_PID, Hold_PID, Feedforward)
     *  @param constantsArray Double array of constants used in controller
     *      PID : [kP, kI, kD]
     *      Feedforward: [kS, kG, kV]
    */
    public DashboardTuning(String subsystem, String type, double[] constantsArray) 
        {
        this.subsystem = subsystem;

        this.constantsArray = constantsArray; 

        if (type.contains("PID")) { 
            this.value1 = String.format("%s_%s_kP_Input", subsystem, type); 
            this.value2 = String.format("%s_%s_kI_Input", subsystem, type); 
            this.value3 = String.format("%s_%s_kD_Input", subsystem, type); 
        }  
        else if (type.contains("Feedforward")) { 
            this.value1 = String.format("%s_%s_kS_Input", subsystem, type); 
            this.value2 = String.format("%s_%s_kG_Input", subsystem, type);
            this.value3 = String.format("%s_%s_kV_Input", subsystem, type); 
        }
        else { 
            System.out.println("Invalid input provided"); 
        }

        putOnDashboard(this.value1, this.constantsArray[0]);
        putOnDashboard(this.value2, this.constantsArray[1]);
        putOnDashboard(this.value3, this.constantsArray[2]);
    }

    /** Methods */ 

    /** set default value of the number */
    public void putOnDashboard(String key, double defaultValue) {
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    }

    /** get the current value */
    public double get(String key, double defaultValue) {
        return SmartDashboard.getNumber(key, defaultValue);
    }

    /** check whether the value has been changed since last check */
    public boolean hasChanged(String key, double defaultValue) {
        double currentValue = get(key, defaultValue);
        if (currentValue != defaultValue) {
            return true;
        } else {
            return false;
        }
    }

    public void getAndUpdate(OrbitPID pid) { 
        if (this.hasChanged(this.value1, this.constantsArray[0]) 
             || this.hasChanged(this.value2, this.constantsArray[1])
             || this.hasChanged(this.value3, this.constantsArray[2])
            ) 
        { 
            double kP = get(this.value1, this.constantsArray[0]); 
            double kI = get(this.value2, this.constantsArray[1]); 
            double kD = get(this.value3, this.constantsArray[2]); 

            pid.configure(kP, kI, kD);

            this.constantsArray[0] = kP; 
            this.constantsArray[1] = kI; 
            this.constantsArray[2] = kD; 
        }
    } 

    public ArmFeedforward getAndUpdate(ArmFeedforward feedforward) { 
        if (this.hasChanged(this.value1, this.constantsArray[0]) 
            || this.hasChanged(this.value2, this.constantsArray[1])
            || this.hasChanged(this.value3, this.constantsArray[2])
            ) 
        { 
            feedforward = new ArmFeedforward(
                get(this.value1, this.constantsArray[0]),  
                get(this.value2, this.constantsArray[1]), 
                get(this.value2, this.constantsArray[2])
                                            );  
        }

        return feedforward;
    }


}