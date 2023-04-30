package frc.robot.nodeSelector;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NodeSelectorNTClient extends SubsystemBase {

    // instantiated and used on robot operations
    private final StringSubscriber nodeSubscriber; 
    private String curSelectedNode; 
    
    public NodeSelectorNTClient() { 
        NetworkTableInstance inst = NetworkTableInstance.getDefault(); 
        inst.startClient4("10.13.60.9:0");
        this.nodeSubscriber = inst.getStringTopic("Selected Node").subscribe(""); 
        this.curSelectedNode = ""; 
    }

    @Override
    public void periodic() { 
        this.curSelectedNode = this.nodeSubscriber.get(); 
    }

    public String getGUISelectedNode() { 
        return this.curSelectedNode; 
    }
}
