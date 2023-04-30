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
        inst.setServer("localhost"); // Default port is 0, may want to add as another parameter 
        this.nodeSubscriber = inst.getStringTopic("Selected Node").subscribe(""); 
        this.curSelectedNode = ""; 
    }

    @Override
    public void periodic() { 
        this.curSelectedNode = this.nodeSubscriber.get(); 
        System.out.println("Cur Node: " + this.curSelectedNode); 
    }

    public String getGUISelectedNode() { 
        return this.curSelectedNode; 
    }

    public static void main(String[] args) { 
        NodeSelectorNTClient client = new NodeSelectorNTClient(); 
        while (true) {
            System.out.println("Cur Node: " + client.getGUISelectedNode()); 
        }
    }
}
