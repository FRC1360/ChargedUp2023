package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NodeSelectorNTServer extends SubsystemBase {

    // instantiated and used on robot operations
    private final StringSubscriber nodeSubscriber; 
    private String curSelectedNode; 

    private NetworkTableInstance inst; 
    
    public NodeSelectorNTServer() { 
        this.inst = NetworkTableInstance.getDefault(); 
        //this.inst.setServer("localhost", 0); // Default port is 0, may want to add as another parameter 
        this.inst.startServer();
        this.nodeSubscriber = inst.getStringTopic("Selected Node").subscribe(""); 
        this.curSelectedNode = ""; 
    }

    @Override
    public void periodic() { 
        this.curSelectedNode = this.nodeSubscriber.get(); 
        System.out.println("Cur Node: " + this.curSelectedNode); 
        //System.out.println("Connection: " + this.inst.getConnections()); 
    }

    public String getGUISelectedNode() { 
        return this.curSelectedNode; 
    }

    public static void main(String[] args) { 
        NodeSelectorNTServer client = new NodeSelectorNTServer(); 
        while (true) {
            System.out.println("Cur Node: " + client.getGUISelectedNode());
        }
    }
}
