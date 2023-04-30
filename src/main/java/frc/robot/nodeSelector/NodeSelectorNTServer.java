package frc.robot.nodeSelector;

import java.io.IOException;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;

import java.io.IOException;

import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.util.WPIUtilJNI;

public class NodeSelectorNTServer extends SubsystemBase {
    // TO BE RUN ON THE DRIVER STATION WITH GUI PROGRAM
    private final StringPublisher nodePublisher; 
    private NodeSelectorGUI gui; 
    
    public NodeSelectorNTServer(NodeSelectorGUI gui) throws IOException { 
        try {
            // Copied from WPILib docs
            NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
            WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
            WPIMathJNI.Helper.setExtractOnStaticLoad(false);
            CameraServerJNI.Helper.setExtractOnStaticLoad(false);
            
            CombinedRuntimeLoader.loadLibraries(NodeSelectorNTServer.class, "wpiutiljni", "wpimathjni", "ntcorejni",
                "cscorejnicvstatic");
        } catch (IOException ex) { 
            System.out.println("ERROR: Loading of WPILib libraries for running GUI NT Server unsuccessful!");
            throw new IOException(); 
        }

        NetworkTableInstance inst = NetworkTableInstance.getDefault(); 
        inst.setServer("10.13.60.9"); // Default port is 0, may want to add as another parameter 
        inst.startServer();
        this.nodePublisher = inst.getStringTopic("Selected Node").publish(); 
        this.gui = gui; 
    }

    public void run() {

        while (true) { 
            try { 
                Thread.sleep(1000); // Reduce unnecessary writes to nt
            } catch (InterruptedException ex) { 
                System.out.println("GUI NT Server interrupted"); 
                return; 
            }
            
            this.nodePublisher.set(this.gui.getCurSelected());
        }
    }
}
