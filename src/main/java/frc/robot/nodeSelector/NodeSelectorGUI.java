package frc.robot.nodeSelector;

import javax.swing.JFrame; 
import javax.swing.JPanel;
import javax.swing.JButton;
import javax.swing.JLabel;
import java.awt.Container;
import java.awt.Component;
import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;
import java.awt.Font;
import java.awt.Dimension;
import java.awt.Color;
import java.awt.Insets; 
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;

public class NodeSelectorGUI {
    // RUN ON DRIVER STATION
    private String curSelected = "";
    private NodeButton prevSelected;
    JLabel nodeTextLabel; 

    public class SetSelectedNodeAction implements ActionListener {

        private NodeSelectorGUI gui;
        private NodeButton button;

        public SetSelectedNodeAction(NodeSelectorGUI gui, NodeButton button) {
            this.gui = gui;
            this.button = button;
        }

        @Override
        public void actionPerformed(ActionEvent e) {
            if (prevSelected != null) 
                this.gui.getPrevSelected().setBackground(new Color(10, 169, 209)); // BLUE
            this.gui.setCurSelected(this.button.getName());
            this.button.setBackground(new Color(0, 255, 0)); // GREEN
            this.gui.getNodeTextLabel().setText(this.gui.curSelected);
            this.gui.setPrevSelected(this.button);
        }
    }

    public class NodeButton extends JButton {

        private boolean isCube;
        private String name;

        private ActionListener nodeActionListener;
        private NodeSelectorGUI gui;

        public NodeButton(NodeSelectorGUI gui, boolean isCube, boolean isHybrid, int row, int col) {
            this.isCube = isCube;
            this.name = "";
            this.gui = gui;

            if (col == 1)
                this.name += "L";
            else if (col == 2)
                this.name += "M";
            else
                this.name += "H";

            this.name += String.format("%s", row);

            this.setText(this.name);

            if (isHybrid) { 
                this.setBackground(new Color(105, 105, 105)); // GREY
                this.setForeground(new Color(255, 255, 255)); // WHITE
            }
            else if (this.isCube) {
                this.setBackground(new Color(213, 38, 202)); // PURPLE
                this.setForeground(new Color(255, 255, 255)); // WHITE
            }
            else {
                this.setBackground(new Color(255, 230, 0)); // YELLOW 
                this.setForeground(new Color(0, 0, 0)); // BLACK
            }

            this.nodeActionListener = new SetSelectedNodeAction(this.gui, this);

            this.addActionListener(this.nodeActionListener);
        }

        public String getName() {
            return this.name;
        }
    }

    public static void generateCol(NodeSelectorGUI gui, JPanel panel, int rowNum) {
        int lpadx = 5;
        int rpadx = 5;
        if (rowNum == 1)
            lpadx = 0;
        else if (rowNum % 3 == 1)
            lpadx += 20;
        else if (rowNum == 9)
            rpadx = 0;

        boolean isCube = false;
        if (rowNum % 3 == 2)
            isCube = true;

        for (int y = 1; y <= 3; y++) {
            int tpady = 10;
            int bpady = 10;
            boolean isHybrid = false; 
            if (y == 1) { 
                tpady = 0;
                isHybrid = true; 
            }
            if (y == 3)
                bpady = 0;
            NodeButton button = gui.new NodeButton(gui, isCube, isHybrid, rowNum, y);
            button.setPreferredSize(new Dimension(100, 150));
            addComponent(panel, button, 200 * rowNum, 300 * y, 200, 300, lpadx, tpady, rpadx, bpady,
                    GridBagConstraints.CENTER, 0);
        }
    }

    private static void addComponent(Container container, Component component, int gridx, int gridy,
            int gridwidth, int gridheight, int lpadx, int tpady, int rpadx, int bpady, int anchor, int fill) {

        GridBagConstraints gbc = new GridBagConstraints(gridx, gridy, gridwidth, gridheight, 1.0, 1.0,
                anchor, fill, new Insets(tpady, lpadx, bpady, rpadx), 0, 0);

        container.add(component, gbc);
    }

    public void setCurSelected(String node) {
        this.curSelected = node;
    }

    public void setPrevSelected(NodeButton prevNode) { 
        this.prevSelected = prevNode; 
    }

    public JLabel getNodeTextLabel() { 
        return this.nodeTextLabel; 
    }

    public String getCurSelected() { 
        return this.curSelected; 
    }

    public NodeButton getPrevSelected() { 
        return this.prevSelected; 
    }

    public NodeSelectorGUI() {
        JFrame frame = new JFrame("Node Selector");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(1500, 1500);

        // cube8Button.setEnabled(false);

        JPanel panel = new JPanel(new GridBagLayout());
        for (int i = 1; i <= 9; i++) {
            generateCol(this, panel, i);
        }

        this.nodeTextLabel = new JLabel("None");
        this.nodeTextLabel.setPreferredSize(new Dimension(100, 50));
        this.nodeTextLabel.setFont(new Font("Arial", 1, 20)); 
        addComponent(panel, nodeTextLabel, 200, 1200, 200, 300, 0, 10, 10, 10, GridBagConstraints.CENTER, 0);
        // midPanel.setPreferredSize(new Dimension(100, 100));
        frame.add(panel);

        frame.pack();

        frame.setVisible(true);
    }

    public static void main(String[] args) throws IOException {

        NodeSelectorGUI gui = new NodeSelectorGUI();

        Thread serverThread = new Thread() { 
            
            NodeSelectorNTServer server = new NodeSelectorNTServer(gui); 

            @Override
            public void run() { 
                try {
                    server.run();
                } catch (IOException ex) { 
                    this.interrupt();
                }
            }
        }; 

        serverThread.start(); 

    }
}