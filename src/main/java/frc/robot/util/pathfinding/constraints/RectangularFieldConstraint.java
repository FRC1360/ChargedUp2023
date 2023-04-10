package frc.robot.util.pathfinding.constraints;

import frc.robot.util.pathfinding.Node;

public class RectangularFieldConstraint implements IFieldConstraints{

    private double xMin;
    private double xMax;
    private double yMin;
    private double yMax;
    
    public RectangularFieldConstraint(double xMin, double yMin, double xMax, double yMax) {
        this.xMin = xMin;
        this.xMax = xMax;
        this.yMin = yMin;
        this.yMax = yMax;
    }

    @Override
    public boolean nodeInFieldConstraint(Node node) {

        if(node.getX() >= this.xMin && node.getX() <= this.xMax && node.getY() >= this.yMin && node.getY() <= this.yMax) {
            return true;
        }

        return false;
    }
    
}
