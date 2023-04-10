package frc.robot.util.pathfinding.constraints;

import frc.robot.util.pathfinding.Node;

public class PointFieldConstraint implements IFieldConstraints {

    double x;
    double y;

    public PointFieldConstraint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public boolean nodeInFieldConstraint(Node node) {
        return (node.getX() == this.x) && (node.getY() == this.y);
    }
    
}
