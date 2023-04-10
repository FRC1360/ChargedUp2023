package frc.robot.util.pathfinding.constraints;

import frc.robot.util.pathfinding.Node;

public class CircularFieldConstraint implements IFieldConstraints {

    private double x;
    private double y;
    private double radius;

    public CircularFieldConstraint(double x, double y, double radius) {
        this.x = x;
        this.y = y;
        this.radius = radius;
    }

    @Override
    public boolean nodeInFieldConstraint(Node node) {
        double xDif = node.getX() - this.x;
        double yDif = node.getY() - this.y;

        double dist = Math.sqrt(Math.pow(xDif, 2) + Math.pow(yDif, 2));

        return dist <= this.radius;
    }
    
}
