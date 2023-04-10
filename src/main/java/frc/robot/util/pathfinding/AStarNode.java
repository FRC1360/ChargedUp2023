package frc.robot.util.pathfinding;

public class AStarNode extends Node implements Comparable<AStarNode>{

    double fScore;

    protected AStarNode(double x, double y) {
        super(x, y);
    }

    protected double getFScore() {
        return fScore;
    }

    protected void setFScore(double fScore) {
        this.fScore = fScore;
    }

    protected static AStarNode createFromNode(Node node) {
        AStarNode a = new AStarNode(node.getX(), node.getY());
        a.setCantorX(node.getCantorX());
        a.setCantorY(node.getCantorY());
        a.setIsValid(node.getIsValid());

        return a;
    }

    @Override
    public int compareTo(AStarNode arg0) {
        if(this.fScore > arg0.fScore) {
            return 1;
        } else if (this.fScore < arg0.fScore) {
            return -1;
        } else {
            return 0;
        }
    }

    @Override
    public int hashCode() {
        return super.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        AStarNode node;

        try {
            node = (AStarNode)obj;
        } catch(Exception e) {
            return false;
        }

        return this.hashCode() == node.hashCode();
    }
}
