package frc.robot.util.pathfinding;

public class Node {

    private double x;  // "Real-life" X position on field
    private double y;  // "Real-life" Y position on field
    private boolean isValid;

    private int cantorX;  // Used for hashing
    private int cantorY;  // Used for hashing

    public Node(double x, double y) {
        this(x, y, true);
    }

    public Node(double x, double y, boolean isValid) {
        this.x = x;
        this.y = y;
        this.isValid = isValid;
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public boolean getIsValid() {
        return this.isValid;
    }

    public void setIsValid(boolean isValid) {
        this.isValid = isValid;
    }

    protected int getCantorX () {
        return this.cantorX;
    }

    protected void setCantorX(int x) {
        this.cantorX = x;
    }

    protected int getCantorY() { 
        return this.cantorY;
    }

    protected void setCantorY(int y) {
        this.cantorY = y;
    }

    public static int cantorPairFunction(int x, int y) {
        return ((int)(0.5 * (x + y) * (x + y + 1))) + y;
    }

    @Override
    public int hashCode() {
        return cantorPairFunction(cantorX, cantorY);
    }

    @Override
    public boolean equals(Object obj) {
        Node node;

        try {
            node = (Node)obj;
        } catch(Exception e) {
            return false;
        }

        if(this.isValid != node.isValid) return false;
        if(this.x != node.x) return false;
        if(this.y != node.y) return false;
        if(this.cantorX != node.cantorX) return false;
        if(this.cantorY != node.cantorY) return false;

        return true;
    }

}
