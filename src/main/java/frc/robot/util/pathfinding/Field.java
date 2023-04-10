package frc.robot.util.pathfinding;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Objects;
import java.util.PriorityQueue;

import frc.robot.util.pathfinding.constraints.IFieldConstraints;

public class Field {

    //public static final double FIELD_X_METERS = 8.1026;  // Width of FRC Field
    //public static final double FIELD_Y_METERS = 16.4846;  // Lenght of FRC Field

    private final double FIELD_X_METERS;  // Width of FRC Field
    private final double FIELD_Y_METERS;  // Lenght of FRC Field
    
    private double resolution;
    private HashMap<Integer, Node> nodes;

    /**
     * 
     * @param resolution - Generates waypoints every x meters (For example, resolution of 0.1 will generate waypoints every 0.1m in x and y direction)
     * @param constraints - Constraints for invalid nodes for pathfinding
     */
    public Field(double resolution, IFieldConstraints... constraints) {
        this(resolution, 8.1026, 16.4846, constraints);
    }

    protected Field(double resolution, double FIELD_X_METERS, double FIELD_Y_METERS, IFieldConstraints... constraints) {
        this.resolution = resolution;
        this.FIELD_X_METERS = FIELD_X_METERS;
        this.FIELD_Y_METERS = FIELD_Y_METERS;

        this.nodes = new HashMap<>();

        int cantorX = 0;
        int cantorY = 0;

        for(double i = 0; i <= this.FIELD_X_METERS; i += resolution) {
            cantorY = 0;

            for(double j = 0; j < this.FIELD_Y_METERS; j += resolution) {
                Node node = new Node(i, j);

                for (IFieldConstraints iFieldConstraint : constraints) {
                    node.setIsValid(!iFieldConstraint.nodeInFieldConstraint(node));
                }

                node.setCantorX(cantorX);
                node.setCantorY(cantorY);
                //System.out.println("X = " + cantorX + ", Y = " + cantorY + ", Hash = " + node.hashCode());
                nodes.put(node.hashCode(), node);

                cantorY++;
            }

            cantorX++;
        }
    }

    public Node getClosestNode(double x, double y) {
        int cantorX = (int)Math.round(x / resolution);
        int cantorY = (int)Math.round(y / resolution);
        return nodes.get(Node.cantorPairFunction(cantorX, cantorY));
    }

    public Node getClosestNode(Node node) {
        return getClosestNode(node.getX(), node.getY());
    }

    public ArrayList<Node> getAdjacentNodes(Node node) {
        ArrayList<Node> adjacentNodes = new ArrayList<>();

        int x = node.getCantorX();
        int y = node.getCantorY();

        adjacentNodes.add(nodes.get(Node.cantorPairFunction(x+1, y))); // No need to guard as it will return null if not found

        if(!(x-1 < 0))
            adjacentNodes.add(nodes.get(Node.cantorPairFunction(x-1, y)));  // Need to guard as Cantor Pair only works for natural numbers

        adjacentNodes.add(nodes.get(Node.cantorPairFunction(x, y+1)));  // No need to guard as it will return null if not found

        if(!(y-1 < 0))
            adjacentNodes.add(nodes.get(Node.cantorPairFunction(x, y-1)));  // Need to guard as Cantor Pair only works for natural numbers

        adjacentNodes.removeIf(Objects::isNull);

        return adjacentNodes;
    }

    public ArrayList<Node> getAdjacentValidNodes(Node node) {
        ArrayList<Node> adjacentNodes = getAdjacentNodes(node);

        adjacentNodes.removeIf(n -> !n.getIsValid());

        return adjacentNodes;
    }

    /**
     * Uses A* pathfinding to generate waypoints for Trajectory Generation
     */
    public ArrayList<AStarNode> generateWaypoints(Node start, Node end) {
        AStarNode pathfindingStart = AStarNode.createFromNode(getClosestNode(start));
        AStarNode pathfindingEnd = AStarNode.createFromNode(getClosestNode(end));

        PriorityQueue<AStarNode> openSet = new PriorityQueue<>();
        HashMap<AStarNode, Double> gScores = new HashMap<>();
        HashMap<AStarNode, AStarNode> cameFrom = new HashMap<>();

        pathfindingStart.setFScore(getHScore(pathfindingStart, pathfindingEnd));

        gScores.put(pathfindingStart, 0.0);
        openSet.add(pathfindingStart);

        while(!openSet.isEmpty()) {
            AStarNode current = openSet.poll();

            //System.out.println("Current is (" + current.getX() + ", " + current.getY() + ")");

            if(current.hashCode() == end.hashCode()) {
                // DONE
                //System.out.println("Found Path");
                ArrayList<AStarNode> path = new ArrayList<>();

                AStarNode currentPathNode = current;

                path.add(currentPathNode);

                while(cameFrom.containsKey(currentPathNode)) {
                    currentPathNode = cameFrom.get(currentPathNode);
                    //System.out.println("Found current path node");
                    path.add(currentPathNode);
                }

                Collections.reverse(path);

                return path;

            }

            ArrayList<Node> neighbours = getAdjacentValidNodes(current);

            for (Node neighbour : neighbours) {
                double gScore = gScores.get(current) + resolution;  // Should probably change this to distance between points, but for now resolution works because waypoint distances are constant
                AStarNode aStarNeighbour = AStarNode.createFromNode(neighbour);

                if(gScore < gScores.getOrDefault(aStarNeighbour, Double.POSITIVE_INFINITY)) {
                    //System.out.println("Waypoint at (" + aStarNeighbour.getX() + ", " + aStarNeighbour.getY() + ") with gScore of " + gScores.getOrDefault(aStarNeighbour, Double.POSITIVE_INFINITY) 
                    //+ " came from (" + current.getX() + ", " + current.getY() + ") with gScore of " + gScore);

                    cameFrom.put(aStarNeighbour, current);
                    gScores.put(aStarNeighbour, gScore);

                    double hScore = getHScore(neighbour, pathfindingEnd);
                    aStarNeighbour.setFScore(gScore + hScore);

                    if(!openSet.contains(aStarNeighbour)) {
                        openSet.add(aStarNeighbour);
                    }
                }
            }

            //System.out.println("-------------------");

            //return null;
        }

        // Could not find path
        //System.out.println("Path not found");
        return null;
    }

    private double getHScore(Node n1, Node n2) {
        return Math.sqrt(Math.pow(n1.getX() - n2.getX(), 2) + Math.pow(n1.getY() - n2.getY(), 2));
    }
    
}
