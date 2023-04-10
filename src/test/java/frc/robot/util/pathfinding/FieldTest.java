package frc.robot.util.pathfinding;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import frc.robot.util.pathfinding.constraints.IFieldConstraints;
import frc.robot.util.pathfinding.constraints.PointFieldConstraint;
import frc.robot.util.pathfinding.constraints.RectangularFieldConstraint;

public class FieldTest {

    @Test
    void testGetExactNode() {
        PointFieldConstraint constraint = new PointFieldConstraint(0.5, 0.5);

        Field field = new Field(0.1, 1.0, 1.0, constraint);

        Node actual = field.getClosestNode(0.5, 0.5);

        assertEquals(false, actual.getIsValid());
    }

    @Test
    void testGetCloseNode() {
        PointFieldConstraint constraint = new PointFieldConstraint(0.5, 0.5);

        Field field = new Field(0.1, 1.0, 1.0, constraint);

        Node actual = field.getClosestNode(0.51, 0.49);
        assertEquals(false, actual.getIsValid());

        actual = field.getClosestNode(0.55, 0.49);
        assertEquals(true, actual.getIsValid());
    }

    //@Test
    void testGetAdjacentNodes() {
        Field field = new Field(0.1, 1.0, 1.0);

        Node homeNode = new Node(0.5, 0.5);

        int homeX = 5;
        int homeY = 5;

        homeNode.setCantorX(homeX);
        homeNode.setCantorY(homeY);

        ArrayList<Node> nodes = field.getAdjacentNodes(homeNode);

        assertEquals(4, nodes.size());
        assertEquals(Node.cantorPairFunction(homeX+1, homeY), nodes.get(0).hashCode());
        assertEquals(Node.cantorPairFunction(homeX-1, homeY), nodes.get(1).hashCode());
        assertEquals(Node.cantorPairFunction(homeX, homeY+1), nodes.get(2).hashCode());
        assertEquals(Node.cantorPairFunction(homeX, homeY-1), nodes.get(3).hashCode());
    }

    //@Test
    void testGetAdjacentNodesCorner() {
        Field field = new Field(0.1, 1.0, 1.0);

        Node homeNode = new Node(0.5, 0.5);

        int homeX = 0;
        int homeY = 0;

        homeNode.setCantorX(homeX);
        homeNode.setCantorY(homeY);

        ArrayList<Node> nodes = field.getAdjacentNodes(homeNode);

        assertEquals(2, nodes.size());
        assertEquals(Node.cantorPairFunction(homeX+1, homeY), nodes.get(0).hashCode());
        assertEquals(Node.cantorPairFunction(homeX, homeY+1), nodes.get(1).hashCode());
    }

    //@Test
    void testGetAdjacentValidNodes() {
        PointFieldConstraint constraint = new PointFieldConstraint(0.4, 0.5);
        Field field = new Field(0.1, 1.0, 1.0, constraint);

        Node homeNode = new Node(0.5, 0.5);

        int homeX = 5;
        int homeY = 5;

        homeNode.setCantorX(homeX);
        homeNode.setCantorY(homeY);

        ArrayList<Node> nodes = field.getAdjacentValidNodes(homeNode);

        assertEquals(3, nodes.size());
        assertEquals(Node.cantorPairFunction(homeX+1, homeY), nodes.get(0).hashCode());
        assertEquals(Node.cantorPairFunction(homeX, homeY+1), nodes.get(1).hashCode());
        assertEquals(Node.cantorPairFunction(homeX, homeY-1), nodes.get(2).hashCode());
    }

    //@Test
    void testWaypointGeneration() {
        Field field = new Field(0.1, 1.0, 1.0);

        Node start = field.getClosestNode(0.0, 0.0);
        Node end = field.getClosestNode(0.5, 0.5);

        ArrayList<AStarNode> path = field.generateWaypoints(start, end);

        for (AStarNode node : path) {
            System.out.println("Waypoint at (" + node.getX() + ", " + node.getY() + ")");
        }

        assertEquals(start, (Node)path.get(0));
        assertEquals(end, (Node)path.get(path.size()-1));
    }

    //@Test
    void testWaypointGenerationWithConstraint() {
        IFieldConstraints constraint = new RectangularFieldConstraint(0.2, 0.1, 0.4, 0.4);
        Field field = new Field(0.1, 1.0, 1.0, constraint);

        Node start = field.getClosestNode(0.0, 0.0);
        Node end = field.getClosestNode(0.5, 0.5);

        ArrayList<AStarNode> path = field.generateWaypoints(start, end);

        for (AStarNode node : path) {
            System.out.println("Waypoint at (" + node.getX() + ", " + node.getY() + ")");
            assertEquals(true, node.getIsValid());
        }

        assertEquals(start, (Node)path.get(0));
        assertEquals(end, (Node)path.get(path.size()-1));   
    }

    //@Test
    void testWaypointGenerationNoValidPath() {
        IFieldConstraints constraint = new RectangularFieldConstraint(0.2, 0.0, 0.4, 0.5);
        Field field = new Field(0.1, 0.6, 0.6, constraint);

        Node start = field.getClosestNode(0.0, 0.0);
        Node end = field.getClosestNode(0.5, 0.5);

        ArrayList<AStarNode> path = field.generateWaypoints(start, end);

        assertEquals(null, path);  
    }

    // TODO - Add behaviour for starting in invalid node
}
