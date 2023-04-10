package frc.robot.util.pathfinding;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class AStarNodeTest {
    

    @Test
    void testAStarNodeCompare() {
        AStarNode node1 = new AStarNode(0.5, 0.5);
        AStarNode node2 = new AStarNode(0.5, 0.5);

        node1.fScore = 0.123;
        node2.fScore = 0.123;
        assertEquals(0, node1.compareTo(node2));

        node1.fScore = 0.1;
        assertEquals(-1, node1.compareTo(node2));

        node1.fScore = 0.2;
        assertEquals(1, node1.compareTo(node2));
    }

    @Test
    void testAStarNodeFromNode() {
        Node parent = new Node(0.5, 1.5, true);
        parent.setCantorX(5);
        parent.setCantorY(15);

        AStarNode child = AStarNode.createFromNode(parent);
        child.setFScore(0.123);

        assertEquals(0.5, child.getX());
        assertEquals(1.5, child.getY());
        assertEquals(5, child.getCantorX());
        assertEquals(15, child.getCantorY());
        assertEquals(true, child.getIsValid());
        assertEquals(parent.hashCode(), child.hashCode());
        assertEquals(0.123, child.getFScore());
    }

}
