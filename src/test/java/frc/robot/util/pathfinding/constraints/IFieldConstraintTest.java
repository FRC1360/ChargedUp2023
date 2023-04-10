package frc.robot.util.pathfinding.constraints;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.util.pathfinding.Node;


public class IFieldConstraintTest {

    @Test
    void testPointConstraint() {
        Node node = new Node(0.5, 0.5);
        IFieldConstraints constraint = new PointFieldConstraint(0.5, 0.5);

        assertEquals(true, constraint.nodeInFieldConstraint(node));

        node = new Node(0.75, 0.75);

        assertEquals(false, constraint.nodeInFieldConstraint(node));
    }

    @Test
    void testRectangularConstraint() {
        Node node = new Node(0.2, 0.3);
        IFieldConstraints constraints = new RectangularFieldConstraint(0.0, 0.0, 0.5, 0.5);

        assertEquals(true, constraints.nodeInFieldConstraint(node));

        node = new Node(0.6, 0.6);

        assertEquals(false, constraints.nodeInFieldConstraint(node));
    }

    // TODO - Add tests for Circular and Rectangular FieldConstraints

}
