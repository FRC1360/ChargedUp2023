package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArm;
import com.revrobotics.CANSparkMax.IdleMode;

public class Pivot extends Commandbase {
    private final CANSparkMax pivot;
    private int pivotSteps;

    public Pivot() {
        pivot = PivotArm.getPivotMotor();
    }

    public void turn(int degrees) {
        pivotSteps = PivotArm.getStepsfromDegrees(degrees);
        pivot.setSpeed(0.25);

        if (pivotArm.getPivotEncoder() >= pivotSteps) {
            pivot.setSpeed(0);
        }
    }

    public void setZero() {
        pivotArm.getPivotEncoder().getEncoder().setPosition(0.0);
    }

}
