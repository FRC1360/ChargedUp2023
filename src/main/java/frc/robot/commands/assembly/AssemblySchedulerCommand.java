package frc.robot.commands.assembly;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;
import frc.robot.util.StateMachine;

public class AssemblySchedulerCommand extends CommandBase {

    public static enum ASSEMBLY_LEVEL { // the set points that can be set by operator
        MEDIUM_CONE,
        MEDIUM_CUBE,
        HIGH_CONE,
        HIGH_CUBE;
    }

    private Command assemblyCommand;
    private Supplier<ASSEMBLY_LEVEL> level;

    private ShoulderSubsystem shoulder;
    private WristSubsystem wrist;
    private ArmSubsystem arm;
    private ArmShoulderMessenger armMessenger;
    private ShoulderWristMessenger shoulderMessenger;
    private LEDSubsystem led;
    private StateMachine sm;
    private int n = 0;

    public AssemblySchedulerCommand(Supplier<ASSEMBLY_LEVEL> level, ShoulderSubsystem shoulder, WristSubsystem wrist, ArmSubsystem arm,
        ArmShoulderMessenger armMessenger, ShoulderWristMessenger shoulderMessenger, LEDSubsystem led, StateMachine sm) {
        this.level = level;
        this.shoulder = shoulder;
        this.wrist = wrist;
        this.arm = arm;
        this.shoulderMessenger = shoulderMessenger;
        this.armMessenger = armMessenger;
        this.led = led;
        this.sm = sm;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("AssSchedCmdN", n++);
        switch(level.get()) {
            case MEDIUM_CONE:
                this.assemblyCommand = new AssemblyMidScoreCommand(shoulder, shoulderMessenger, wrist, arm, armMessenger, led, () -> false, sm);
                break;
            
            case MEDIUM_CUBE:
                this.assemblyCommand = new AssemblyMidScoreCommand(shoulder, shoulderMessenger, wrist, arm, armMessenger, led, () -> true, sm);
                break;
            
            case HIGH_CONE:
                this.assemblyCommand = new AssemblyHighScoreCommand(shoulder, shoulderMessenger, wrist, arm, armMessenger, () -> false, led, sm);
                break;
            
            case HIGH_CUBE:
                this.assemblyCommand = new AssemblyHighScoreCommand(shoulder, shoulderMessenger, wrist, arm, armMessenger, () -> true, led, sm);
                break;

            default:
                break;
        }

        this.assemblyCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return this.assemblyCommand.isFinished();
    }
    
    
}
