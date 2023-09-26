package frc.robot.autos.procedures;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.assembly.AssemblyHomePositionCommand;
import frc.robot.commands.assembly.autoAssembly.AutoAssemblyConeHighScoreCommand;
import frc.robot.commands.shoulder.ShoulderHoldCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;
import frc.robot.subsystems.SwerveDrive.DrivetrainSubsystem;
import frc.robot.util.StateMachine;
import frc.robot.subsystems.WristSubsystem;

public class ConeScoreHighAuto extends SequentialCommandGroup {

    public ConeScoreHighAuto(SwerveSubsystem dt, ShoulderSubsystem shoulder,
            ShoulderWristMessenger shoulderWristMessenger,
            WristSubsystem wrist, ArmSubsystem arm, IntakeSubsystem intake,
            ArmShoulderMessenger armMessenger, LEDSubsystem ledSubsystem, StateMachine sm) {
        addCommands(
                new AutoAssemblyConeHighScoreCommand(dt, shoulder, shoulderWristMessenger, wrist,
                        arm, intake, ledSubsystem, armMessenger, sm),
                new AssemblyHomePositionCommand(shoulder, shoulderWristMessenger,
                        wrist, arm, armMessenger, ledSubsystem, sm));
    }
}
