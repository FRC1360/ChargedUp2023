package frc.robot.autos.procedures;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.basic.LockWheels;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.assembly.AssemblyHighScoreCommand;
import frc.robot.commands.assembly.AssemblyHomePositionCommand;
import frc.robot.commands.intake.AutoIntakeCommand;
import frc.robot.commands.intake.AutoPutDownCommand;
import frc.robot.commands.intake.IntakeHoldCommand;
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

public class CubeScoreHighAuto extends SequentialCommandGroup {

    public CubeScoreHighAuto(SwerveSubsystem dt, ShoulderSubsystem shoulder,
            ShoulderWristMessenger shoulderWristMessenger,
            WristSubsystem wrist, ArmSubsystem arm, IntakeSubsystem intake,
            ArmShoulderMessenger armMessenger, LEDSubsystem ledSubsystem, StateMachine sm) {

        addCommands(

                new AutoIntakeCommand(intake, 0.25)
                        .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                        .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                        .raceWith(new ArmHoldCommand(arm)),

                new AssemblyHighScoreCommand(shoulder, shoulderWristMessenger, wrist, arm, armMessenger, () -> true,
                        ledSubsystem, sm)
                        .raceWith(new LockWheels(dt))
                        .raceWith(new IntakeHoldCommand(intake)),

                new AutoPutDownCommand(intake, 0.8)
                        .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                        .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                        .raceWith(new ArmHoldCommand(arm)),

                new AssemblyHomePositionCommand(shoulder, shoulderWristMessenger, wrist, arm, armMessenger,
                        ledSubsystem, sm));
    }
}
