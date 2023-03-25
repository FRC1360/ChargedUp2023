package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmGoToPositionCommand;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.assembly.AssemblyHomePositionCommand;
import frc.robot.commands.intake.AutoIntakeCommand;
import frc.robot.commands.intake.AutoPutDownCommand;
import frc.robot.commands.shoulder.ShoulderGoToPositionCommand;
import frc.robot.commands.shoulder.ShoulderHoldCommand;
import frc.robot.commands.wrist.WristGoToPositionCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;

public class SysCheck extends SequentialCommandGroup {

    public SysCheck(ShoulderSubsystem shoulder, ArmSubsystem arm, WristSubsystem wrist, IntakeSubsystem intake, ShoulderWristMessenger shoulderWristMessenger, Trigger proceed) { 

        addCommands(
            new ShoulderGoToPositionCommand(shoulder, 0) // raise arm
                .raceWith(new WristHoldCommand(wrist))
                .raceWith(new ArmHoldCommand(arm)),

            new ArmHoldCommand(arm)
                .alongWith(new WristHoldCommand(wrist))
                .alongWith(new ShoulderHoldCommand(shoulder, () -> 0.0))
                .until(proceed),

            new ShoulderGoToPositionCommand(shoulder, -45) // lower to 45
                .raceWith(new WristHoldCommand(wrist))
                .raceWith(new ArmHoldCommand(arm)),

            new ArmHoldCommand(arm)
                .alongWith(new WristHoldCommand(wrist))
                .alongWith(new ShoulderHoldCommand(shoulder, () -> 0.0))
                .until(proceed),

            new ArmGoToPositionCommand(arm, shoulderWristMessenger, 10.0) // extend arm
                .raceWith(new ShoulderHoldCommand(shoulder, () -> 0.0))
                .raceWith(new WristHoldCommand(wrist)),
            
            new ArmHoldCommand(arm)
                .alongWith(new WristHoldCommand(wrist))
                .alongWith(new ShoulderHoldCommand(shoulder, () -> 0.0))
                .until(proceed),

            new ArmGoToPositionCommand(arm, shoulderWristMessenger, 0.0) // retract arm
                .raceWith(new ShoulderHoldCommand(shoulder, () -> 0.0))
                .raceWith(new WristHoldCommand(wrist)),
            
            new ArmHoldCommand(arm)
                .alongWith(new WristHoldCommand(wrist))
                .alongWith(new ShoulderHoldCommand(shoulder, () -> 0.0))
                .until(proceed),

            new WristGoToPositionCommand(wrist, 90)
                .raceWith(new ShoulderHoldCommand(shoulder, () -> 0.0)) // wrist up
                .raceWith(new ArmHoldCommand(arm)),

            new ArmHoldCommand(arm)
                .alongWith(new WristHoldCommand(wrist))
                .alongWith(new ShoulderHoldCommand(shoulder, () -> 0.0))
                .until(proceed),

            new WristGoToPositionCommand(wrist, -90)
                .raceWith(new ShoulderHoldCommand(shoulder, () -> 0.0)) // wrist down
                .raceWith(new ArmHoldCommand(arm)),

            new ArmHoldCommand(arm)
                .alongWith(new WristHoldCommand(wrist))
                .alongWith(new ShoulderHoldCommand(shoulder, () -> 0.0)) // intake at full speed
                .alongWith(new AutoIntakeCommand(intake, 1))
                .until(proceed),

            new ArmHoldCommand(arm)
                .alongWith(new WristHoldCommand(wrist))
                .alongWith(new ShoulderHoldCommand(shoulder, () -> 0.0)) // drop at full speed
                .alongWith(new AutoPutDownCommand(intake, 1))
                .until(proceed),

            new AssemblyHomePositionCommand(shoulder, shoulderWristMessenger, wrist, arm)
        );
    }
}
