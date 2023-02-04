package frc.robot.commands.assembly;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shoulder.ShoulderGoToPositionCommand;
import frc.robot.commands.wrist.WristGoToCachePositionCommand;
import frc.robot.commands.wrist.WristGoToPositionCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AssemblyGoToPositionCommand extends SequentialCommandGroup {

    public AssemblyGoToPositionCommand(ShoulderSubsystem shoulder, WristSubsystem wrist, double angle) {    
        addCommands(new InstantCommand(() -> shoulder.setScheduledAngle(angle)),  // Set scheduled angle - scheduled angle is used to check if transitioning as targetAngle() is set during GoTo commands, which happens after descision
            new InstantCommand(() -> shoulder.checkTransitioning()),  // Manually check if we are transitioning
            new ScheduleCommand(  // WPILib work around for ensuring shoulder.inTransitionState() shows the current transition state
                new ConditionalCommand(

                    // Transition = True
                    new InstantCommand(() -> wrist.setCacheOffset())  // Save the current targetAngle
                    .andThen(new WristGoToPositionCommand(wrist, 90))  // Got to 90 degrees
                    .andThen(
                        new ShoulderGoToPositionCommand(shoulder, angle)  // Set shoulder angle
                        .raceWith(new WristHoldCommand(wrist))  // Manually call WristHoldCommand() because conditional commands require all subsystems in all subcommands
                    )
                    .andThen(new ScheduleCommand(  // WPILib workaround to ensure WristGoToCachePosition initialize is called the right time
                        new WristGoToCachePositionCommand(wrist)
                    )),  

                    // Transition = False
                    new ShoulderGoToPositionCommand(shoulder, angle)  // Set shoulder angle
                    .raceWith(new WristHoldCommand(wrist)), // Manually call WristHoldCommand() because conditional commands require all subsystems in all subcommands
                    
                    shoulder.inTransitionState())
            ));

    }

    
    
}
