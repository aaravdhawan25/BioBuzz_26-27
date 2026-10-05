package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Blocker;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;

public class TransferCommand extends SequentialCommandGroup {

    public TransferCommand(Robot robot){
        addCommands(
                new ParallelRaceGroup(
                        new WaitUntilCommand(
                                () -> robot.shooter.atTargetSpeed()
                        ),
                        new WaitCommand(2000)
                ),
                new BlockerCommand(robot, Blocker.BlockerState.OPEN),
                new WaitCommand(200),
                new IntakeCommand(robot, Intake.IntakeStates.ON)
        );
    }

}
