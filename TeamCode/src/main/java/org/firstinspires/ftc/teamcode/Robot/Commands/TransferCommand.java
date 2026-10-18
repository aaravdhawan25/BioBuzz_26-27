package org.firstinspires.ftc.teamcode.Robot.Commands;



import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

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
