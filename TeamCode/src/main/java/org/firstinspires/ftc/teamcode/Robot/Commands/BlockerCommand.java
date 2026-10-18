package org.firstinspires.ftc.teamcode.Robot.Commands;


import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Blocker;

public class BlockerCommand extends ParallelCommandGroup {

    public BlockerCommand(Robot robot, Blocker.BlockerState state){
        addCommands(
                new InstantCommand(() -> robot.blocker.setState(state), robot.blocker)
        );
    }

}
