package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Blocker;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class TransferCancelCommand extends ParallelCommandGroup {

    public TransferCancelCommand(Robot robot){
        addCommands(
                new BlockerCommand(robot, Blocker.BlockerState.CLOSED),
                new IntakeCommand(robot, Intake.IntakeStates.OFF),
                new ShooterCommand(robot, Shooter.ShooterStates.OFF)
        );
    }

}
