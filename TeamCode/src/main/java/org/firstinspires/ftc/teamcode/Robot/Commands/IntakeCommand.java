package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;

public class IntakeCommand extends ParallelCommandGroup {

    public IntakeCommand(Robot robot, Intake.IntakeStates state){
        addCommands(
                new InstantCommand(() -> robot.intake.setState(state), robot.intake)
        );
    }

    public IntakeCommand(Robot robot, Intake.RampStates state){
        addCommands(
                new InstantCommand(() -> robot.intake.setState(state), robot.intake)
        );

    }

}
