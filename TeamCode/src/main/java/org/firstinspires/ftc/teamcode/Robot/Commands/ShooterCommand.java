package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Susbsystems.Shooter;

public class ShooterCommand extends ParallelCommandGroup {

    public ShooterCommand(Robot robot, Shooter.ShooterStates state){
        addCommands(
                new InstantCommand(() -> robot.shooter.setState(state), robot.shooter)
        );
    }

}
