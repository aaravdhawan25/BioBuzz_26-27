package org.firstinspires.ftc.teamcode.Robot.Commands;



import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class ShooterCommand extends ParallelCommandGroup {

    public ShooterCommand(Robot robot, Shooter.ShooterStates state){
        addCommands(
                new InstantCommand(() -> robot.shooter.setState(state), robot.shooter)
        );
    }

}
