package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class TurretCommand extends ParallelCommandGroup {

    public TurretCommand(Robot robot, Turret.TurretState state){
        addCommands(
                new InstantCommand(() -> robot.turret.setState(state))
        );
    }

}
