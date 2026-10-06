package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Commands.BlockerCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.TransferCancelCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.TransferCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.TurretCommand;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Blocker;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

@TeleOp(name = "TeleOp Red")
public class TeleopRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, "RED", false);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new TurretCommand(robot, Turret.TurretState.MATH)
        );
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenReleased(
                new TurretCommand(robot, Turret.TurretState.FORWARD)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(robot::resetPosition)
        );

        gp2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(robot::flipGoal)
        );
        gp2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ParallelCommandGroup(
                        new ShooterCommand(robot, Shooter.ShooterStates.ON),
                        new TransferCommand(robot)
                )
        );
        gp2.getGamepadButton(GamepadKeys.Button.Y).whenReleased(
                new TransferCancelCommand(robot)
        );
        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new IntakeCommand(robot, Intake.IntakeStates.ON)
        );
        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                new IntakeCommand(robot, Intake.IntakeStates.OFF)
        );
        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new IntakeCommand(robot, Intake.IntakeStates.REVERSE)
        );
        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
                new IntakeCommand(robot, Intake.IntakeStates.OFF)
        );

        CommandScheduler.getInstance().schedule(
                new BlockerCommand(robot, Blocker.BlockerState.CLOSED),
                new TurretCommand(robot, Turret.TurretState.FORWARD)
        );

        while (opModeInInit()){

        }

        waitForStart();

        if (isStopRequested()){
            robot.stop();
        }

        while (opModeIsActive()){
            ManualDrive.driveOrHold(
                    robot.follower,
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );
            robot.update();
        }

    }
}
