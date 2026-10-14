package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Commands.BlockerCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.FollowPathCommand;
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
import org.firstinspires.ftc.teamcode.Utils.AutoUtils.AutoConstants;
import org.firstinspires.ftc.teamcode.Utils.AutoUtils.AutoPoseData;

public class AutoOutline extends OpMode {

    Robot robot;
    String color;
    SequentialCommandGroup autonomous;

    private AutoPaths paths;

    public AutoOutline(String color){
        this.color = color;
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, color, true);
        paths = new AutoPaths(color);
        CommandScheduler.getInstance().schedule(
                new BlockerCommand(robot, Blocker.BlockerState.CLOSED),
                new TurretCommand(robot, Turret.TurretState.FORWARD),
                new IntakeCommand(robot, Intake.RampStates.RETRACTED)
        );

        autonomous = new SequentialCommandGroup(
                shootFour(),
                new InstantCommand(robot::flipGoal),
                intake(paths.startToGarden()),
                new FollowPathCommand(robot.follower, paths.gardenToLaunch()),
                shootFour(),
                intakeFromFlower(paths.launchToFlower()),
                shootFour(),
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, paths.flowerToPark()),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new IntakeCommand(robot, Intake.RampStates.RETRACTED)
                        )
                )

        );


    }

    @Override
    public void start(){
        CommandScheduler.getInstance().schedule(
                autonomous
        );
    }

    @Override
    public void loop() {
        robot.update();
    }

    public SequentialCommandGroup shootFour(){
        return new SequentialCommandGroup(
                new TurretCommand(robot, Turret.TurretState.MATH),
                new ShooterCommand(robot, Shooter.ShooterStates.ON),
                new TransferCommand(robot),
                new WaitCommand(500),
                new TransferCancelCommand(robot)
        );
    }

    public ParallelCommandGroup intake(Path path){
        return new ParallelCommandGroup(
                new FollowPathCommand(robot.follower, path),
                new SequentialCommandGroup(
                        new WaitCommand(800),
                        new IntakeCommand(robot, Intake.IntakeStates.ON),
                        new WaitUntilCommand(() -> !robot.follower.isBusy()),
                        new WaitCommand(600),
                        new IntakeCommand(robot, Intake.IntakeStates.OFF)

                )
        );
    }

    public ParallelCommandGroup intakeFromFlower(Path path){
        return new ParallelCommandGroup(
                new FollowPathCommand(robot.follower, path),
                new SequentialCommandGroup(
                        new WaitCommand(800),
                        new IntakeCommand(robot, Intake.IntakeStates.ON),
                        new IntakeCommand(robot, Intake.RampStates.DEPLOYED),
                        new WaitUntilCommand(() -> !robot.follower.isBusy()),
                        new WaitCommand(600),
                        new IntakeCommand(robot, Intake.IntakeStates.OFF)
                )
        );
    }


    public class AutoPaths {

        PoseFactory poseFactory = PoseFactory.degrees();

        Pose startPose;
        Pose gardenPollenPose;
        Pose startToGardenControl;
        Pose gardenTurnSeg1Start;
        Pose gardenTurnSeg1End;
        Pose gardenTurnSeg2Start;
        Pose gardenTurnSeg2End;
        Pose hiveLaunchPose;
        Pose gardenToLaunchControl1;
        Pose gardenToLaunchControl2;
        Pose launchTurnStart;
        Pose launchTurnEnd;
        Pose launchDepartPose;
        Pose flowerPose;
        Pose launchToFlowerControl;
        Pose loadingZoneParkPose;
        Pose flowerToParkControl1;
        Pose flowerToParkControl2;

        public AutoPaths(String color) {
            startPose = pose(AutoPoseData.startPose, color);

            gardenPollenPose = pose(AutoPoseData.gardenPollenPose, color);
            startToGardenControl = pose(AutoPoseData.startToGardenControl, color);
            gardenTurnSeg1Start = pose(AutoPoseData.gardenTurnSeg1Start, color);
            gardenTurnSeg1End = pose(AutoPoseData.gardenTurnSeg1End, color);
            gardenTurnSeg2Start = pose(AutoPoseData.gardenTurnSeg2Start, color);
            gardenTurnSeg2End = pose(AutoPoseData.gardenTurnSeg2End, color);

            hiveLaunchPose = pose(AutoPoseData.hiveLaunchPose, color);
            gardenToLaunchControl1 = pose(AutoPoseData.gardenToLaunchControl1, color);
            gardenToLaunchControl2 = pose(AutoPoseData.gardenToLaunchControl2, color);
            launchTurnStart = pose(AutoPoseData.launchTurnStart, color);
            launchTurnEnd = pose(AutoPoseData.launchTurnEnd, color);

            launchDepartPose = pose(AutoPoseData.launchDepartPose, color);
            flowerPose = pose(AutoPoseData.flowerPose, color);
            launchToFlowerControl = pose(AutoPoseData.launchToFlowerControl, color);

            loadingZoneParkPose = pose(AutoPoseData.loadingZoneParkPose, color);
            flowerToParkControl1 = pose(AutoPoseData.flowerToParkControl1, color);
            flowerToParkControl2 = pose(AutoPoseData.flowerToParkControl2, color);

            robot.follower.setPose(startPose);
        }

        private Pose pose(Pose pose, String color) {
            return poseFactory.of(
                    AutoPoseData.mirrorX(pose.x(), color),
                    AutoPoseData.mirrorY(pose.y(), color),
                    AutoPoseData.mirrorHeading(Math.toDegrees(pose.heading()), color));
        }

        public Path startToGarden() {
            return Paths.curve(startPose, startToGardenControl, gardenPollenPose)
                    .heading(Interpolator.piecewise()
                            .until(AutoConstants.gardenTurnSplit, Interpolator.linear(gardenTurnSeg1Start, gardenTurnSeg1End))
                            .until(AutoConstants.gardenEndTValue, Interpolator.linear(gardenTurnSeg2Start, gardenTurnSeg2End)));
        }

        public Path gardenToLaunch() {
            return Paths.curve(gardenPollenPose, gardenToLaunchControl1, gardenToLaunchControl2, hiveLaunchPose)
                    .heading(Interpolator.piecewise()
                            .until(AutoConstants.launchTurnSplit, Interpolator.linear(launchTurnStart, launchTurnEnd))
                            .until(AutoConstants.launchEndTValue, Interpolator.tangent));
        }

        public Path launchToFlower() {
            return Paths.curve(launchDepartPose, launchToFlowerControl, flowerPose).linear(launchDepartPose, flowerPose);
        }

        public Path flowerToPark() {
            return Paths.curve(flowerPose, flowerToParkControl1, flowerToParkControl2, loadingZoneParkPose).reverseTangent();
        }

        public Pose getStartPose() {
            return startPose;
        }

    }

}
