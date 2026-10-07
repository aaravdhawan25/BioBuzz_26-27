package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
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
                intake(paths.startToGarden()),
                new FollowPathCommand(robot.follower, paths.gardenToLaunch1()),
                shootFour(),
                intakeFromFlower(paths.launch1ToFlower()),
                new FollowPathCommand(robot.follower, paths.flowerToLaunch2()),
                new IntakeCommand(robot, Intake.RampStates.RETRACTED),
                shootFour(),
                new FollowPathCommand(robot.follower, paths.launch2ToPark())
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
        Pose hiveLaunchPose1;
        Pose gardenToLaunch1Control;
        Pose launch1TurnStart;
        Pose launch1TurnEnd;
        Pose flowerPose;
        Pose launch1ToFlowerControl;
        Pose flowerDepartPose;
        Pose hiveLaunchPose2;
        Pose flowerToLaunch2Control;
        Pose loadingZoneParkPose;
        Pose launch2ToParkControl1;
        Pose launch2ToParkControl2;

        public AutoPaths(String color) {
            startPose = pose(AutoPoseData.startPose, color);
            gardenPollenPose = pose(AutoPoseData.gardenPollenPose, color);
            gardenTurnSeg1Start = pose(AutoPoseData.gardenTurnSeg1Start, color);
            gardenTurnSeg1End = pose(AutoPoseData.gardenTurnSeg1End, color);
            gardenTurnSeg2Start = pose(AutoPoseData.gardenTurnSeg2Start, color);
            gardenTurnSeg2End = pose(AutoPoseData.gardenTurnSeg2End, color);

            hiveLaunchPose1 = pose(AutoPoseData.hiveLaunchPose1, color);
            gardenToLaunch1Control = pose(AutoPoseData.gardenToLaunch1Control, color);
            launch1TurnStart = pose(AutoPoseData.launch1TurnStart, color);
            launch1TurnEnd = pose(AutoPoseData.launch1TurnEnd, color);

            flowerPose = pose(AutoPoseData.flowerPose, color);
            launch1ToFlowerControl = pose(AutoPoseData.launch1ToFlowerControl, color);

            flowerDepartPose = pose(AutoPoseData.flowerDepartPose, color);
            hiveLaunchPose2 = pose(AutoPoseData.hiveLaunchPose2, color);
            flowerToLaunch2Control = pose(AutoPoseData.flowerToLaunch2Control, color);

            loadingZoneParkPose = pose(AutoPoseData.loadingZoneParkPose, color);
            launch2ToParkControl1 = pose(AutoPoseData.launch2ToParkControl1, color);
            launch2ToParkControl2 = pose(AutoPoseData.launch2ToParkControl2, color);
            robot.follower.setPose(startPose);
        }

        private Pose pose(Pose pose, String color) {
            return poseFactory.of(
                    AutoPoseData.mirrorX(pose.x(), color),
                    AutoPoseData.mirrorY(pose.y(), color),
                    AutoPoseData.mirrorHeading(pose.heading(), color));
        }

        public Path startToGarden() {
            return Paths.curve(startPose, startToGardenControl, gardenPollenPose)
                    .heading(Interpolator.piecewise()
                            .until(AutoConstants.gardenTurnSplit, Interpolator.linear(gardenTurnSeg1Start, gardenTurnSeg1End))
                            .until(1, Interpolator.linear(gardenTurnSeg2Start, gardenTurnSeg2End)));
        }

        public Path gardenToLaunch1() {
            return Paths.curve(gardenPollenPose, gardenToLaunch1Control, hiveLaunchPose1)
                    .heading(Interpolator.piecewise()
                            .until(1, Interpolator.linear(launch1TurnStart, launch1TurnEnd)));
        }

        public Path launch1ToFlower() {
            return Paths.curve(hiveLaunchPose1, launch1ToFlowerControl, flowerPose).tangent();
        }

        public Path flowerToLaunch2() {
            return Paths.curve(flowerDepartPose, flowerToLaunch2Control, hiveLaunchPose2).linear(flowerDepartPose, hiveLaunchPose2);
        }

        public Path launch2ToPark() {
            return Paths.curve(hiveLaunchPose2, launch2ToParkControl1, launch2ToParkControl2, loadingZoneParkPose).reverseTangent();
        }

        public Pose getStartPose() {
            return startPose;
        }
    }

}
