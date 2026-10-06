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
                new TurretCommand(robot, Turret.TurretState.FORWARD)
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
            startPose = pose(AutoConstants.startX, AutoConstants.startY, AutoConstants.startHeading, color);
            gardenPollenPose = pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, AutoConstants.gardenPollenHeading, color);
            startToGardenControl = pose(AutoConstants.startToGardenContX, AutoConstants.startToGardenContY, 0, color);
            gardenTurnSeg1Start = pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, AutoConstants.gardenTurnSeg1StartHeading, color);
            gardenTurnSeg1End = pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, AutoConstants.gardenTurnSeg1EndHeading, color);
            gardenTurnSeg2Start = pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, AutoConstants.gardenTurnSeg2StartHeading, color);
            gardenTurnSeg2End = pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, AutoConstants.gardenTurnSeg2EndHeading, color);

            hiveLaunchPose1 = pose(AutoConstants.hiveLaunch1X, AutoConstants.hiveLaunch1Y, AutoConstants.hiveLaunch1Heading, color);
            gardenToLaunch1Control = pose(AutoConstants.gardenToLaunch1ContX, AutoConstants.gardenToLaunch1ContY, 0, color);
            launch1TurnStart = pose(AutoConstants.hiveLaunch1X, AutoConstants.hiveLaunch1Y, AutoConstants.launch1TurnStartHeading, color);
            launch1TurnEnd = pose(AutoConstants.hiveLaunch1X, AutoConstants.hiveLaunch1Y, AutoConstants.launch1TurnEndHeading, color);

            flowerPose = pose(AutoConstants.flowerX, AutoConstants.flowerY, AutoConstants.flowerHeading, color);
            launch1ToFlowerControl = pose(AutoConstants.launch1ToFlowerContX, AutoConstants.launch1ToFlowerContY, 0, color);

            flowerDepartPose = pose(AutoConstants.flowerX, AutoConstants.flowerY, AutoConstants.flowerDepartHeading, color);
            hiveLaunchPose2 = pose(AutoConstants.hiveLaunch2X, AutoConstants.hiveLaunch2Y, AutoConstants.hiveLaunch2Heading, color);
            flowerToLaunch2Control = pose(AutoConstants.flowerToLaunch2ContX, AutoConstants.flowerToLaunch2ContY, 0, color);

            loadingZoneParkPose = pose(AutoConstants.loadingZoneParkX, AutoConstants.loadingZoneParkY, AutoConstants.loadingZoneParkHeading, color);
            launch2ToParkControl1 = pose(AutoConstants.launch2ToParkCont1X, AutoConstants.launch2ToParkCont1Y, 0, color);
            launch2ToParkControl2 = pose(AutoConstants.launch2ToParkCont2X, AutoConstants.launch2ToParkCont2Y, 0, color);
            robot.follower.setPose(startPose);
        }

        private Pose pose(double x, double y, double headingDeg, String color) {
            return poseFactory.of(
                    AutoPoseData.mirrorX(x, color),
                    AutoPoseData.mirrorY(y, color),
                    AutoPoseData.mirrorHeading(headingDeg, color));
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
