package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Commands.BallHuntCommand;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Utils.Telem;

/**
 * Bench test for the Hive Vision ball hunt. Put the robot on the field facing
 * some red nectar, press start: it scans, Pedro-drives near the closest one,
 * camera-chases it in, and repeats until 2 pickups or 20 s.
 *
 * Robot.update() owns follower.update() as usual; the hunt never calls it.
 */
@Autonomous(name = "Ball Hunt Test", group = "Test")
public class BallHuntTest extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, "RED", true);
        // Field-center start so ball projections land inside the field bounds.
        robot.follower.setPose(PoseFactory.degrees().of(72, 72, 90));
    }

    @Override
    public void init_loop() {
        Telem.addData("Limelight", robot.limelight == null
                ? "NOT FOUND - add \"limelight\" to the config" : "ok");
        Telem.update();
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(BallHuntCommand.alliance(robot, 2, 20));
    }

    @Override
    public void loop() {
        robot.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
