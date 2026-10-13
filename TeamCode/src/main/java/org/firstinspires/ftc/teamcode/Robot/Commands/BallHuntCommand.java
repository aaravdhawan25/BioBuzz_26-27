package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.Telem;
import org.firstinspires.ftc.teamcode.Vision.BallChaseFollower;
import org.firstinspires.ftc.teamcode.Vision.BallTracker;

/**
 * Hive Vision ball hunt as an FTCLib command.
 *
 * Scans with the Limelight, Pedro-drives near the closest allowed ball, then
 * camera-chases it into the intake. Finishes after {@code count} pickups or
 * {@code seconds}, whichever comes first.
 *
 * Does NOT call follower.update() - Robot.update() already does that every loop.
 *
 *   new BallHuntCommand(robot, 2, 20, BallTracker.CLASS_RED)
 *   BallHuntCommand.alliance(robot, 2, 20)   // red or blue based on Robot.isRed
 */
public class BallHuntCommand extends CommandBase {

    private final Robot robot;
    private final BallChaseFollower hunt;
    private Intake.IntakeStates lastIntake = null;

    public BallHuntCommand(Robot robot, int count, double seconds, Integer... classes) {
        this.robot = robot;
        BallTracker tracker = new BallTracker(robot.limelight);
        this.hunt = new BallChaseFollower(robot.follower, tracker, this::setIntake);
        hunt.setAllowedClasses(classes);
        hunt.setMaxPickups(count);
        hunt.setTimeBudgetSec(seconds);
        addRequirements(robot.intake);
    }

    /** Chase this alliance's nectar color. */
    public static BallHuntCommand alliance(Robot robot, int count, double seconds) {
        return new BallHuntCommand(robot, count, seconds,
                Robot.isRed ? BallTracker.CLASS_RED : BallTracker.CLASS_BLUE);
    }

    @Override
    public void initialize() {
        lastIntake = null;
        hunt.start();
    }

    @Override
    public void execute() {
        hunt.update();
        if (Telem.telemetry != null) hunt.addTelemetry(Telem.telemetry);
    }

    @Override
    public boolean isFinished() {
        return hunt.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) hunt.abort();
        setIntake(0);
    }

    public int getPickups() {
        return hunt.getPickups();
    }

    /** Map the hunt's intake power onto the Intake subsystem; only write on change. */
    private void setIntake(double power) {
        Intake.IntakeStates next = power > 0 ? Intake.IntakeStates.ON
                : power < 0 ? Intake.IntakeStates.REVERSE
                : Intake.IntakeStates.OFF;
        if (next != lastIntake) {
            robot.intake.setState(next);
            lastIntake = next;
        }
    }
}
