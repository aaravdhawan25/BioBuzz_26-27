package org.firstinspires.ftc.teamcode.HiveVision;

import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class BallChaseFollower {

    public enum State { IDLE, SCAN, TRAVEL, TURN, CHASE, PICKUP, DONE }

    private static final double CAM_FWD_OFFSET  = 0.0;
    private static final double CAM_LEFT_OFFSET = 0.0;
    private static final double HFOV_DEG        = 54.5;
    private static final double VFOV_DEG        = 42.0;
    private static final double FOV_MARGIN_DEG  = 4.0;

    public static double MAX_PLAN_RANGE    = 40.0;
    public static double MERGE_RADIUS      = 4.0;
    public static double APPROACH_DIST     = 24.0;
    public static double CLEAR_RADIUS      = 12.0;
    public static double FIELD_MIN         = 6.0;
    public static double FIELD_MAX         = 138.0;
    public static long   TRAVEL_TIMEOUT_MS = 4000;

    public static long   SETTLE_MS        = 250;
    public static long   SCAN_MS          = 250;
    public static double SEARCH_STEP_DEG  = 45.0;
    public static int    SEARCH_MAX_STEPS = 8;

    public static double TURN_TO_KP      = 0.8;
    public static double TURN_TO_MIN     = 0.12;
    public static double TURN_TO_MAX     = 0.5;
    public static double TURN_TO_TOL_DEG = 4.0;
    public static long   TURN_TIMEOUT_MS = 2500;

    public static double STOP_DIST      = 16.0;
    public static double AIM_TOL_DEG    = 5.0;
    public static double DRIVE_MIN_TX   = 15.0;
    public static double DRIVE_KP       = 0.03;
    public static double MIN_FWD        = 0.15;
    public static double MAX_FWD        = 0.7;
    public static double TURN_KP        = 0.025;
    public static double MIN_TURN       = 0.08;
    public static double MAX_TURN       = 0.6;
    public static double COAST_MAX_DIST = 26.0;
    public static double COAST_POWER    = 0.25;
    public static long   COAST_MS       = 450;
    public static long   LOST_MS        = 500;
    public static long   PICKUP_DWELL_MS = 600;
    public static double INTAKE_POWER    = 1.0;

    private static class Ball {
        double x, y;
        int n = 1;
        Ball(double x, double y) { this.x = x; this.y = y; }
    }

    private final Follower follower;
    private final BallTracker tracker;
    private final DcMotor intake;

    private State state = State.IDLE;
    private State afterTurn = State.SCAN;
    private boolean teleop = false;

    private List<Ball> memory = new ArrayList<>();
    private final List<Ball> samples = new ArrayList<>();
    private Ball currentTarget = null;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime lastSeen = new ElapsedTime();
    private final ElapsedTime total = new ElapsedTime();
    private double lastSeenDist = Double.MAX_VALUE;
    private boolean chaseSawBall = false;
    private boolean haveChaseBall = false;
    private double lastChaseX, lastChaseY;
    private double turnTarget = 0;

    private int pickups = 0;
    private int maxPickups = 0;
    private int searchSteps = 0;
    private double budgetSec = Double.MAX_VALUE;

    public BallChaseFollower(Follower follower, BallTracker tracker, DcMotor intakeOrNull) {
        if (follower == null) throw new IllegalArgumentException(
                "BallChaseFollower: follower is null - pass in your Pedro Follower");
        this.follower = follower;
        this.tracker = tracker;
        this.intake = intakeOrNull;
    }

    public BallChaseFollower(Follower follower, Limelight3A limelight, DcMotor intakeOrNull) {
        this(follower, new BallTracker(limelight), intakeOrNull);
    }

    public void setAllowedClasses(Set<Integer> classes) { tracker.setAllowedClasses(classes); }
    public void setAllowedClasses(Integer... classes)     { tracker.setAllowedClasses(classes); }

    public void setMaxPickups(int n)       { maxPickups = n; }
    public void setTimeBudgetSec(double s) { budgetSec = s; }
    public int getPickups()                { return pickups; }
    public State getState()                { return state; }
    public boolean isDone()                { return state == State.DONE; }
    public boolean isActive()              { return state != State.IDLE && state != State.DONE; }

    public void start() {
        memory.clear();
        pickups = 0;
        searchSteps = 0;
        currentTarget = null;
        tracker.reset();
        total.reset();
        enter(State.SCAN);
    }

    public void abort() {
        follower.stop();
        follower.manual(DrivePowers.zero());
        teleop = true;
        setIntake(0);
        drive(0, 0, 0);
        state = State.IDLE;
    }

    public void update() {
        if (!isActive()) return;
        if (total.seconds() > budgetSec) { finish(); return; }

        switch (state) {
            case SCAN:    doScan();    break;
            case TRAVEL:  doTravel();  break;
            case TURN:    doTurn();    break;
            case CHASE:   doChase();   break;
            case PICKUP:  doPickup();  break;
            default: break;
        }
    }

    public void addTelemetry(Telemetry t) {
        t.addData("hunt", "%s  pickups=%d/%s  remembered=%d", state, pickups,
                maxPickups > 0 ? Integer.toString(maxPickups) : "unlimited", memory.size());
        BallTracker.Sighting s = tracker.getLast();
        if (s != null) {
            t.addData("hunt target", "tx=%+.1f ty=%+.1f dist=%.1f in cls=%d conf=%.2f%s",
                    s.txDeg, s.tyDeg, s.distIn, s.classId, s.confidence,
                    s.predicted ? " (predicted)" : "");
        }
        for (int i = 0; i < memory.size() && i < 4; i++) {
            Ball b = memory.get(i);
            t.addData("  ball" + i, "(%.1f, %.1f) n=%d", b.x, b.y, b.n);
        }
    }

    public List<Pose> seeBalls() {
        Pose robot = follower.pose();
        List<Pose> out = new ArrayList<>();
        List<BallTracker.RawDet> dets = tracker.getGatedDetections();
        if (dets == null) return out;
        for (BallTracker.RawDet d : dets) {
            double[] p = project(d.txDeg, d.tyDeg, robot);
            if (p == null) continue;
            double facing = Math.atan2(p[1] - robot.y(), p[0] - robot.x());
            out.add(new Pose(p[0], p[1], facing));
        }
        java.util.Collections.sort(out, (a, b) -> Double.compare(
                Math.hypot(a.x() - robot.x(), a.y() - robot.y()),
                Math.hypot(b.x() - robot.x(), b.y() - robot.y())));
        return out;
    }

    public Pose nearestBall() {
        List<Pose> balls = seeBalls();
        return balls.isEmpty() ? null : balls.get(0);
    }

    public Pose approachPose(Pose ball, double stopShort) {
        Pose robot = follower.pose();
        double h = Math.atan2(ball.y() - robot.y(), ball.x() - robot.x());
        return new Pose(ball.x() - stopShort * Math.cos(h),
                ball.y() - stopShort * Math.sin(h), h);
    }

    private void doScan() {
        drive(0, 0, 0);
        setIntake(0);
        double t = stateTimer.milliseconds();
        if (t < SETTLE_MS) return;

        Pose pose = follower.pose();
        if (t < SETTLE_MS + SCAN_MS) {
            sampleDetections(pose);
            return;
        }
        finishScan(pose);
    }

    private void sampleDetections(Pose pose) {

        if (tracker.getStalenessMs() > stateTimer.milliseconds() - SETTLE_MS) return;

        List<BallTracker.RawDet> dets = tracker.getGatedDetections();
        if (dets == null) return;

        for (BallTracker.RawDet d : dets) {
            double[] p = project(d.txDeg, d.tyDeg, pose);
            if (p == null) continue;
            if (Math.hypot(p[0] - pose.x(), p[1] - pose.y()) > MAX_PLAN_RANGE) continue;
            addSample(p[0], p[1]);
        }
    }

    private void addSample(double x, double y) {
        for (Ball b : samples) {
            if (Math.hypot(b.x - x, b.y - y) < MERGE_RADIUS) {
                b.x = (b.x * b.n + x) / (b.n + 1);
                b.y = (b.y * b.n + y) / (b.n + 1);
                b.n++;
                return;
            }
        }
        samples.add(new Ball(x, y));
    }

    private void finishScan(Pose pose) {

        List<Ball> next = new ArrayList<>();
        for (Ball b : memory) {
            if (!inView(b, pose)) next.add(b);
        }
        next.addAll(samples);
        memory = next;

        if (maxPickups > 0 && pickups >= maxPickups) { finish(); return; }

        Ball target = nearest(memory, pose);
        if (target == null) {
            if (searchSteps >= SEARCH_MAX_STEPS) { finish(); return; }
            searchSteps++;
            startTurn(pose.heading() + Math.toRadians(SEARCH_STEP_DEG), State.SCAN);
            return;
        }

        searchSteps = 0;
        currentTarget = target;
        double dx = target.x - pose.x();
        double dy = target.y - pose.y();
        double dist = Math.hypot(dx, dy);
        double heading = Math.atan2(dy, dx);

        if (dist > APPROACH_DIST + 4.0) {
            startTravel(pose, target, heading);
        } else {
            startTurn(heading, State.CHASE);
        }
    }

    private void startTravel(Pose pose, Ball target, double heading) {
        double ax = Range.clip(target.x - APPROACH_DIST * Math.cos(heading), FIELD_MIN, FIELD_MAX);
        double ay = Range.clip(target.y - APPROACH_DIST * Math.sin(heading), FIELD_MIN, FIELD_MAX);

        if (Math.hypot(ax - pose.x(), ay - pose.y()) < 3.0) {
            startTurn(heading, State.CHASE);
            return;
        }

        double endHeading = pose.heading() + wrap(heading - pose.heading());

        Path path = Paths.line(pose, new Pose(ax, ay)).linear(pose.heading(), endHeading);

        follower.follow(path);
        teleop = false;
        enter(State.TRAVEL);
    }

    private void doTravel() {
        setIntake(0);
        if (!follower.isBusy()) {
            enter(State.CHASE);
        } else if (stateTimer.milliseconds() > TRAVEL_TIMEOUT_MS) {
            follower.stop();
            teleop = false;
            enter(State.SCAN);
        }
    }

    private void startTurn(double targetHeading, State next) {
        turnTarget = targetHeading;
        afterTurn = next;
        enter(State.TURN);
    }

    private void doTurn() {
        setIntake(0);
        double err = wrap(turnTarget - follower.pose().heading());
        if (Math.abs(err) < Math.toRadians(TURN_TO_TOL_DEG)
                || stateTimer.milliseconds() > TURN_TIMEOUT_MS) {
            drive(0, 0, 0);
            enter(afterTurn);
            return;
        }
        double cmd = Range.clip(err * TURN_TO_KP, -TURN_TO_MAX, TURN_TO_MAX);
        if (Math.abs(cmd) < TURN_TO_MIN) cmd = Math.copySign(TURN_TO_MIN, err);
        drive(0, 0, cmd);
    }

    private void doChase() {
        Pose pose = follower.pose();
        BallTracker.Sighting s = tracker.update();

        if (s != null && !s.predicted) {
            double tx = s.txDeg;
            double ty = s.tyDeg;
            double dist = s.distIn;

            lastSeen.reset();
            lastSeenDist = dist;
            chaseSawBall = true;

            double[] p = project(tx, ty, pose);
            if (p != null) { lastChaseX = p[0]; lastChaseY = p[1]; haveChaseBall = true; }

            boolean aimed = Math.abs(tx) < AIM_TOL_DEG;
            if (dist <= STOP_DIST && aimed) {
                drive(0, 0, 0);
                enter(State.PICKUP);
                return;
            }

            double turn = Range.clip(-tx * TURN_KP, -MAX_TURN, MAX_TURN);
            if (!aimed && Math.abs(turn) < MIN_TURN) turn = Math.copySign(MIN_TURN, -tx);

            double fwd = 0.0;
            if (dist > STOP_DIST && Math.abs(tx) < DRIVE_MIN_TX) {
                fwd = Range.clip((dist - STOP_DIST) * DRIVE_KP, MIN_FWD, MAX_FWD);
            }
            setIntake(INTAKE_POWER);
            drive(fwd, 0, turn);
            return;
        }

        if (s != null) {

            boolean wasClose = lastSeenDist <= COAST_MAX_DIST;
            if (wasClose) {
                if (lastSeen.milliseconds() < COAST_MS) {
                    setIntake(INTAKE_POWER);
                    drive(COAST_POWER, 0, 0);
                    return;
                }
                drive(0, 0, 0);
                enter(State.PICKUP);
                return;
            }
            double turn = Range.clip(-s.txDeg * TURN_KP, -MAX_TURN, MAX_TURN);
            if (Math.abs(turn) < MIN_TURN) turn = Math.copySign(MIN_TURN, -s.txDeg);
            drive(0, 0, turn);
            return;
        }

        if (chaseSawBall && lastSeenDist <= COAST_MAX_DIST) {
            if (lastSeen.milliseconds() < COAST_MS) {
                setIntake(INTAKE_POWER);
                drive(COAST_POWER, 0, 0);
            } else {
                drive(0, 0, 0);
                enter(State.PICKUP);
            }
            return;
        }

        drive(0, 0, 0);
        if (lastSeen.milliseconds() > LOST_MS) {
            if (currentTarget != null) memory.remove(currentTarget);
            enter(State.SCAN);
        }
    }

    private void doPickup() {
        drive(0, 0, 0);
        setIntake(INTAKE_POWER);
        if (stateTimer.milliseconds() < PICKUP_DWELL_MS) return;

        pickups++;
        double cx, cy;
        boolean haveCoords = true;
        if (haveChaseBall) { cx = lastChaseX; cy = lastChaseY; }
        else if (currentTarget != null) { cx = currentTarget.x; cy = currentTarget.y; }
        else { cx = 0; cy = 0; haveCoords = false; }

        if (haveCoords) {
            for (int i = memory.size() - 1; i >= 0; i--) {
                Ball b = memory.get(i);
                if (Math.hypot(b.x - cx, b.y - cy) < CLEAR_RADIUS) memory.remove(i);
            }
        }
        if (currentTarget != null) memory.remove(currentTarget);
        currentTarget = null;
        haveChaseBall = false;
        tracker.reset();

        if (maxPickups > 0 && pickups >= maxPickups) { finish(); return; }
        enter(State.SCAN);
    }

    private void finish() {
        follower.stop();
        follower.manual(DrivePowers.zero());
        teleop = true;
        drive(0, 0, 0);
        setIntake(0);
        state = State.DONE;
    }

    private double[] project(double txDeg, double tyDeg, Pose pose) {
        double p = Math.toRadians(BallTracker.CAM_PITCH_DEG);
        double h = BallTracker.CAM_H - BallTracker.BALL_H;
        double tanTx = Math.tan(Math.toRadians(txDeg));
        double tanTy = Math.tan(Math.toRadians(tyDeg));

        double denom = Math.sin(p) - tanTy * Math.cos(p);
        if (denom <= 0.02) return null;
        double s = h / denom;

        double xr = s * (Math.cos(p) + tanTy * Math.sin(p)) + CAM_FWD_OFFSET;
        double yr = -s * tanTx + CAM_LEFT_OFFSET;

        double th = pose.heading();
        double fx = pose.x() + xr * Math.cos(th) - yr * Math.sin(th);
        double fy = pose.y() + xr * Math.sin(th) + yr * Math.cos(th);
        return new double[]{fx, fy};
    }

    private boolean inView(Ball b, Pose pose) {
        double dxw = b.x - pose.x();
        double dyw = b.y - pose.y();
        double th = pose.heading();
        double xr =  dxw * Math.cos(th) + dyw * Math.sin(th);
        double yr = -dxw * Math.sin(th) + dyw * Math.cos(th);

        double dx = xr - CAM_FWD_OFFSET;
        double dy = yr - CAM_LEFT_OFFSET;
        if (Math.hypot(dx, dy) > MAX_PLAN_RANGE) return false;

        double p = Math.toRadians(BallTracker.CAM_PITCH_DEG);
        double h = BallTracker.CAM_H - BallTracker.BALL_H;
        double zc = dx * Math.cos(p) + h * Math.sin(p);
        if (zc <= 0) return false;
        double xc = -dy;
        double yc = dx * Math.sin(p) - h * Math.cos(p);

        double txp = Math.toDegrees(Math.atan2(xc, zc));
        double typ = Math.toDegrees(Math.atan2(yc, zc));
        return Math.abs(txp) < HFOV_DEG / 2 - FOV_MARGIN_DEG
                && Math.abs(typ) < VFOV_DEG / 2 - FOV_MARGIN_DEG;
    }

    private Ball nearest(List<Ball> list, Pose pose) {
        Ball best = null;
        double bestD = Double.MAX_VALUE;
        for (Ball b : list) {
            double d = Math.hypot(b.x - pose.x(), b.y - pose.y());
            if (d < bestD) { bestD = d; best = b; }
        }
        return best;
    }

    private void enter(State s) {
        state = s;
        stateTimer.reset();
        if (s == State.SCAN) samples.clear();
        if (s == State.CHASE) {
            lastSeen.reset();
            lastSeenDist = Double.MAX_VALUE;
            chaseSawBall = false;
            haveChaseBall = false;
        }
        if (s != State.TRAVEL && !teleop) {
            follower.manual(DrivePowers.zero());
            teleop = true;
        }
    }

    private void drive(double fwd, double strafe, double turn) {
        follower.manual(new DrivePowers(fwd, strafe, turn));
    }

    private void setIntake(double power) {
        if (intake != null) intake.setPower(power);
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
}