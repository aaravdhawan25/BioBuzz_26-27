package org.firstinspires.ftc.teamcode.HiveVision;

import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Constants.BallChaseFollowerConstants;
import org.firstinspires.ftc.teamcode.Utils.Constants.BallTrackerConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class BallChaseFollower {

    public enum State { IDLE, SCAN, TRAVEL, TURN, CHASE, PICKUP, DONE }

    private static class Ball {
        double x, y;
        int n = 1;
        Ball(double x, double y) { this.x = x; this.y = y; }
    }

    private final Follower follower;
    private final BallTracker tracker;

    private State state = State.IDLE;
    private State afterTurn = State.SCAN;
    private boolean teleop = false;

    private List<Ball> memory = new ArrayList<>();
    private final List<Ball> samples = new ArrayList<>();
    private Ball currentTarget = null;

    private List<Pose> visibleBalls = new ArrayList<>();
    private double ballDistance = Double.POSITIVE_INFINITY;
    private BallTracker.Sighting sighting;

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

    public BallChaseFollower(Follower follower, BallTracker tracker) {
        if (follower == null) throw new IllegalArgumentException(
                "BallChaseFollower: follower is null - pass in your Pedro Follower");
        this.follower = follower;
        this.tracker = tracker;
    }

    public BallChaseFollower(Follower follower, Limelight3A limelight) {
        this(follower, new BallTracker(limelight));
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

    public void updateVision() {
        sighting = tracker.update();
        Pose robot = follower.pose();
        List<Pose> balls = new ArrayList<>();
        double closest = Double.POSITIVE_INFINITY;
        List<BallTracker.RawDet> dets = tracker.getGatedDetections();
        if (dets != null) {
            for (BallTracker.RawDet d : dets) {
                closest = Math.min(closest, cameraRange(d.txDeg, d.tyDeg));
                double[] p = project(d.txDeg, d.tyDeg, robot);
                if (p == null) continue;
                double facing = Math.atan2(p[1] - robot.y(), p[0] - robot.x());
                balls.add(new Pose(p[0], p[1], facing));
            }
            java.util.Collections.sort(balls, (a, b) -> Double.compare(
                    Math.hypot(a.x() - robot.x(), a.y() - robot.y()),
                    Math.hypot(b.x() - robot.x(), b.y() - robot.y())));
        }
        visibleBalls = balls;
        ballDistance = closest;
    }

    public List<Pose> seeBalls() {
        return visibleBalls;
    }

    public double distanceToBall() {
        return ballDistance;
    }

    public BallTracker.Sighting getSighting() {
        return sighting;
    }

    private double cameraRange(double txDeg, double tyDeg) {
        double p = Math.toRadians(BallTrackerConstants.camPitchDeg);
        double h = BallTrackerConstants.camHeight - BallTrackerConstants.ballHeight;
        double tanTx = Math.tan(Math.toRadians(txDeg));
        double tanTy = Math.tan(Math.toRadians(tyDeg));
        double denom = Math.sin(p) - tanTy * Math.cos(p);
        if (denom <= 0.02) return Double.POSITIVE_INFINITY;
        double s = h / denom;
        double forward = s * (Math.cos(p) + tanTy * Math.sin(p));
        double left = -s * tanTx;
        return Math.hypot(forward, left);
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
        double t = stateTimer.milliseconds();
        if (t < BallChaseFollowerConstants.settleMs) return;

        Pose pose = follower.pose();
        if (t < BallChaseFollowerConstants.settleMs + BallChaseFollowerConstants.scanMs) {
            sampleDetections(pose);
            return;
        }
        finishScan(pose);
    }

    private void sampleDetections(Pose pose) {

        if (tracker.getStalenessMs() > stateTimer.milliseconds() - BallChaseFollowerConstants.settleMs) return;

        List<BallTracker.RawDet> dets = tracker.getGatedDetections();
        if (dets == null) return;

        for (BallTracker.RawDet d : dets) {
            double[] p = project(d.txDeg, d.tyDeg, pose);
            if (p == null) continue;
            if (Math.hypot(p[0] - pose.x(), p[1] - pose.y()) > BallChaseFollowerConstants.maxPlanRange) continue;
            addSample(p[0], p[1]);
        }
    }

    private void addSample(double x, double y) {
        for (Ball b : samples) {
            if (Math.hypot(b.x - x, b.y - y) < BallChaseFollowerConstants.mergeRadius) {
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
            if (searchSteps >= BallChaseFollowerConstants.searchMaxSteps) { finish(); return; }
            searchSteps++;
            startTurn(pose.heading() + Math.toRadians(BallChaseFollowerConstants.searchStepDeg), State.SCAN);
            return;
        }

        searchSteps = 0;
        currentTarget = target;
        double dx = target.x - pose.x();
        double dy = target.y - pose.y();
        double dist = Math.hypot(dx, dy);
        double heading = Math.atan2(dy, dx);

        if (dist > BallChaseFollowerConstants.approachDist + 4.0) {
            startTravel(pose, target, heading);
        } else {
            startTurn(heading, State.CHASE);
        }
    }

    private void startTravel(Pose pose, Ball target, double heading) {
        double ax = Range.clip(target.x - BallChaseFollowerConstants.approachDist * Math.cos(heading), BallChaseFollowerConstants.fieldMin, BallChaseFollowerConstants.fieldMax);
        double ay = Range.clip(target.y - BallChaseFollowerConstants.approachDist * Math.sin(heading), BallChaseFollowerConstants.fieldMin, BallChaseFollowerConstants.fieldMax);

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
        if (!follower.isBusy()) {
            enter(State.CHASE);
        } else if (stateTimer.milliseconds() > BallChaseFollowerConstants.travelTimeoutMs) {
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
        double err = wrap(turnTarget - follower.pose().heading());
        if (Math.abs(err) < Math.toRadians(BallChaseFollowerConstants.turnToTolDeg)
                || stateTimer.milliseconds() > BallChaseFollowerConstants.turnTimeoutMs) {
            drive(0, 0, 0);
            enter(afterTurn);
            return;
        }
        double cmd = Range.clip(err * BallChaseFollowerConstants.turnToKp, -BallChaseFollowerConstants.turnToMax, BallChaseFollowerConstants.turnToMax);
        if (Math.abs(cmd) < BallChaseFollowerConstants.turnToMin) cmd = Math.copySign(BallChaseFollowerConstants.turnToMin, err);
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

            boolean aimed = Math.abs(tx) < BallChaseFollowerConstants.aimTolDeg;
            if (dist <= BallChaseFollowerConstants.stopDist && aimed) {
                drive(0, 0, 0);
                enter(State.PICKUP);
                return;
            }

            double turn = Range.clip(-tx * BallChaseFollowerConstants.turnKp, -BallChaseFollowerConstants.maxTurn, BallChaseFollowerConstants.maxTurn);
            if (!aimed && Math.abs(turn) < BallChaseFollowerConstants.minTurn) turn = Math.copySign(BallChaseFollowerConstants.minTurn, -tx);

            double fwd = 0.0;
            if (dist > BallChaseFollowerConstants.stopDist && Math.abs(tx) < BallChaseFollowerConstants.driveMinTx) {
                fwd = Range.clip((dist - BallChaseFollowerConstants.stopDist) * BallChaseFollowerConstants.driveKp, BallChaseFollowerConstants.minFwd, BallChaseFollowerConstants.maxFwd);
            }
            drive(fwd, 0, turn);
            return;
        }

        if (s != null) {

            boolean wasClose = lastSeenDist <= BallChaseFollowerConstants.coastMaxDist;
            if (wasClose) {
                if (lastSeen.milliseconds() < BallChaseFollowerConstants.coastMs) {
                    drive(BallChaseFollowerConstants.coastPower, 0, 0);
                    return;
                }
                drive(0, 0, 0);
                enter(State.PICKUP);
                return;
            }
            double turn = Range.clip(-s.txDeg * BallChaseFollowerConstants.turnKp, -BallChaseFollowerConstants.maxTurn, BallChaseFollowerConstants.maxTurn);
            if (Math.abs(turn) < BallChaseFollowerConstants.minTurn) turn = Math.copySign(BallChaseFollowerConstants.minTurn, -s.txDeg);
            drive(0, 0, turn);
            return;
        }

        if (chaseSawBall && lastSeenDist <= BallChaseFollowerConstants.coastMaxDist) {
            if (lastSeen.milliseconds() < BallChaseFollowerConstants.coastMs) {
                drive(BallChaseFollowerConstants.coastPower, 0, 0);
            } else {
                drive(0, 0, 0);
                enter(State.PICKUP);
            }
            return;
        }

        drive(0, 0, 0);
        if (lastSeen.milliseconds() > BallChaseFollowerConstants.lostMs) {
            if (currentTarget != null) memory.remove(currentTarget);
            enter(State.SCAN);
        }
    }

    private void doPickup() {
        drive(0, 0, 0);
        if (stateTimer.milliseconds() < BallChaseFollowerConstants.pickupDwellMs) return;

        pickups++;
        double cx, cy;
        boolean haveCoords = true;
        if (haveChaseBall) { cx = lastChaseX; cy = lastChaseY; }
        else if (currentTarget != null) { cx = currentTarget.x; cy = currentTarget.y; }
        else { cx = 0; cy = 0; haveCoords = false; }

        if (haveCoords) {
            for (int i = memory.size() - 1; i >= 0; i--) {
                Ball b = memory.get(i);
                if (Math.hypot(b.x - cx, b.y - cy) < BallChaseFollowerConstants.clearRadius) memory.remove(i);
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
        state = State.DONE;
    }

    private double[] project(double txDeg, double tyDeg, Pose pose) {
        double p = Math.toRadians(BallTrackerConstants.camPitchDeg);
        double h = BallTrackerConstants.camHeight - BallTrackerConstants.ballHeight;
        double tanTx = Math.tan(Math.toRadians(txDeg));
        double tanTy = Math.tan(Math.toRadians(tyDeg));

        double denom = Math.sin(p) - tanTy * Math.cos(p);
        if (denom <= 0.02) return null;
        double s = h / denom;

        double xr = s * (Math.cos(p) + tanTy * Math.sin(p)) + BallTrackerConstants.camXOffset;
        double yr = -s * tanTx + BallTrackerConstants.camYOffset;

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

        double dx = xr - BallTrackerConstants.camXOffset;
        double dy = yr - BallTrackerConstants.camYOffset;
        if (Math.hypot(dx, dy) > BallChaseFollowerConstants.maxPlanRange) return false;

        double p = Math.toRadians(BallTrackerConstants.camPitchDeg);
        double h = BallTrackerConstants.camHeight - BallTrackerConstants.ballHeight;
        double zc = dx * Math.cos(p) + h * Math.sin(p);
        if (zc <= 0) return false;
        double xc = -dy;
        double yc = dx * Math.sin(p) - h * Math.cos(p);

        double txp = Math.toDegrees(Math.atan2(xc, zc));
        double typ = Math.toDegrees(Math.atan2(yc, zc));
        return Math.abs(txp) < BallTrackerConstants.hfovDeg / 2 - BallTrackerConstants.fovMarginDeg
                && Math.abs(typ) < BallTrackerConstants.vfovDeg / 2 - BallTrackerConstants.fovMarginDeg;
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

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
}