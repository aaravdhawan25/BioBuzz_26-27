package org.firstinspires.ftc.teamcode.HiveVision;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Utils.Constants.FieldPlannerConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class FieldPlanner {

    private static double pad() {
        return FieldPlannerConstants.robotRadius + FieldPlannerConstants.safetyMargin;
    }

    public static boolean blocked(double x, double y) {
        double p = pad();
        return x > FieldPlannerConstants.obstacleMinX - p && x < FieldPlannerConstants.obstacleMaxX + p && y > FieldPlannerConstants.obstacleMinY - p && y < FieldPlannerConstants.obstacleMaxY + p;
    }

    public static boolean onField(double x, double y) {
        return x >= FieldPlannerConstants.fieldMin && x <= FieldPlannerConstants.fieldMax && y >= FieldPlannerConstants.fieldMin && y <= FieldPlannerConstants.fieldMax;
    }

    public static boolean segmentClear(double ax, double ay, double bx, double by) {
        double p = pad();
        double minX = FieldPlannerConstants.obstacleMinX - p, maxX = FieldPlannerConstants.obstacleMaxX + p, minY = FieldPlannerConstants.obstacleMinY - p, maxY = FieldPlannerConstants.obstacleMaxY + p;
        double dx = bx - ax, dy = by - ay;
        double t0 = 0, t1 = 1;
        double[] ps = {-dx, dx, -dy, dy};
        double[] qs = {ax - minX, maxX - ax, ay - minY, maxY - ay};
        for (int i = 0; i < 4; i++) {
            if (Math.abs(ps[i]) < 1e-9) {
                if (qs[i] <= 0) return true;
            } else {
                double t = qs[i] / ps[i];
                if (ps[i] < 0) t0 = Math.max(t0, t);
                else t1 = Math.min(t1, t);
            }
        }
        return t1 - t0 <= 1e-6;
    }

    public static List<double[]> route(double sx, double sy, double ex, double ey) {
        if (blocked(ex, ey)) return null;
        if (blocked(sx, sy) || segmentClear(sx, sy, ex, ey)) {
            List<double[]> direct = new ArrayList<>();
            direct.add(new double[]{sx, sy});
            direct.add(new double[]{ex, ey});
            return direct;
        }

        double c = pad() + FieldPlannerConstants.cornerExtra;
        double[][] nodes = {
                {sx, sy},
                {FieldPlannerConstants.obstacleMinX - c, FieldPlannerConstants.obstacleMinY - c},
                {FieldPlannerConstants.obstacleMaxX + c, FieldPlannerConstants.obstacleMinY - c},
                {FieldPlannerConstants.obstacleMaxX + c, FieldPlannerConstants.obstacleMaxY + c},
                {FieldPlannerConstants.obstacleMinX - c, FieldPlannerConstants.obstacleMaxY + c},
                {ex, ey}
        };
        int n = nodes.length;
        double[] dist = new double[n];
        int[] prev = new int[n];
        boolean[] done = new boolean[n];
        Arrays.fill(dist, Double.MAX_VALUE);
        Arrays.fill(prev, -1);
        dist[0] = 0;

        for (int iter = 0; iter < n; iter++) {
            int u = -1;
            for (int i = 0; i < n; i++) {
                if (!done[i] && (u < 0 || dist[i] < dist[u])) u = i;
            }
            if (u < 0 || dist[u] == Double.MAX_VALUE) break;
            done[u] = true;
            for (int v = 0; v < n; v++) {
                if (done[v] || v == u) continue;
                if (v != n - 1 && v != 0 && !onField(nodes[v][0], nodes[v][1])) continue;
                if (!segmentClear(nodes[u][0], nodes[u][1], nodes[v][0], nodes[v][1])) continue;
                double d = dist[u] + Math.hypot(nodes[v][0] - nodes[u][0], nodes[v][1] - nodes[u][1]);
                if (d < dist[v]) {
                    dist[v] = d;
                    prev[v] = u;
                }
            }
        }

        if (dist[n - 1] == Double.MAX_VALUE) return null;
        List<double[]> out = new ArrayList<>();
        for (int v = n - 1; v >= 0; v = prev[v]) out.add(0, nodes[v]);
        return out;
    }

    public static double length(List<double[]> pts) {
        double d = 0;
        for (int i = 1; i < pts.size(); i++) {
            d += Math.hypot(pts.get(i)[0] - pts.get(i - 1)[0], pts.get(i)[1] - pts.get(i - 1)[1]);
        }
        return d;
    }

    public static Path plan(Pose start, Pose end) {
        List<double[]> pts = route(start.x(), start.y(), end.x(), end.y());
        if (pts == null) return null;

        double h0 = start.heading();
        double h1 = h0 + wrap(end.heading() - h0);

        if (pts.size() == 2) {
            return Paths.line(start, end).linear(h0, h1);
        }

        for (int tries = 0; tries < FieldPlannerConstants.filletTries; tries++) {
            Path smooth = filleted(pts, FieldPlannerConstants.filletRadius * (1.0 - (double) tries / FieldPlannerConstants.filletTries));
            if (smooth != null) {
                Path withHeading = smooth.linear(h0, h1);
                if (curveClear(withHeading, start, end)) return withHeading;
            }
        }

        Path[] legs = new Path[pts.size() - 1];
        for (int i = 0; i < legs.length; i++) {
            legs[i] = Paths.line(
                    new Pose(pts.get(i)[0], pts.get(i)[1]),
                    new Pose(pts.get(i + 1)[0], pts.get(i + 1)[1]));
        }
        return Paths.path(legs).linear(h0, h1);
    }

    private static Path filleted(List<double[]> pts, double radius) {
        List<Path> parts = new ArrayList<>();
        double[] cursor = pts.get(0);
        for (int i = 1; i < pts.size() - 1; i++) {
            double[] a = pts.get(i - 1), c = pts.get(i), b = pts.get(i + 1);
            double inLen = Math.hypot(c[0] - a[0], c[1] - a[1]);
            double outLen = Math.hypot(b[0] - c[0], b[1] - c[1]);
            double r = Math.min(radius, 0.45 * Math.min(inLen, outLen));
            if (r < 1.0) return null;
            double[] pIn = {c[0] - r * (c[0] - a[0]) / inLen, c[1] - r * (c[1] - a[1]) / inLen};
            double[] pOut = {c[0] + r * (b[0] - c[0]) / outLen, c[1] + r * (b[1] - c[1]) / outLen};
            if (Math.hypot(pIn[0] - cursor[0], pIn[1] - cursor[1]) > 0.5) {
                parts.add(Paths.line(new Pose(cursor[0], cursor[1]), new Pose(pIn[0], pIn[1])));
            }
            parts.add(Paths.curve(new Pose(pIn[0], pIn[1]), new Pose(c[0], c[1]), new Pose(pOut[0], pOut[1])));
            cursor = pOut;
        }
        double[] e = pts.get(pts.size() - 1);
        parts.add(Paths.line(new Pose(cursor[0], cursor[1]), new Pose(e[0], e[1])));
        return Paths.path(parts.toArray(new Path[0]));
    }

    private static boolean curveClear(Path path, Pose start, Pose end) {
        double minX = Math.min(FieldPlannerConstants.fieldMin, Math.min(start.x(), end.x()));
        double maxX = Math.max(FieldPlannerConstants.fieldMax, Math.max(start.x(), end.x()));
        double minY = Math.min(FieldPlannerConstants.fieldMin, Math.min(start.y(), end.y()));
        double maxY = Math.max(FieldPlannerConstants.fieldMax, Math.max(start.y(), end.y()));
        for (int i = 0; i <= FieldPlannerConstants.curveSamples; i++) {
            Pose p = path.get((double) i / FieldPlannerConstants.curveSamples);
            if (blocked(p.x(), p.y())) return false;
            if (p.x() < minX || p.x() > maxX || p.y() < minY || p.y() > maxY) return false;
        }
        return true;
    }

    public static Pose bestApproach(Pose robot, List<Pose> balls, double stopShort) {
        Pose best = null;
        double bestLen = Double.MAX_VALUE;
        for (Pose ball : balls) {
            for (int k = -1; k < FieldPlannerConstants.approachDirections; k++) {
                double a = k < 0
                        ? Math.atan2(ball.y() - robot.y(), ball.x() - robot.x())
                        : 2 * Math.PI * k / FieldPlannerConstants.approachDirections;
                double ax = ball.x() - stopShort * Math.cos(a);
                double ay = ball.y() - stopShort * Math.sin(a);
                if (blocked(ax, ay) || !onField(ax, ay)) continue;
                List<double[]> pts = route(robot.x(), robot.y(), ax, ay);
                if (pts == null) continue;
                double len = length(pts) + turnCost(robot.heading(), a);
                if (len < bestLen) {
                    bestLen = len;
                    best = new Pose(ax, ay, a);
                }
            }
        }
        return best;
    }

    public static class Target {
        public final Pose entry;
        public final Pose exit;
        public final int balls;
        public final double score;

        Target(Pose entry, Pose exit, int balls, double score) {
            this.entry = entry;
            this.exit = exit;
            this.balls = balls;
            this.score = score;
        }
    }

    public static Target bestTarget(Pose robot, List<Pose> balls, double stopShort) {
        return bestTarget(robot, balls, stopShort, FieldPlannerConstants.intakeCapacity);
    }

    public static Target bestTarget(Pose robot, List<Pose> balls, double stopShort, int maxBalls) {
        Target best = null;
        int limit = Math.min(maxBalls, FieldPlannerConstants.intakeCapacity);
        if (limit < 1) return null;

        Pose single = bestApproach(robot, balls, stopShort);
        if (single != null) {
            List<double[]> pts = route(robot.x(), robot.y(), single.x(), single.y());
            double cost = length(pts) + turnCost(robot.heading(), single.heading()) + FieldPlannerConstants.tripOverhead;
            best = new Target(single, null, 1, cost);
        }

        for (int i = 0; i < balls.size(); i++) {
            for (int j = 0; j < balls.size(); j++) {
                if (i == j) continue;
                Pose a = balls.get(i), b = balls.get(j);
                double len = Math.hypot(b.x() - a.x(), b.y() - a.y());
                if (len < 1e-3 || len > FieldPlannerConstants.maxSweepLength) continue;
                double ux = (b.x() - a.x()) / len, uy = (b.y() - a.y()) / len;

                List<double[]> covered = new ArrayList<>();
                for (Pose k : balls) {
                    double t = (k.x() - a.x()) * ux + (k.y() - a.y()) * uy;
                    double side = -(k.x() - a.x()) * uy + (k.y() - a.y()) * ux;
                    if (t >= -0.5 && t <= len + 0.5 && Math.abs(side) <= FieldPlannerConstants.intakeHalfWidth) {
                        covered.add(new double[]{t});
                    }
                }
                Collections.sort(covered, (p, q) -> Double.compare(p[0], q[0]));
                int count = Math.min(covered.size(), limit);
                if (count < 2) continue;
                double tLast = covered.get(count - 1)[0];

                double h = Math.atan2(uy, ux);
                double ex = a.x() - stopShort * ux, ey = a.y() - stopShort * uy;
                double xx = a.x() + (tLast - stopShort) * ux, xy = a.y() + (tLast - stopShort) * uy;
                if (blocked(ex, ey) || !onField(ex, ey) || blocked(xx, xy) || !onField(xx, xy)) continue;
                if (!segmentClear(ex, ey, xx, xy)) continue;
                List<double[]> pts = route(robot.x(), robot.y(), ex, ey);
                if (pts == null) continue;

                double cost = length(pts) + turnCost(robot.heading(), h) + tLast + FieldPlannerConstants.tripOverhead;
                double score = cost / count;
                if (best == null || score < best.score) {
                    best = new Target(new Pose(ex, ey, h), new Pose(xx, xy, h), count, score);
                }
            }
        }
        return best;
    }

    public static Path planTarget(Pose start, Target target) {
        Path toEntry = plan(start, target.entry);
        if (toEntry == null || target.exit == null) return toEntry;
        Path sweep = Paths.line(target.entry, target.exit).constant(target.entry.heading());
        return Paths.path(toEntry, sweep);
    }

    private static double turnCost(double from, double to) {
        return Math.abs(wrap(to - from)) * FieldPlannerConstants.turnCostPerRadian;
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
}