package org.firstinspires.ftc.teamcode.HiveVision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.Constants.BallTrackerConstants;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class BallTracker {

    public static final int CLASS_YELLOW_NEUTRAL = 0;
    public static final int CLASS_RED            = 1;
    public static final int CLASS_BLUE           = 2;
    public static final Set<Integer> CLASSES_RED_BLUE = setOf(CLASS_RED, CLASS_BLUE);
    public static final Set<Integer> CLASSES_ALL      = setOf(CLASS_RED, CLASS_BLUE, CLASS_YELLOW_NEUTRAL);

    public static class Sighting {
        public final int    classId;
        public final double txDeg;
        public final double tyDeg;
        public final double confidence;
        public final double distIn;
        public final boolean predicted;

        public Sighting(int classId, double txDeg, double tyDeg, double confidence, boolean predicted, double distIn) {
            this.classId    = classId;
            this.txDeg      = txDeg;
            this.tyDeg      = tyDeg;
            this.confidence = confidence;
            this.predicted  = predicted;
            this.distIn     = distIn;
        }
    }

    public static final class RawDet {
        public final int    classId;
        public final double txDeg, tyDeg, confidence, area;
        public RawDet(int classId, double txDeg, double tyDeg, double confidence, double area) {
            this.classId    = classId;
            this.txDeg      = txDeg;
            this.tyDeg      = tyDeg;
            this.confidence = confidence;
            this.area       = area;
        }
    }

    public interface DetectionSource {
        List<RawDet> latest();
        long stalenessMs();
    }

    private static final class LimelightSource implements DetectionSource {
        private final Limelight3A limelight;
        LimelightSource(Limelight3A limelight) { this.limelight = limelight; }

        @Override public List<RawDet> latest() {
            LLResult r = limelight == null ? null : limelight.getLatestResult();
            List<LLResultTypes.DetectorResult> d = r == null ? null : r.getDetectorResults();
            if (d == null || d.isEmpty()) return java.util.Collections.emptyList();
            List<RawDet> out = new java.util.ArrayList<>(d.size());
            for (LLResultTypes.DetectorResult det : d) {
                out.add(new RawDet(det.getClassId(), det.getTargetXDegrees(),
                        det.getTargetYDegrees(), det.getConfidence(), det.getTargetArea()));
            }
            return out;
        }

        @Override public long stalenessMs() {
            LLResult r = limelight == null ? null : limelight.getLatestResult();
            return r == null ? Long.MAX_VALUE : r.getStaleness();
        }
    }

    private final DetectionSource source;
    private Set<Integer> allowed = new HashSet<>(CLASSES_RED_BLUE);

    private Double lockTx = null, lockTy = null;
    private Integer lockClass = null;
    private final ElapsedTime lockAge = new ElapsedTime();
    private Sighting locked = null;
    private Sighting last = null;

    public BallTracker(Limelight3A limelight) {
        this(new LimelightSource(limelight));
    }

    public BallTracker(DetectionSource source) {
        this.source = source;
    }

    public void setAllowedClasses(Set<Integer> classes) {
        reset();
        this.allowed = new HashSet<>(classes);
    }

    public void setAllowedClasses(Integer... classes) {
        setAllowedClasses(setOf(classes));
    }

    public void addAllowedClass(int classId) {
        allowed.add(classId);
    }

    public boolean isAllowed(int classId) {
        return allowed.contains(classId);
    }

    public Set<Integer> getAllowedClasses() {
        return allowed;
    }

    public void reset() {
        lockClass = null;
        lockTx = null;
        lockTy = null;
        locked = null;
        last = null;
    }

    public Sighting update() {
        last = compute();
        return last;
    }

    public Sighting getLast() {
        return last;
    }

    public long getStalenessMs() {
        return source.stalenessMs();
    }

    public List<RawDet> getGatedDetections() {
        if (source.stalenessMs() > BallTrackerConstants.maxStalenessMs) return null;
        List<RawDet> dets = source.latest();
        if (dets == null || dets.isEmpty()) return null;

        List<RawDet> out = new java.util.ArrayList<>();
        for (RawDet d : dets) {
            if (!allowed.contains(d.classId)) continue;
            if (d.confidence < BallTrackerConstants.minConfidence) continue;
            out.add(d);
        }
        return out;
    }

    public List<RawDet> getConfidentResults() {
        if (source.stalenessMs() > BallTrackerConstants.maxStalenessMs) return null;
        List<RawDet> dets = source.latest();
        if (dets == null || dets.isEmpty()) return null;

        List<RawDet> out = new java.util.ArrayList<>();
        for (RawDet d : dets) {
            if (d.confidence < BallTrackerConstants.minConfidence) continue;
            out.add(d);
        }
        return out;
    }

    public void lockOn(int classId, double txDeg, double tyDeg, double confidence) {
        lockClass = classId;
        lockTx = txDeg;
        lockTy = tyDeg;
        lockAge.reset();
        locked = new Sighting(classId, txDeg, tyDeg, confidence, false, groundRange(tyDeg));
    }

    public double groundRange(double tyDeg) {
        double depression = BallTrackerConstants.camPitchDeg - tyDeg;
        if (depression < 1.0) return 999.0;
        return (BallTrackerConstants.camHeight - BallTrackerConstants.ballHeight) / Math.tan(Math.toRadians(depression));
    }

    private Sighting compute() {
        List<RawDet> dets = source.stalenessMs() > BallTrackerConstants.maxStalenessMs ? null : source.latest();

        if (dets == null || dets.isEmpty()) {
            return coastOrDrop();
        }

        RawDet nearest = null;
        RawDet lockMatch = null;
        double bestJump = Double.MAX_VALUE;

        for (RawDet d : dets) {
            if (!allowed.contains(d.classId)) continue;
            if (lockClass != null && d.classId != lockClass) continue;
            if (d.confidence < BallTrackerConstants.minConfidence) continue;

            if (nearest == null || d.tyDeg < nearest.tyDeg) {
                nearest = d;
            }
            if (lockTx != null) {
                double jump = Math.hypot(d.txDeg - lockTx, d.tyDeg - lockTy);
                if (jump < bestJump) {
                    bestJump = jump;
                    lockMatch = d;
                }
            }
        }

        if (nearest == null) return coastOrDrop();

        if (lockTx != null && lockMatch != null && bestJump <= BallTrackerConstants.lockGateDeg) {
            return adopt(lockMatch);
        }
        if (locked != null && lockAge.milliseconds() <= BallTrackerConstants.lockLostMs) {
            return lockedPredicted();
        }
        return adopt(nearest);
    }

    private Sighting adopt(RawDet d) {
        lockClass = d.classId;
        lockTx = d.txDeg;
        lockTy = d.tyDeg;
        lockAge.reset();
        locked = new Sighting(d.classId, lockTx, lockTy, d.confidence,
                false, groundRange(lockTy));
        return locked;
    }

    private Sighting lockedPredicted() {
        if (locked == null) return null;
        return new Sighting(locked.classId, locked.txDeg, locked.tyDeg, locked.confidence,
                true, locked.distIn);
    }

    private Sighting coastOrDrop() {
        if (locked != null && lockAge.milliseconds() <= BallTrackerConstants.lockLostMs) {
            return lockedPredicted();
        }
        dropLock();
        return null;
    }

    private void dropLock() {
        lockClass = null;
        lockTx = null;
        lockTy = null;
        locked = null;
    }

    private static Set<Integer> setOf(Integer... ids) {
        return new HashSet<>(Arrays.asList(ids));
    }
}