package org.firstinspires.ftc.teamcode.Utils.AutoUtils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

@Config
public class AutoPoseData {

    // Just Start Pose
    public static Pose startPose = new Pose(AutoConstants.startX, AutoConstants.startY, AutoConstants.startHeading);

    // Path 1
    public static Pose gardenPollenPose = new Pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, AutoConstants.gardenPollenHeading);
    public static Pose startToGardenControl = new Pose(AutoConstants.startToGardenContX, AutoConstants.startToGardenContY, 0);
    public static Pose gardenTurnSeg1Start = new Pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, AutoConstants.gardenTurnSeg1StartHeading);
    public static Pose gardenTurnSeg1End = new Pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, AutoConstants.gardenTurnSeg1EndHeading);
    public static Pose gardenTurnSeg2Start = new Pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, AutoConstants.gardenTurnSeg2StartHeading);
    public static Pose gardenTurnSeg2End = new Pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, AutoConstants.gardenTurnSeg2EndHeading);

    // Path 2
    public static Pose hiveLaunchPose1 = new Pose(AutoConstants.hiveLaunch1X, AutoConstants.hiveLaunch1Y, AutoConstants.hiveLaunch1Heading);
    public static Pose gardenToLaunch1Control = new Pose(AutoConstants.gardenToLaunch1ContX, AutoConstants.gardenToLaunch1ContY, 0);
    public static Pose launch1TurnStart = new Pose(AutoConstants.hiveLaunch1X, AutoConstants.hiveLaunch1Y, AutoConstants.launch1TurnStartHeading);
    public static Pose launch1TurnEnd = new Pose(AutoConstants.hiveLaunch1X, AutoConstants.hiveLaunch1Y, AutoConstants.launch1TurnEndHeading);

    // Path 3
    public static Pose flowerPose = new Pose(AutoConstants.flowerX, AutoConstants.flowerY, AutoConstants.flowerHeading);
    public static Pose launch1ToFlowerControl = new Pose(AutoConstants.launch1ToFlowerContX, AutoConstants.launch1ToFlowerContY, 0);

    // Path 4
    public static Pose flowerDepartPose = new Pose(AutoConstants.flowerX, AutoConstants.flowerY, AutoConstants.flowerDepartHeading);
    public static Pose hiveLaunchPose2 = new Pose(AutoConstants.hiveLaunch2X, AutoConstants.hiveLaunch2Y, AutoConstants.hiveLaunch2Heading);
    public static Pose flowerToLaunch2Control = new Pose(AutoConstants.flowerToLaunch2ContX, AutoConstants.flowerToLaunch2ContY, 0);

    // Path 5
    public static Pose loadingZoneParkPose = new Pose(AutoConstants.loadingZoneParkX, AutoConstants.loadingZoneParkY, AutoConstants.loadingZoneParkHeading);
    public static Pose launch2ToParkControl1 = new Pose(AutoConstants.launch2ToParkCont1X, AutoConstants.launch2ToParkCont1Y, 0);
    public static Pose launch2ToParkControl2 = new Pose(AutoConstants.launch2ToParkCont2X, AutoConstants.launch2ToParkCont2Y, 0);

    public static double mirrorX(double x, String color) {
        return color.equals("BLUE") ? 141.5 - x : x;
    }
    public static double mirrorY(double y, String color) {
        return color.equals("BLUE") ? 141.5 - y : y;
    }
    public static double mirrorHeading(double deg, String color) {
        return color.equals("BLUE") ? deg + 180 : deg;
    }
    public static Pose mirror(Pose p, String color) {
        return color.equals("BLUE") ? new Pose(141.5 - p.x(), 141.5 - p.y(), Math.toRadians(Math.toDegrees(p.heading()) + 180)) : p;
    }
}