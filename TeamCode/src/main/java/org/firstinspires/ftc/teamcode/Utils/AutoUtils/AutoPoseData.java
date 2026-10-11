package org.firstinspires.ftc.teamcode.Utils.AutoUtils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.Utils.AutoUtils.AutoConstants;

@Config
public class AutoPoseData {

    public static Pose startPose = new Pose(AutoConstants.startX, AutoConstants.startY, Math.toRadians(AutoConstants.startHeading));

    public static Pose gardenPollenPose = new Pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, Math.toRadians(AutoConstants.gardenPollenHeading));
    public static Pose startToGardenControl = new Pose(AutoConstants.startToGardenContX, AutoConstants.startToGardenContY, 0);
    public static Pose gardenTurnSeg1Start = new Pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, Math.toRadians(AutoConstants.gardenTurnSeg1StartHeading));
    public static Pose gardenTurnSeg1End = new Pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, Math.toRadians(AutoConstants.gardenTurnSeg1EndHeading));
    public static Pose gardenTurnSeg2Start = new Pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, Math.toRadians(AutoConstants.gardenTurnSeg2StartHeading));
    public static Pose gardenTurnSeg2End = new Pose(AutoConstants.gardenPollenX, AutoConstants.gardenPollenY, Math.toRadians(AutoConstants.gardenTurnSeg2EndHeading));

    public static Pose hiveLaunchPose = new Pose(AutoConstants.hiveLaunchX, AutoConstants.hiveLaunchY, Math.toRadians(AutoConstants.hiveLaunchHeading));
    public static Pose gardenToLaunchControl1 = new Pose(AutoConstants.gardenToLaunchCont1X, AutoConstants.gardenToLaunchCont1Y, 0);
    public static Pose gardenToLaunchControl2 = new Pose(AutoConstants.gardenToLaunchCont2X, AutoConstants.gardenToLaunchCont2Y, 0);
    public static Pose launchTurnStart = new Pose(AutoConstants.hiveLaunchX, AutoConstants.hiveLaunchY, Math.toRadians(AutoConstants.launchTurnStartHeading));
    public static Pose launchTurnEnd = new Pose(AutoConstants.hiveLaunchX, AutoConstants.hiveLaunchY, Math.toRadians(AutoConstants.launchTurnEndHeading));

    public static Pose launchDepartPose = new Pose(AutoConstants.hiveLaunchX, AutoConstants.hiveLaunchY, Math.toRadians(AutoConstants.launchDepartHeading));
    public static Pose flowerPose = new Pose(AutoConstants.flowerX, AutoConstants.flowerY, Math.toRadians(AutoConstants.flowerHeading));
    public static Pose launchToFlowerControl = new Pose(AutoConstants.launchToFlowerContX, AutoConstants.launchToFlowerContY, 0);

    public static Pose loadingZoneParkPose = new Pose(AutoConstants.loadingZoneParkX, AutoConstants.loadingZoneParkY, Math.toRadians(AutoConstants.loadingZoneParkHeading));
    public static Pose flowerToParkControl1 = new Pose(AutoConstants.flowerToParkCont1X, AutoConstants.flowerToParkCont1Y, 0);
    public static Pose flowerToParkControl2 = new Pose(AutoConstants.flowerToParkCont2X, AutoConstants.flowerToParkCont2Y, 0);

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