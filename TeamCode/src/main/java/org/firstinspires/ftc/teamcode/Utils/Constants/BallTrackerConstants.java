package org.firstinspires.ftc.teamcode.Utils.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BallTrackerConstants {

    public static double camPitchDeg = 25.0;
    public static double camHeight = 9.2;
    public static double ballHeight = 3.0;
    public static double camXOffset = 0.0;
    public static double camYOffset = 0.0;

    public static double hfovDeg = 54.5;
    public static double vfovDeg = 42.0;
    public static double fovMarginDeg = 4.0;

    public static double minConfidence = 0.44;
    public static long maxStalenessMs = 120;

    public static double lockGateDeg = 12.0;
    public static long lockLostMs = 300;

}