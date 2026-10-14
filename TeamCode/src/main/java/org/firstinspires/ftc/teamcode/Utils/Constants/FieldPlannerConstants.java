package org.firstinspires.ftc.teamcode.Utils.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class FieldPlannerConstants {

    public static double obstacleMinX = 45.68;
    public static double obstacleMaxX = 95.82;
    public static double obstacleMinY = 51.24;
    public static double obstacleMaxY = 90.62;

    public static double robotRadius = 9.9;
    public static double safetyMargin = 2.0;
    public static double fieldMin = 7.0;
    public static double fieldMax = 134.5;

    public static double cornerExtra = 4.0;
    public static double filletRadius = 14.0;
    public static int filletTries = 6;
    public static int curveSamples = 80;
    public static int approachDirections = 16;

    public static double turnCostPerRadian = 4.0;

    public static double stopShort = 10.0;
    public static double intakeHalfWidth = 5.0;
    public static int intakeCapacity = 3;
    public static double maxSweepLength = 36.0;
    public static double tripOverhead = 12.0;


}