package org.firstinspires.ftc.teamcode.Utils.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class FieldPlannerConstants {

    public static double obstacleMinX = 46.5;
    public static double obstacleMaxX = 98.0;
    public static double obstacleMinY = 47.5;
    public static double obstacleMaxY = 98.0;

    public static double robotRadius = 13.0;
    public static double safetyMargin = 2.0;
    public static double fieldMin = 9.0;
    public static double fieldMax = 135.0;

    public static double cornerExtra = 4.0;
    public static double filletRadius = 14.0;
    public static int filletTries = 6;
    public static int curveSamples = 80;
    public static int approachDirections = 16;

    public static double turnCostPerRadian = 4.0;

    public static double stopShort = 10.0;

}