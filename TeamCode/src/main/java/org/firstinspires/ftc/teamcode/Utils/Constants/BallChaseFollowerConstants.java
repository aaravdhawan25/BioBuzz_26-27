package org.firstinspires.ftc.teamcode.Utils.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BallChaseFollowerConstants {

    public static double maxPlanRange = 40.0;
    public static double mergeRadius = 4.0;
    public static double approachDist = 24.0;
    public static double clearRadius = 12.0;
    public static double fieldMin = 6.0;
    public static double fieldMax = 138.0;
    public static long travelTimeoutMs = 4000;

    public static long settleMs = 250;
    public static long scanMs = 250;
    public static double searchStepDeg = 45.0;
    public static int searchMaxSteps = 8;

    public static double turnToKp = 0.8;
    public static double turnToMin = 0.12;
    public static double turnToMax = 0.5;
    public static double turnToTolDeg = 4.0;
    public static long turnTimeoutMs = 2500;

    public static double stopDist = 16.0;
    public static double aimTolDeg = 5.0;
    public static double driveMinTx = 15.0;
    public static double driveKp = 0.03;
    public static double minFwd = 0.15;
    public static double maxFwd = 0.7;
    public static double turnKp = 0.025;
    public static double minTurn = 0.08;
    public static double maxTurn = 0.6;
    public static double coastMaxDist = 26.0;
    public static double coastPower = 0.25;
    public static long coastMs = 450;
    public static long lostMs = 500;
    public static long pickupDwellMs = 600;

}