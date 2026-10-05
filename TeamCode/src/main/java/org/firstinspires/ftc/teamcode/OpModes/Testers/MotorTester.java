package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@Utility
public class MotorTester extends LinearOpMode {

    public static String motorHmName = "";
    public static double motorPower = 0;
    DcMotorEx motor1;

    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.get(DcMotorEx.class, motorHmName);

        waitForStart();
        while (opModeIsActive()){
            motor1.setPower(motorPower);
        }

    }


}
