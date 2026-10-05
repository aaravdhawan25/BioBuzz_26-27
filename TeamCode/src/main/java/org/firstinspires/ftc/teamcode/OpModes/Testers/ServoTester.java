package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@Utility
public class ServoTester extends LinearOpMode {

    public static String servo1HmName = "", servo2HmName = "";
    public static double servoPos = 0;
    private static Servo servo1;
    private static Servo servo2;

    public static ServoTestState state = ServoTestState.SINGLE;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(Servo.class, servo1HmName);

        waitForStart();
        while (opModeIsActive()){
            setState(state, hardwareMap);
        }

    }

    public static void setState(ServoTestState state, HardwareMap hm){
        switch (state){
            case SINGLE:
                servo1.setPosition(servoPos);
                break;
            case DUAL:
                servo2 = hm.get(Servo.class, servo2HmName);
                servo1.setPosition(servoPos);
                servo2.setPosition(servoPos);
                break;
        }
    }

    public enum ServoTestState{
        SINGLE,
        DUAL
    }

}
