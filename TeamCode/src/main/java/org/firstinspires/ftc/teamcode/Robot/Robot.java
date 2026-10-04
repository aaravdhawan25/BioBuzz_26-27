package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Susbsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Susbsystems.Shooter;

import java.util.List;

public class Robot {
    public List<LynxModule> hubs;
    Intake intake;
    Shooter shooter;
    DcMotorEx intakeMotor, transferMotor;
    DcMotorEx shooterMotor, shooterMotor2;
    RevColorSensorV3 sensorV3;
    Servo compressionServo;
    String color;
    boolean isAuto;
    public Robot(HardwareMap map, Telemetry tel, String color, boolean isAuto){

        hubs = map.getAll(LynxModule.class);
        for(LynxModule hub : hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        this.color = color;
        this.isAuto = isAuto;
        this.intakeMotor = map.get(DcMotorEx.class, "intakeMotor");
        this.transferMotor = map.get(DcMotorEx.class, "transferMotor");
        this.shooterMotor = map.get(DcMotorEx.class, "shooter");
        this.shooterMotor2 = map.get(DcMotorEx.class, "shooter2");
        this.sensorV3 = map.get(RevColorSensorV3.class, "shooterSensor");
        this.compressionServo = map.get(Servo.class, "compressionServo");
        intake = new Intake(intakeMotor, transferMotor);
        shooter = new Shooter(shooterMotor, shooterMotor2, compressionServo, sensorV3);

    }

    public void init(){
        for (LynxModule hub : hubs){
            hub.clearBulkCache();
        }
    }

    public void update(){
        CommandScheduler.getInstance().run();
    }

}
