package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Blocker;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.Constants.BotConstants;
import org.firstinspires.ftc.teamcode.Utils.Constants.HardwareMapNames;
import org.firstinspires.ftc.teamcode.Utils.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Utils.Telem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

public class Robot {
    public List<LynxModule> hubs;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Follower follower;
    public Blocker blocker;
    DcMotorEx intakeMotor, transferMotor;
    DcMotorEx shooterMotor, shooterMotor2;
    RevColorSensorV3 sensorV3;
    Servo flowerRamp;
    Servo compressionServo, linearServo;
    Servo blockerServo;
    Servo turretServoOne, turretServoTwo;
    String color;
    boolean isAuto;
    boolean holding;
    public static boolean startSide;
    public static boolean isRed;
    public static Pose currentPose = new Pose(0,0,0);
    public Robot(HardwareMap map, Telemetry tel, String color, boolean isAuto){

        hubs = map.getAll(LynxModule.class);
        for(LynxModule hub : hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        this.color = color;
        this.isAuto = isAuto;
        this.isRed = color.equals("RED");
        this.startSide = true;
        this.holding = false;

        this.intakeMotor = map.get(DcMotorEx.class, HardwareMapNames.intakeMotorConfig);
        this.transferMotor = map.get(DcMotorEx.class, HardwareMapNames.transferMotorConfig);
        this.shooterMotor = map.get(DcMotorEx.class, HardwareMapNames.shooterMotorConfig);
        this.shooterMotor2 = map.get(DcMotorEx.class, HardwareMapNames.shooterMotor2Config);
        this.sensorV3 = map.get(RevColorSensorV3.class, HardwareMapNames.shooterSensorConfig);
        this.flowerRamp = map.get(Servo.class, HardwareMapNames.flowerRampConfig);
        this.compressionServo = map.get(Servo.class, HardwareMapNames.compressionServoConfig);
        this.linearServo = map.get(Servo.class, HardwareMapNames.slidingTurretConfig);
        this.blockerServo = map.get(Servo.class, HardwareMapNames.blockerServoConfig);
        this.turretServoOne = map.get(Servo.class, HardwareMapNames.turretServo1Config);
        this.turretServoTwo = map.get(Servo.class, HardwareMapNames.turretServo2Config);

        intake = new Intake(intakeMotor, transferMotor, flowerRamp);
        shooter = new Shooter(shooterMotor, shooterMotor2, compressionServo, linearServo, sensorV3);
        blocker = new Blocker(blockerServo);
        turret = new Turret(turretServoOne, turretServoTwo);
        follower = Constants.create(map);

        Telem.init(tel);
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(intake, shooter, blocker, turret);
    }

    public void update(){
        CommandScheduler.getInstance().run();
        follower.update();
        currentPose = follower.pose();

        if(intake != null)
            Telem.addData("Intake State", intake.getState());
        if(shooter != null){
            Telem.addData("Shooter State", shooter.getShooterState());
            Telem.addData("Shooter Compression State", Shooter.getCompressionState());
        }
        if(turret != null)
            Telem.addData("Turret State", turret.getState());

        Telem.update();
        for (LynxModule hub : hubs){
            hub.clearBulkCache();
        }
    }

    public void stop(){
        Pose pose = follower.pose();
        CommandScheduler.getInstance().reset();
        for (LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        Robot.currentPose = pose;
    }

    public void flipGoal(){
        startSide = !startSide;
    }

    public void resetPosition(){
        follower.setPose(PoseFactory.degrees().of(8.483, 8.263, 90));
    }

    public static double getDistanceFromGoal(Pose pose){
        Pose goalPose = Robot.getGoalPose();

        double dX = pose.x() - goalPose.x();
        double dY = pose.y() - goalPose.y();

        return Math.hypot(dX, dY);
    }

    public static Pose getCurrentPosition(){
        return currentPose;
    }

    public static Pose getGoalPose(){
        if (isRed && startSide){
            return new Pose(BotConstants.redGoalPoseX,BotConstants.redGoalPoseStartY);
        }
        if (isRed && !startSide){
            return new Pose(BotConstants.redGoalPoseX, BotConstants.redGoalPoseTipY);
        }
        if (!isRed && !startSide){
            return new Pose(BotConstants.blueGoalPoseX, BotConstants.blueGoalPoseTipY);
        }

        return new Pose(BotConstants.blueGoalPoseX, BotConstants.blueGoalPoseStartY);
    }

    public static Pose getTurretPosition(Pose cur) {
        double heading = cur.heading();
        double turretX = (Shooter.getCompressionState() == Shooter.CompressionStates.NECTAR) ? (cur.x()
                + TurretConstants.turretXOFFSET * Math.cos(heading)
                - TurretConstants.turretYOFFSET * Math.sin(heading)) : cur.x();
        double turretY = (Shooter.getCompressionState() == Shooter.CompressionStates.NECTAR) ? (cur.y()
                + TurretConstants.turretXOFFSET * Math.sin(heading)
                + TurretConstants.turretYOFFSET * Math.cos(heading)) : cur.y();

        return new Pose(turretX, turretY, heading);
    }
}
