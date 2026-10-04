package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static org.firstinspires.ftc.teamcode.Utils.Constants.ShooterConstants.kD;
import static org.firstinspires.ftc.teamcode.Utils.Constants.ShooterConstants.kF;
import static org.firstinspires.ftc.teamcode.Utils.Constants.ShooterConstants.kI;
import static org.firstinspires.ftc.teamcode.Utils.Constants.ShooterConstants.kP;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utils.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Utils.Constants.V3Constants;
import org.firstinspires.ftc.teamcode.Utils.Telem;

public class Shooter implements Subsystem {

    DcMotorEx shooterMotor, shooterMotor2;
    RevColorSensorV3 sensorV3;
    Servo compressionControl;
    ShooterStates shooterStates;

    CompressionStates compressionStates;
    PIDFController shooterPIDController;
    public Shooter(DcMotorEx shooterMotor, DcMotorEx shooterMotor2, Servo compressionControl, RevColorSensorV3 sensorV3){
        this.shooterMotor = shooterMotor;
        this.shooterMotor2 = shooterMotor2;
        this.compressionControl = compressionControl;
        this.sensorV3 = sensorV3;
        this.shooterPIDController = new PIDFController(kP, kI, kD, kF);
        this.shooterPIDController.setTolerance(10);
    }

    public ShooterStates getShooterState(){
        return shooterStates;
    }

    public CompressionStates getCompressionState(){
        return compressionStates;
    }

    public void setState(ShooterStates states){
        this.shooterStates = states;

        switch (states){


        }

    }

    public void setState(CompressionStates state){
        this.compressionStates = state;

        switch (state){
            case NECTAR:
                setCompressionControl(ShooterConstants.nectarPose);
                break;

            case POLLEN:
                setCompressionControl(ShooterConstants.pollenPose);
                break;

        }

    }

    @Override
    public void periodic(){
        classifyBall();
        setState(shooterStates);
    }

    public double toRPM(double tps){
        return tps * 60.0 / ShooterConstants.TICKS_PER_REV;
    }

    public boolean atTargetSpeed(){
        return shooterPIDController.getPositionError() <= 100;
    }

    public void setCompressionControl(double pos){
        compressionControl.setPosition(pos);
    }

    public void classifyBall() {
        CompressionStates lastState = getCompressionState();
        int r = sensorV3.red(), g = sensorV3.green(), b = sensorV3.blue();
        double total = r + g + b;

        double rf = r / total, gf = g / total, bf = b / total;

        if(total < V3Constants.totalRGB){
            setState(lastState);
        }

        if (rf > V3Constants.redFractionPollen && gf > V3Constants.greenFractionPollen){
            setState(CompressionStates.POLLEN);
        }
        if (rf > V3Constants.redFractionNectar || bf > V3Constants.blueFractionNectar) {
            setState(CompressionStates.NECTAR);
        }

    }

    public void setShooterPower(double rpm){

        double velocity = Math.abs(shooterMotor.getVelocity());
        double currentRPM = toRPM(velocity);

        shooterPIDController.setPIDF(kP, kI, kD, kF);
        double power = shooterPIDController.calculate(currentRPM, rpm);

        power += (rpm > 0) ? (ShooterConstants.kF * (rpm / ShooterConstants.MAX_RPM)) : 0.0;
        power = Range.clip(power, 0, 1);

        shooterMotor.setPower(power);
        shooterMotor2.setPower(power);

        Telem.addData("Shooter Current RPM", currentRPM);
        Telem.addData("Shooter Target RPM", rpm);

    }

    public enum ShooterStates {
        NORMAL,
        MATH,
        OFF,
        REVERSE
    }

    public enum CompressionStates {
        POLLEN,
        NECTAR
    }

}
