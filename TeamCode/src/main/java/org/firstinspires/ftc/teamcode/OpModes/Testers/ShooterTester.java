package org.firstinspires.ftc.teamcode.OpModes.Testers;

import static org.firstinspires.ftc.teamcode.Utils.Constants.ShooterConstants.kD;
import static org.firstinspires.ftc.teamcode.Utils.Constants.ShooterConstants.kF;
import static org.firstinspires.ftc.teamcode.Utils.Constants.ShooterConstants.kI;
import static org.firstinspires.ftc.teamcode.Utils.Constants.ShooterConstants.kP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utils.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Utils.Telem;


@TeleOp(name = "Shooter Tester")
@Config
public class ShooterTester extends LinearOpMode {

    public static double rpm = 1000;
    DcMotorEx shooterDown;
    DcMotorEx shooterTop;

    @Override
    public void runOpMode() throws InterruptedException {
        Telem.init(telemetry);
        shooterTop = hardwareMap.get(DcMotorEx.class, "TopShooter");
        shooterDown = hardwareMap.get(DcMotorEx.class, "DownShooter");
        shooterTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterTop.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterDown.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isStopRequested()){
            return;
        }

        waitForStart();
        while(opModeIsActive()){
            setShooterPower(rpm);
        }




    }

    public double toRPM(double tps){
        return tps * 60.0 / ShooterConstants.TICKS_PER_REV;
    }

    public void setShooterPower(double rpm){

        double velocity = Math.abs(shooterTop.getVelocity());
        double currentRPM = toRPM(velocity);


        shooterTop.setPower(currentRPM < rpm ? 1 : 0);
        shooterDown.setPower(currentRPM < rpm ? 1 : 0);

        Telem.addData("Shooter Current RPM", currentRPM);
        Telem.addData("Shooter Target RPM", rpm);

    }

}
