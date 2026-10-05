package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Constants.IntakeConstants;

public class Intake implements Subsystem {

    DcMotorEx intakeMotor, transferMotor;
    Servo flowerRamp;
    IntakeStates states;
    public Intake(DcMotorEx intakeMotor, DcMotorEx transferMotor, Servo flowerRamp){
        this.intakeMotor = intakeMotor;
        this.transferMotor = transferMotor;
        this.flowerRamp = flowerRamp;
    }

    public IntakeStates getState(){
        return states;
    }

    public void setState(IntakeStates states){
        this.states = states;

        switch (states){
            case ON:
                intakeMotor.setPower(IntakeConstants.intakeOnPower);
                transferMotor.setPower(IntakeConstants.intakeOnPower);
                break;
            case OFF:
                intakeMotor.setPower(IntakeConstants.intakeOnPower);
                transferMotor.setPower(IntakeConstants.intakeOffPower);
                break;
            case REVERSE:
                intakeMotor.setPower(IntakeConstants.intakeReversePower);
                transferMotor.setPower(IntakeConstants.intakeReversePower);
                break;

        }

    }

    @Override
    public void periodic(){

    }

    public enum IntakeStates {
        ON,
        OFF,
        REVERSE
    }

}
