package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Constants.IntakeConstants;

public class Intake implements Subsystem {

    DcMotorEx intakeMotor, transferMotor;
    Servo flowerRamp;
    IntakeStates states;
    RampStates state;
    public Intake(DcMotorEx intakeMotor, DcMotorEx transferMotor, Servo flowerRamp){
        this.intakeMotor = intakeMotor;
        this.transferMotor = transferMotor;
        this.flowerRamp = flowerRamp;
    }

    public IntakeStates getState(){
        return states;
    }
    public RampStates getRampState(){
        return state;
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

    public void setState(RampStates state){
        this.state = state;

        switch (state){
            case DEPLOYED:
                flowerRamp.setPosition(IntakeConstants.rampDeployedPosition);
                break;
            case RETRACTED:
                flowerRamp.setPosition(IntakeConstants.rampRetractedPosition);
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

    public enum RampStates {
        DEPLOYED,
        RETRACTED
    }

}
