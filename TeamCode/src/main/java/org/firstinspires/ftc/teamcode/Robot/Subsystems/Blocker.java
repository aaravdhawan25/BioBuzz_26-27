package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Constants.BlockerConstants;

public class Blocker implements Subsystem {
    Servo blockerServo;
    BlockerState state;

    public Blocker(Servo blockerServo){
        this.blockerServo = blockerServo;
    }

    public BlockerState getState(){
        return state;
    }

    public void setState(BlockerState state){
        this.state = state;

        switch (state){
            case CLOSED:
                blockerServo.setPosition(BlockerConstants.blockerClosed);
                break;
            case OPEN:
                blockerServo.setPosition(BlockerConstants.blockerOpen);
                break;
        }

    }

    @Override
    public void periodic(){

    }

    public enum BlockerState {
        CLOSED,
        OPEN
    }

}
