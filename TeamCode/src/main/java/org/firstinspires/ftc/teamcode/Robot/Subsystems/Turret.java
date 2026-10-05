package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Utils.Constants.TurretConstants;

public class Turret implements Subsystem {

    Servo turretServoOne, turretServoTwo;
    TurretState state;

    public Turret(Servo turretServoOne, Servo turretServoTwo){
        this.turretServoOne = turretServoOne;
        this.turretServoTwo = turretServoTwo;
    }

    public TurretState getState(){
        return state;
    }

    public void setState(TurretState state){
        this.state = state;
        switch (state){
            case FORWARD:
                setServoTurretPosition(TurretConstants.turretForwardPos);
                break;
            case MATH:
                pointToGoal(
                        Robot.getTurretPosition(
                                Robot.getCurrentPosition()
                        )
                );
                break;
        }
    }

    @Override
    public void periodic(){
        setState(state);
    }

    public void pointToGoal(Pose cur) {
        Pose hive = Robot.getGoalPose();
        double fieldRelativeAngle = Math.atan2(
                hive.y() - cur.y(),
                hive.x() - cur.x()
        );

        double robotRelativeAngle = fieldRelativeAngle - cur.heading() - Math.PI;

        while (robotRelativeAngle > Math.PI) robotRelativeAngle -= 2 * Math.PI;
        while (robotRelativeAngle < -Math.PI) robotRelativeAngle += 2 * Math.PI;

        double degAngle = Math.toDegrees(robotRelativeAngle);
        double servoPosition = TurretConstants.OFFSET + TurretConstants.slopeMultiplier * degAngle;

    }

    public void setServoTurretPosition(double pos){
        pos = Range.clip(pos, 0, 1);
        turretServoOne.setPosition(pos);
        turretServoTwo.setPosition(pos);
    }

    public enum TurretState {
        FORWARD,
        MATH
    }


}
