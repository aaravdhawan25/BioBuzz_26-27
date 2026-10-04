package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Telem {
    public static MultipleTelemetry telemetry;

    public static void init(Telemetry tel){
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), tel);
    }

    public static void addData(String str, Object obj){
        telemetry.addData(str, obj);
    }
    public static void addLine(String str){
        telemetry.addLine(str);
    }

    public static void addData(String str, String format, Object args){
        telemetry.addData(str,format,args);
    }

    public static void update(){
        telemetry.update();
    }


}
