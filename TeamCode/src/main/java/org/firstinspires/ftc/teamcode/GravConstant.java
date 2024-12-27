package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "grav constant finder")
public class GravConstant extends OpMode {
    public static double power;
    Hardware robot = Hardware.getInstance();
    Telemetry telemetryA;
    public void init(){
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
    }
    public void loop(){
        robot.armExtension.setPower(power);
        telemetryA.addData("Current Power", power);
        telemetryA.addData("Current Position", robot.armExtension.getCurrentPosition());
    }
}
