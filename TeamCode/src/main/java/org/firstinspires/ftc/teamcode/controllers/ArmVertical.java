package org.firstinspires.ftc.teamcode.controllers;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
@TeleOp (name = "turning")
public class ArmVertical extends OpMode {
    Hardware robot = Hardware.getInstance();
    private Telemetry telemetryA;
    public static double kP = 0.0031, kD = 0.0028, kI = 0, kF = 0;
    double output;
    public static int target;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SquidPID squid;
    public void init(){
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        robot.init(hardwareMap);
        squid = new SquidPID(kP, kD, kI, kF);
    }
    public void loop(){
        output = squid.calculate((robot.armVertical.getCurrentPosition()),  target);
        telemetryA.addLine("Error: " + output);
        telemetryA.addLine("current position is: " + robot.armVertical.getCurrentPosition());
        robot.armVertical.setPower(output);

        telemetryA.update();

    }
}