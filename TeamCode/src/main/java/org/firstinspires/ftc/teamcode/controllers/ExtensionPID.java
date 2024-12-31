package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
@TeleOp (name = "Extension Tuner")
public class ExtensionPID extends OpMode {
    Hardware robot = Hardware.getInstance();
    private Telemetry telemetryA;
    public static double kP = 0.02;
    public static double kD = 0.015;
    public static double kI, kF;
    public static int armVerticalPos;

    private double gravityComp;
    double output;
    public static int setpoint;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    PIDFController pidf;

    public void init(){
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        pidf = new PIDFController(kP, kD, kI, kF);
        robot.init(hardwareMap);
    }
    public void loop(){
        output = pidf.calculate((robot.armExtension.getCurrentPosition()),  setpoint);
//        gravityComp = -0.001 * Math.sin(Math.PI*armVerticalPos/7600);
        telemetryA.addLine("Error: " + output);
        telemetryA.addLine("current position is: " + robot.armExtension.getCurrentPosition());
        robot.armExtension.setPower(output /*+ gravityComp*/);
        telemetryA.update();
    }
    public void extend(int target){
        gravityComp = -0.001 * Math.sin((Math.PI*robot.armVertical.getCurrentPosition())/7600);
        output = pidf.calculate(robot.armExtension.getCurrentPosition(), target);
        robot.armExtension.setPower(output);
    }
}
