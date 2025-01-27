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
@TeleOp (name = "turning")
public class VerticalPID extends OpMode {
    Hardware robot = Hardware.getInstance();
    private Telemetry telemetryA;
    public static double kP = 0.0219, kD = 0.0001, kI = 0, kF = 0;
    double output, armOutput;

    PIDFController extensionPID;
    public static int setpoint;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    PIDFController pidf;
    boolean hi = false;

    public void init(){
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        robot.init(hardwareMap);
        pidf = new PIDFController(kP, kI, kD, kF);
        extensionPID = new PIDFController(0.02, 0, 0.0001, 0);

    }
    public void loop(){
        if (gamepad2.y){
            hi = true;
        }

        if (hi){
                output = pidf.calculate((robot.armVertical.getCurrentPosition()),  -2000);
            telemetryA.addLine("Error: " + output);
            telemetryA.addLine("current position is: " + robot.armVertical.getCurrentPosition());
            armOutput = extensionPID.calculate(robot.armExtension.getCurrentPosition(), -1200);
            robot.armVertical.setPower(output);
            robot.armExtension.setPower(armOutput);
            telemetryA.addLine("Extension Error: " + armOutput);
            telemetryA.addLine("current extension position is: " + robot.armExtension.getCurrentPosition());
        }

        telemetryA.update();

    }
    public void raise(int target){
//        output = squid.calculate(robot.armVertical.getCurrentPosition(), target);
//        robot.armVertical.setPower(output);
    }
}