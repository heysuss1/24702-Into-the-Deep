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
//    private Telemetry telemetryA;
   public static double kP, kI, kD, kF;

   double extensionError;
   double verticalError;
   public static double kpVert, kIVert, kDVert, kFVert;

   public static int targetVert;
//    public static int armVerticalPos;

//    private double gravityComp;
//    double output;
    public static int setpoint;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    PIDFController pidf;
    PIDFController pidfVert;

    public void init(){
//        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        pidf = new PIDFController(kP, kI, kD, kF);
        pidfVert = new PIDFController(kpVert, kIVert, kDVert, kFVert);
        robot.init(hardwareMap);
    }
    public void loop(){
        extensionError = setpoint - robot.armExtension.getCurrentPosition();
        verticalError = targetVert - robot.armVertical.getCurrentPosition();
        pidfVert.setPIDF(kpVert, kIVert, kDVert, kFVert);
        pidf.setPIDF(kP, kI, kD, kF);
        pidfVert.calculate(0, verticalError);
        pidf.calculate(0, extensionError);
        robot.armVertical.setPower(pidfVert.calculate(0, verticalError));
        robot.armExtension.setPower(pidf.calculate(0, extensionError));


    }
//    public void extend(int target)
//        gravityComp = -0.001 * Math.sin((Math.PI*robot.armVertical.getCurrentPosition())/7600);
//        output = pidf.calculate(robot.armExtension.getCurrentPosition(), target);
//        robot.armExtension.setPower(output);
//    }
}
