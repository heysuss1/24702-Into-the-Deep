package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp (name = "Extension Tuner")
public class ArmExtension extends OpMode {
    Hardware robot = Hardware.getInstance();
    private Telemetry telemetryA;
    public static double kP = 0.001;
    public static double kD = 0.00112;
    public static double kI, kF;

    private double gravityComp;
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
        output = squid.calculate((robot.armExtension.getCurrentPosition()),  target);
        gravityComp = -0.001 * Math.sin((Math.PI*robot.armExtension.getCurrentPosition())/7600);
        telemetryA.addLine("Error: " + output);
        telemetryA.addLine("current position is: " + robot.armExtension.getCurrentPosition());
        robot.armExtension.setPower(output + gravityComp);
        telemetryA.update();

    }
}
