package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous (name = "Ur Mom")
public class ArmVerticalPid extends OpMode {
    private Telemetry telemetryA;
    public static double kP, kI, kD, kF;
    FtcDashboard dashboard = FtcDashboard.getInstance();


    double output;
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public static int setPoint;
//    SquidPID squid = new SquidPID(kP, kI, kD, kF);
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    Hardware robot = Hardware.getInstance();
    public void init(){
        robot.init(hardwareMap);
        output = pidf.calculate(robot.armExtension.getCurrentPosition(), setPoint);

    }
    public void loop(){
        output = pidf.calculate(robot.armExtension.getCurrentPosition(), setPoint);
        while (!pidf.atSetPoint()){
            output = pidf.calculate(robot.armExtension.getCurrentPosition(), setPoint);
            telemetry.addData("Error", output);
            robot.armExtension.setVelocity(output);
            telemetry.update();
            
            requestOpModeStop();
        }
        robot.armExtension.setVelocity(0);
        telemetry.update();

    }


}
