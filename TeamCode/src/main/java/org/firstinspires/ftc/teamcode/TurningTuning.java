package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Config
@TeleOp (name ="Turning Tuning")
public class TurningTuning extends OpMode {
    Hardware robot = Hardware.getInstance();
    private Telemetry telemetryA;
    public static double kP = 0.0031, kD = 0.0028, kI = 0, kF = 0;
    Follower follower;
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
        output = squid.calculate((Math.toDegrees(robot.odo.getHeading())),  target);
        telemetryA.addLine("Error: " + output);
        telemetryA.addLine("current position is: " + Math.toDegrees(robot.odo.getHeading()));
        robot.setPower(output, output, -output, -output);
        telemetryA.update();
        robot.odo.update();

    }
}
