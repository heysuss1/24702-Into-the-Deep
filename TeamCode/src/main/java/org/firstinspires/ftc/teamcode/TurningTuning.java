package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Config
@TeleOp (name ="Turning Tuning")
public class TurningTuning extends OpMode {
    Hardware robot = Hardware.getInstance();
    private Telemetry telemetryA;
    public static double kP;
    Follower follower;
    PController controller;
    double output;
    public static int target;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SquidPID squid;
    public void init(){
        robot.init(hardwareMap);
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        controller = new PController(kP);
        follower = new Follower(hardwareMap);
    }
    public void loop(){
        output = controller.calculate(follower.getPose().getHeading(), target);
        if (follower.getPose().getHeading() < 180 ){
            robot.setPower(-output,-output, output, output);
        } else {
            robot.setPower(output, output, -output, -output);
        }

    }

}
