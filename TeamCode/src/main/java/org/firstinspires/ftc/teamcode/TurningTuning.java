package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.SquidPID;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Config
@TeleOp (name ="Turning Tuning")
@Disabled
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
        follower.setStartingPose(new Pose(0,0,90));
    }
    public void loop(){
        output = controller.calculate(follower.getPose().getHeading(), target);
        if (follower.getPose().getHeading() < 180 ){
            robot.setPower(-output,-output, output, output);
        } else {
            robot.setPower(output, output, -output, -output);
        }
        telemetryA.addData("Target", target);
        telemetryA.addData("Current Position", follower.getPose().getHeading());
        follower.update();
    }
}
