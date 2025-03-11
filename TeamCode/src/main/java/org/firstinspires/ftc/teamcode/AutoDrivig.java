package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.intellij.lang.annotations.JdkConstants;

@TeleOp (name = "Auto Driving")
public class AutoDrivig extends LinearOpMode {

    Hardware robot = Hardware.getInstance();
    Follower follower;
    public static Pose pickUpSpecimen2Pose = new Pose(10, 49, Math.toRadians(179));
    public static Pose forwardToSubmersible2Pose = new Pose(30.65, 65, 0);
    public void runOpMode(){
        robot.init(hardwareMap);
        follower = new Follower(hardwareMap);
        waitForStart();
        int x = 0;
        Path toSubmersible = new Path(new BezierLine(new Point(follower.getPose()), new Point(forwardToSubmersible2Pose)));
        while (opModeIsActive()){
            if (x == 0) {
                toSubmersible.setConstantHeadingInterpolation(0);
                follower.followPath(toSubmersible);

                x++;
            }
            follower.update();
        }
    }
}
