package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous (name = "Test Auto")
public class TestAuto extends OpMode {
    Follower follower;
    Hardware robot = Hardware.getInstance();
    Pose starting = new Pose(5, 65);
    enum State{
        GO_TO_BAR,
        HANG_SPECIMEN

    }

    State state = State.GO_TO_BAR;
//    PathBuilder toSubmersible;

    //Assumes robot starts at (5, 65)
    public void init(){
        follower = new Follower(hardwareMap);
        robot.init(hardwareMap);
        follower.setStartingPose(starting);

        follower.followPath(path());

    }

    public static PathChain path(){
        PathBuilder builder = new PathBuilder();
        builder.addPath((new BezierLine(new Point(5, 65, Point.CARTESIAN), new Point(31, 65, Point.CARTESIAN))))
                .setTangentHeadingInterpolation();
//                .addPath(new BezierLine(new Point(40, 65, Point.CARTESIAN), new Point(5, 65, Point.CARTESIAN)))
//                .setTangentHeadingInterpolation();
//                .addPath(new BezierLine(new Point(46, 30, Point.CARTESIAN), new Point(56, 24, Point.CARTESIAN)));
        return builder.build();
    }



    public void doAuto(){
        switch (state){
            case GO_TO_BAR:

                break;
            default:
                stop();




        }
    }
    @Override
    public void loop(){
        telemetry.addData("X Position", follower.getPose());
        telemetry.addData("Position", robot.armExtension.getCurrentPosition());
        robot.armExtension.setPower(0.8);
        robot.armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armExtension.setTargetPosition(-1200);
        robot.armExtension.setPower(0.8);
        if (robot.armExtension.getCurrentPosition() < 1199) {
            robot.armExtension.setPower(0);
        }
        follower.update();



    }
}
