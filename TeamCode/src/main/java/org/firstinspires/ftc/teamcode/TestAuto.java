package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

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
    enum ActionState{
        RAISE_ARMS,
        HANG_SPECIMEN

    }

    State state = State.GO_TO_BAR;
    ActionState actionState = ActionState.RAISE_ARMS;
//    PathBuilder toSubmersible;

    //Assumes robot starts at (5, 65)
    public void init(){
        follower = new Follower(hardwareMap);
        robot.init(hardwareMap);
        follower.setStartingPose(starting);
        robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armVertical.setTargetPosition(1200);
        robot.armVertical.setPower(0.8);
        follower.followPath(path());
    }

    public static PathChain path(){
        PathBuilder builder = new PathBuilder();
        builder.addPath((new BezierLine(new Point(5, 65, Point.CARTESIAN), new Point(31.6, 65, Point.CARTESIAN))))
                .setTangentHeadingInterpolation();
//                .addPath(new BezierLine(new Point(40, 65, Point.CARTESIAN), new Point(5, 65, Point.CARTESIAN)))
//                .setTangentHeadingInterpolation();
//                .addPath(new BezierLine(new Point(46, 30, Point.CARTESIAN), new Point(56, 24, Point.CARTESIAN)));
        return builder.build();
    }


    public void setPathState(State prevState){
        state = prevState;
        autonomousPathUpdate();
    }
    public void autonomousPathUpdate(){
        switch (state){
            case GO_TO_BAR:
                follower.followPath(path());
                setPathState(State.HANG_SPECIMEN);
                break;
            default:
                stop();


        }
    }

    public void setAction(ActionState prevState){
        actionState = prevState;
        autonomousActionUpdate();
    }
    public void autonomousActionUpdate(){
        switch (actionState){
            case RAISE_ARMS:
                robot.leftServo.setPosition(.69);
                robot.rightServo.setPosition(.1);
                robot.armExtension.setPower(1);
                robot.armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armExtension.setTargetPosition(-1200);
                robot.armVertical.setPower(1);
                robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armVertical.setTargetPosition(2300);
                if (robot.armExtension.getCurrentPosition() < -1199 && robot.armVertical.getCurrentPosition() > 2290) {
                    setAction(ActionState.HANG_SPECIMEN);
                }
                break;
            case HANG_SPECIMEN:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    robot.armExtension.setPower(1);
                    robot.armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armExtension.setTargetPosition(-350);
                    robot.armVertical.setPower(1);
                    robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armVertical.setTargetPosition(2100);
                }

//                if (robot.armExtension.getCurrentPosition() < - && robot.armVertical.getCurrentPosition() > ) {
//                    setAction(ActionState.HANG_SPECIMEN);
//                }
                break;
            default:
                stop();

        }
    }

//    public void

    @Override
    public void loop(){
        telemetry.addData("X Position", follower.getPose());
        telemetry.addData("Position", robot.armExtension.getCurrentPosition());
        telemetry.addData("Arm Position", robot.armVertical.getCurrentPosition());
        autonomousPathUpdate();
        autonomousActionUpdate();
        follower.update();



    }
}
