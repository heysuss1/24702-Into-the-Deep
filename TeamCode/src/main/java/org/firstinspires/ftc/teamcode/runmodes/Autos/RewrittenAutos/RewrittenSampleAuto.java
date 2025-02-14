package org.firstinspires.ftc.teamcode.runmodes.Autos.RewrittenAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto;
import org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto;
import org.firstinspires.ftc.teamcode.runmodes.Autos.SamplesAuto;


@Autonomous(name = "Rewritten Class")
public class RewrittenSampleAuto extends OpMode {
    Follower follower;
    Hardware robot = Hardware.getInstance();
    Pose starting = new Pose(5, 126, 0);
    Timer timer;
    int pitch, roll;
    int clawCloser = 0;
    int ARM_CONSTANT = 570;
    public static Pose startPose = new Pose(5, 126, Math.toRadians(0));
    public static Pose preloadPose = new Pose(18, 127, Math.toRadians(0));
    public static Pose backUpPose = new Pose(12, 127, Math.toRadians(0));
    public static Pose sample1Pose = new Pose(4.4, 109.5, Math.toRadians(130));
    public static Pose bucketPose = new Pose (10.4, 122, Math.toRadians(-130));
    public static Pose sample2Pose = new Pose (12.8, 109, Math.toRadians(-90));
    public static Pose sample3Pose = new Pose (22.7, 88, Math.toRadians(0));
    public static Pose parkingPose= new Pose (-6, 80, Math.toRadians(-179));
    boolean clawClosed = false;

    ElapsedTime armTimer;
    int counter = 0;

    boolean isClawClosed;

    enum State{
        SCORE_PRELOAD_BASKET,
        GO_BACKWARDS,
        GO_TO_SAMPLE1,
        GO_TO_BASKET,
        GO_TO_SAMPLE2,
        GO_TO_BASKET_FROM_SAMPLE_2,
        GO_TO_SAMPLE3,
        GO_TO_BASKET_FROM_SAMPLE_3,
        GO_TO_PARKING

    }

    enum ActionState{
        RAISE_ARMS,
        SCORE_SAMPLE,
        OPEN_CLAW,
        GRAB_SAMPLE1,
        CLOSE_CLAW,
        PUT_IN_BUCKET,
        OPEN_CLAW_2,
        GRAB_SAMPLE2,
        CLOSE_CLAW_2,
        PUT_IN_BUCKET_2,
        GRAB_SAMPLE3,
        CLOSE_CLAW_3,
        PUT_IN_BUCKET_3,
        PARK
    }
    enum ClawUpdate{
        CLOSE_SAMPLE_1,
    }
    State state = State.SCORE_PRELOAD_BASKET;
    ActionState actionState = ActionState.RAISE_ARMS;
//    PathBuilder toSubmersible;

    //Assumes robot starts at (5, 65);

    public PathChain hangPreload, backUp, toSample1, toBucket1, toSample2, toBucket2, toSample3, toBucket3, toParking;
    public void buildPaths(){
        hangPreload = follower.pathBuilder().addPath(new BezierLine(new Point (startPose), new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .build();
        backUp = follower.pathBuilder().addPath(new BezierLine(new Point (preloadPose), new Point(backUpPose)))
                .setLinearHeadingInterpolation(preloadPose.getHeading(), backUpPose.getHeading())
                .build();
        toSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point (backUpPose), new Point(sample1Pose)))
                .setLinearHeadingInterpolation(backUpPose.getHeading(), sample1Pose.getHeading())
                .build();
        toBucket1 = follower.pathBuilder().addPath(new BezierLine(new Point (sample1Pose), new Point(bucketPose)))
                .setLinearHeadingInterpolation(sample1Pose.getHeading(), bucketPose.getHeading())
                .build();
        toSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point (bucketPose), new Point(sample2Pose)))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), sample2Pose.getHeading())
                .build();
        toBucket2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point (sample2Pose), new Point(bucketPose)))
                .setLinearHeadingInterpolation(sample2Pose.getHeading(), bucketPose.getHeading())
                .build();
        toSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point (bucketPose), new Point(sample3Pose)))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), sample3Pose.getHeading())
                .build();
        toBucket3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point (sample3Pose), new Point(bucketPose)))
                .setLinearHeadingInterpolation(sample3Pose.getHeading(), bucketPose.getHeading())
                .build();
        toParking = follower.pathBuilder()
                .addPath(new BezierLine(new Point (bucketPose), new Point(parkingPose)))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), parkingPose.getHeading())
                .build();
    }
    public void rotateArmForwards() {robot.pitch = 175;}
    public void sidewaysClaw(){
        robot.roll = 45;
    }
    public void normalClaw(){
        robot.roll = 0;
    }
    public void setAction(ActionState prevState){
        actionState = prevState;
        armTimer.reset();
        autonomousActionUpdate();
    }
    public void openClaw(){
        robot.claw.setPosition(0.15);
        clawClosed = false;
    }
    public void closeClaw(){
        robot.claw.setPosition(0.4);
        clawClosed = true;
    }
    public void armExtend(int ticks){
        robot.armExtension.setPower(1);
        robot.armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armExtension.setTargetPosition(ticks);
    }
    public void armUp(int ticks){
        robot.armVertical.setPower(1);
        robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armVertical.setTargetPosition(ticks);
    }
    public boolean armFinished(int motorTarget, int currentPosition){
        return Math.abs(motorTarget - currentPosition) <= 2;
    }
    public void grabSample(State currentPathState, int extensionTarget, int verticalTarget, ActionState nextActionState){
        openClaw();
        if (follower.getPose().getY() < 118.5){
            normalClaw();
            armUp(verticalTarget);
            armExtend(extensionTarget);
        }
        if(!follower.isBusy() && state == currentPathState &&  armFinished(verticalTarget-ARM_CONSTANT, robot.armVertical.getCurrentPosition())){
            if (armTimer.seconds() > 2.6){
                closeClaw();
                setAction(nextActionState);
                armTimer.reset();
            }
        }
    }

    public void scoreSample(State currentPathState, ActionState nextActionState){
        clawCloser = 0;
        closeClaw();
        if (state == currentPathState && !follower.isBusy()){
            armUp(2500-ARM_CONSTANT);
            armExtend(-2500);
            rotateArmBackWards();
            if (follower.getCurrentPath().isAtParametricEnd() && armFinished (2500-ARM_CONSTANT, robot.armVertical.getCurrentPosition()) && armFinished(-2500, robot.armExtension.getCurrentPosition()) && !robot.armVertical.isBusy()){
                if (armTimer.seconds() > 1){
                    armTimer.reset();
                }
                openClaw();
                if (armTimer.seconds() > 0.4){
                    setAction(nextActionState);
                }
            }
        }
    }
    public void rotateArmBackWards(){
        robot.pitch = 15;
        robot.roll = 0;
    }
    public void setPathState(State prevState) {
        state = prevState;
        timer.resetTimer();
        autonomousPathUpdate();
    }
    public void autonomousActionUpdate(){
        switch (actionState){
            case RAISE_ARMS:
                rotateArmForwards();
                armExtend(-2650);
                armUp(2050-ARM_CONSTANT);
                if (armFinished(-2650, robot.armExtension.getCurrentPosition()) && armFinished(2050-ARM_CONSTANT, robot.armVertical.getCurrentPosition())) {
                    setAction(ActionState.SCORE_SAMPLE);
                }
                break;
            case SCORE_SAMPLE:
                if (state == State.GO_BACKWARDS && !follower.isBusy() && armTimer.seconds() > 1){
                    openClaw();
                    setAction(ActionState.OPEN_CLAW);
                }
                break;
            case OPEN_CLAW:
                openClaw();
                if (armTimer.seconds() > 0.3) setAction(ActionState.GRAB_SAMPLE1);
            case GRAB_SAMPLE1:
                grabSample(State.GO_TO_BASKET, -755, -700-ARM_CONSTANT, ActionState.CLOSE_CLAW);
                break;
            case CLOSE_CLAW:
                closeClaw();
                setAction(ActionState.PUT_IN_BUCKET);
                break;
            case PUT_IN_BUCKET:
                scoreSample(State.GO_TO_SAMPLE2, ActionState.OPEN_CLAW_2);
                break;
            case OPEN_CLAW_2:
                openClaw();
                if (armTimer.seconds() > 0.3) setAction(ActionState.GRAB_SAMPLE2);
                break;
            case GRAB_SAMPLE2:
                grabSample(State.GO_TO_BASKET_FROM_SAMPLE_2, -790, -600-ARM_CONSTANT, ActionState.CLOSE_CLAW_2);
                break;
            case CLOSE_CLAW_2:
                closeClaw();
                if (armTimer.seconds() > 0.3) setAction(ActionState.PUT_IN_BUCKET_2);
            case PUT_IN_BUCKET_2:
                scoreSample(State.GO_TO_SAMPLE3, ActionState.GRAB_SAMPLE3);
                break;
            case GRAB_SAMPLE3:
                rotateArmForwards();
                sidewaysClaw();
                if ( follower.getPose().getY() < 115 && !isClawClosed){
                    armUp(-300-ARM_CONSTANT);
                    isClawClosed = true;
                    armExtend(-5);
                }
                if(follower.getCurrentPath().isAtParametricEnd() && state == State.GO_TO_BASKET_FROM_SAMPLE_3){
                    if (robot.armVertical.getCurrentPosition() < -295-ARM_CONSTANT){
                        armExtend(-720);
                    }

                    if (robot.armExtension.getCurrentPosition() < -485){
                        armUp(-750-ARM_CONSTANT);
                    }
                    if (armTimer.seconds() > 3 && robot.armExtension.getCurrentPosition() < -495 && robot.armVertical.getCurrentPosition() < -720-ARM_CONSTANT){
                        closeClaw();
                        setAction(ActionState.CLOSE_CLAW_3);
                        armTimer.reset();
                    }
                };
                break;
            case PUT_IN_BUCKET_3:
                scoreSample(State.GO_TO_PARKING, ActionState.CLOSE_CLAW_3);

        }
    }
    public void autonomousPathUpdate(){
        switch (state) {
            case SCORE_PRELOAD_BASKET:
                if (actionState == ActionState.SCORE_SAMPLE){
                    follower.followPath(hangPreload);
                    setPathState(State.GO_BACKWARDS);
                }
                break;
            case GO_BACKWARDS:
                if (actionState == ActionState.GRAB_SAMPLE1){
                    follower.followPath(backUp);
                    setPathState(State.GO_TO_SAMPLE1);
                }
                break;
            case GO_TO_SAMPLE1:
                if (follower.getCurrentPath().isAtParametricEnd() && actionState == ActionState.GRAB_SAMPLE1 ){
                    follower.followPath(toSample1, true);
                    closeClaw();
                    setPathState(State.GO_TO_BASKET);
                }
                break;
            case GO_TO_BASKET:
                if (actionState == ActionState.PUT_IN_BUCKET && !follower.isBusy()&& timer.getElapsedTimeSeconds() > 3) {
                    rotateArmBackWards();
                    follower.followPath(toBucket1, true);
                    setPathState(State.GO_TO_SAMPLE2);
                }
                break;
            case GO_TO_SAMPLE2:
                if (actionState == ActionState.GRAB_SAMPLE2 && follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(toSample2, true);
                    setPathState(State.GO_TO_BASKET_FROM_SAMPLE_2);
                }
                break;
            case GO_TO_BASKET_FROM_SAMPLE_2:
                if (actionState == ActionState.PUT_IN_BUCKET_2 && !follower.isBusy() && timer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(toBucket2, true);
                    setPathState(State.GO_TO_SAMPLE3);
                }
                break;
            case GO_TO_SAMPLE3:
                if (actionState == ActionState.GRAB_SAMPLE3 && !follower.isBusy()){
                    follower.followPath(toSample3, true);
                    rotateArmForwards();
                    setPathState(State.GO_TO_BASKET_FROM_SAMPLE_3);
                }
                break;
            case GO_TO_BASKET_FROM_SAMPLE_3:
                if (actionState == ActionState.PUT_IN_BUCKET_3 && follower.getCurrentPath().isAtParametricEnd() & timer.getElapsedTimeSeconds() > 2){
                    follower.followPath(toBucket3, true);
                    setPathState(State.GO_TO_PARKING);
                }
                break;
            case GO_TO_PARKING:
                if (actionState == ActionState.PARK && follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(toParking, true);
                }
                break;
//            default:
//                stop();
        }
    }
    public void init(){
        follower = new Follower(hardwareMap);
        robot.init(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(.8);
        isClawClosed = false;
        robot.armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateArmForwards();
        closeClaw();
        timer = new Timer();
        armTimer = new ElapsedTime();
        buildPaths();
    }
    public void loop(){
        robot.diddylate(robot.pitch, robot.roll);
        telemetry.addData("Path State", state);
        follower.update();
        telemetry.update();
        autonomousPathUpdate();
        autonomousActionUpdate();
    }
}
