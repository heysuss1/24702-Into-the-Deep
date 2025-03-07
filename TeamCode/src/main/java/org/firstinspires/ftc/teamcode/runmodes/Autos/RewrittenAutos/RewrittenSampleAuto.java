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


@Autonomous(name = "States sample Auto ")
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
    public static Pose sample1Pose = new Pose(4.6, 109.5, Math.toRadians(-90));
    public static Pose bucketPose = new Pose (10, 123, Math.toRadians(-130));
    public static Pose sample2Pose = new Pose (14, 109, Math.toRadians(-90));
    public static Pose sample3Pose = new Pose (22.7, 89.4, Math.toRadians(0));
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
        GO_TO_PARKING,
        DONE

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
            if (!follower.isBusy() && armFinished (2500-ARM_CONSTANT, robot.armVertical.getCurrentPosition()) && armFinished(-2500, robot.armExtension.getCurrentPosition()) && !robot.armVertical.isBusy()){
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
clawCloser = 0;
        switch (actionState){
            case RAISE_ARMS:
                rotateArmForwards();
                armExtend(-1927);
                armUp(2020-ARM_CONSTANT);
                if (robot.armExtension.getCurrentPosition() < -2640 && robot.armVertical.getCurrentPosition() > (2010-ARM_CONSTANT)) {
                    setAction(ActionState.SCORE_SAMPLE);
                }
                break;
            case SCORE_SAMPLE:
                if (state == State.GO_BACKWARDS && !follower.isBusy() && armTimer.seconds() > 1.2){
                    openClaw();
                    setAction(ActionState.OPEN_CLAW);
                }
//                if (armTimer.seconds() > ){
//                }
//                if (robot.armExtension.getCurrentPosition() > -351 && robot.armVertical.getCurrentPosition() < (1902-ARM_CONSTANT) || armTimer.seconds() > 2.5) {
//                    setAction(ActionState.GRAB_SAMPLE1);
//                }
//                telemetry.update();
                break;
////                if (robot.armExtension.getCurrentPosition() < - && robot.armVertical.getCurrentPosition() > ) {
////                    setAction(ActionState.HANG_SPECIMEN);
////                }
//                break;
            case OPEN_CLAW:
                openClaw();
                if (armTimer.seconds() > 0.1) setAction(ActionState.GRAB_SAMPLE1);
                break;
            case GRAB_SAMPLE1:
                openClaw();
                if (follower.getPose().getY() < 118.5){
                    normalClaw();
                    // We need to figure out how to do this but for now Thread.sleep(300);
                    armUp(-750-ARM_CONSTANT);
                    armExtend(-675);
                }
                if(!follower.isBusy() && state == State.GO_TO_BASKET && robot.armVertical.getCurrentPosition() < (-748-ARM_CONSTANT) && robot.armExtension.getCurrentPosition() < -940){
                    if (armTimer.seconds() > 2){
                        closeClaw();
                        setAction(ActionState.CLOSE_CLAW);
                        armTimer.reset();
                    }
                }
                break;
            case CLOSE_CLAW:
                closeClaw();
                if (armTimer.seconds() > 1.5) setAction(ActionState.PUT_IN_BUCKET);
                break;
            case PUT_IN_BUCKET:
                clawCloser = 0;
                closeClaw();
                armUp(2900-ARM_CONSTANT);
                armExtend(-1576);
                rotateArmBackWards();
                    if (!follower.isBusy() && robot.armVertical.getCurrentPosition() > (2820-ARM_CONSTANT) && robot.armExtension.getCurrentPosition() < -2240 && !robot.armVertical.isBusy()){
                        if (armTimer.seconds() > 1){
                            armTimer.reset();
                        }
//                        if (robot.armVertical.getCurrentPosition() > (2650-ARM_CONSTANT)){
                        openClaw();
//                        if (armTimer.seconds() > 0.15){
                            setAction(ActionState.OPEN_CLAW_2);
//                        }
                    }
                break;
            case OPEN_CLAW_2:
                openClaw();
                setAction(ActionState.GRAB_SAMPLE2);
                break;
            case GRAB_SAMPLE2:
                openClaw();
                rotateArmForwards();
                if (/*state == State.GO_TO_BASKET && !follower.isBusy()*/ follower.getPose().getY() < 115){
                    // We need to figure out how to do this but for now Thread.sleep(300);
                    armUp(-700-ARM_CONSTANT);
                    armExtend(-700);
                }
//                if (follower.getPose().getY() > 113){
//                    armUp(-470-ARM_CONSTANT);
//                }
                if(!follower.isBusy() && state == State.GO_TO_BASKET_FROM_SAMPLE_2 && robot.armVertical.getCurrentPosition() < (-680-ARM_CONSTANT) && robot.armExtension.getCurrentPosition() < -970){
                    if (armTimer.seconds() > 2){
                        closeClaw();
                        setAction(ActionState.CLOSE_CLAW_2);
                        armTimer.reset();
                    }
//                    if (armTimer.seconds() > 0.3 && clawClosed){
//                        setAction(ActionState.PUT_IN_BUCKET);
//                    }
                }
                break;
            case CLOSE_CLAW_2:
                closeClaw();
                if (armTimer.seconds() > 0.5) setAction(ActionState.PUT_IN_BUCKET_2);
                break;
            case PUT_IN_BUCKET_2:
                clawCloser = 0;
                closeClaw();
                armUp(2950-ARM_CONSTANT);
                armExtend(-1585);
                rotateArmBackWards();
                    if (!follower.isBusy() && robot.armExtension.getCurrentPosition() < -2250 && !robot.armVertical.isBusy()){
                        if (armTimer.seconds() > 1){
                            armTimer.reset();
                        }
//                        if (robot.armVertical.getCurrentPosition() > (2650-ARM_CONSTANT)){
                        openClaw();
//                        if (armTimer.seconds() > 0.15){
                            setAction(ActionState.GRAB_SAMPLE3);
//                        }
                    }
                break;
            case GRAB_SAMPLE3:
                sidewaysClaw();
                rotateArmForwards();
                if ( follower.getPose().getY() < 115 && !isClawClosed){
                    // We need to figure out how to do this but for now Thread.sleep(300);
                    armUp(-300-ARM_CONSTANT);
                    isClawClosed = true;
                    armExtend(-5);
                }
                if(!follower.isBusy() && state == State.GO_TO_BASKET_FROM_SAMPLE_3){
                    if (robot.armVertical.getCurrentPosition() < -295-ARM_CONSTANT){
                        armExtend(-605);
                    }

                    if (robot.armExtension.getCurrentPosition() < -340){
                        armUp(-950-ARM_CONSTANT);
                    }
                    if (armTimer.seconds() > 2 && robot.armExtension.getCurrentPosition() < -415 && robot.armVertical.getCurrentPosition() < -850-ARM_CONSTANT){
                        closeClaw();
                        setAction(ActionState.CLOSE_CLAW_3);
                        armTimer.reset();
                    }
                }
                    break;

            case CLOSE_CLAW_3:
                closeClaw();
                if (armTimer.seconds() > 0.5){
                    setAction(ActionState.PUT_IN_BUCKET_3);
                }
                break;
            case PUT_IN_BUCKET_3:
                normalClaw();
                clawCloser = 0;
                closeClaw();
                armUp(2950-ARM_CONSTANT);
                armExtend(-1580);
                rotateArmBackWards();
                    if (!follower.isBusy() && robot.armVertical.getCurrentPosition() > (2900-ARM_CONSTANT) && !robot.armVertical.isBusy()){
                        if (armTimer.seconds() > 1){
                            armTimer.reset();
                        }
//                        if (robot.armVertical.getCurrentPosition() > (2650-ARM_CONSTANT)){
                            openClaw();
//                        if (armTimer.seconds() > 0.15){
                            setAction(ActionState.PARK);
//                        }
                    }
                break;
            case PARK:
                if (armTimer.seconds() > 1){
                    rotateArmForwards();
                    armUp(580-ARM_CONSTANT);
                    armExtend(-700);
                }
                break;
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
                if (actionState == ActionState.GRAB_SAMPLE1 && !follower.isBusy()){
                    follower.followPath(backUp);
                    setPathState(State.GO_TO_SAMPLE1);
                }
                break;
            case GO_TO_SAMPLE1:
                if (!follower.isBusy() && actionState == ActionState.GRAB_SAMPLE1){
                    follower.followPath(toSample1, true);
                    closeClaw();
                    setPathState(State.GO_TO_BASKET);
                }
                break;
            case GO_TO_BASKET:
                if (actionState == ActionState.PUT_IN_BUCKET && !follower.isBusy()) {
                    rotateArmBackWards();
                    follower.followPath(toBucket1, true);
                    setPathState(State.GO_TO_SAMPLE2);
                }
                break;
            case GO_TO_SAMPLE2:
                if (actionState == ActionState.GRAB_SAMPLE2 && !follower.isBusy()){
                    follower.followPath(toSample2, true);
                    setPathState(State.GO_TO_BASKET_FROM_SAMPLE_2);
                }
                break;
            case GO_TO_BASKET_FROM_SAMPLE_2:
                if (!follower.isBusy() && actionState == ActionState.PUT_IN_BUCKET_2 && timer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(toBucket2, true);
                    setPathState(State.GO_TO_SAMPLE3);
                }
                break;
            case GO_TO_SAMPLE3:
                if (actionState == ActionState.GRAB_SAMPLE3 && !follower.isBusy() && !follower.isBusy()){
                    follower.followPath(toSample3, true);
                    rotateArmForwards();
                    setPathState(State.GO_TO_BASKET_FROM_SAMPLE_3);
                }
                break;
            case GO_TO_BASKET_FROM_SAMPLE_3:
                if (!follower.isBusy() && actionState == ActionState.PUT_IN_BUCKET_3 && timer.getElapsedTimeSeconds() > 2){
                    follower.followPath(toBucket3, true);
                    setPathState(State.GO_TO_PARKING);
                }
                break;
            case GO_TO_PARKING:
                if (actionState == ActionState.PARK && !follower.isBusy()){
                    follower.followPath(toParking, true);
                    setPathState(State.DONE);
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
        telemetry.addData("Action State", actionState);
        telemetry.addData("Arm Extension", robot.armExtension.getCurrentPosition());
        telemetry.addData("Arm Vertical", robot.armVertical.getCurrentPosition());
        follower.update();
        telemetry.update();
        autonomousPathUpdate();
        autonomousActionUpdate();
    }
}
