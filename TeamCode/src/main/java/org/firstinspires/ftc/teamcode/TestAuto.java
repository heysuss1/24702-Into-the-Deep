package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous (name = "Samples Auto")
public class TestAuto extends OpMode {
    Follower follower;
    Hardware robot = Hardware.getInstance();
    Pose starting = new Pose(5, 78, 0);
    Timer timer;
    Pose bucket = new Pose(15, 127, Point.CARTESIAN);
    Pose frontOfSubmersible = new Pose(34, 71, Point.CARTESIAN);
    Pose frontOfObservationZone = new Pose(9, 17, Point.CARTESIAN);

    double lastX = starting.getX();
    double lastY = starting.getY();
    double lastH = starting.getHeading();

    ElapsedTime armTimer;
    Path toSubmersible, backUp, toSample1, toBucket, toSample2, toBucketFromSample2, toSample3, toBucketFromSample3, toParking;

    enum State{
        GO_TO_BAR,
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
        HANG_SPECIMEN,
        GRAB_SAMPLE1,
        PUT_IN_BUCKET,
        GRAB_SAMPLE2,
        PUT_IN_BUCKET_2,
        GRAB_SAMPLE3,
        PUT_IN_BUCKET_3,
        PARK
    }
    enum ClawUpdate{
        CLOSE_SAMPLE_1,
    }
    State state = State.GO_TO_BAR;
    ActionState actionState = ActionState.RAISE_ARMS;
//    PathBuilder toSubmersible;

    //Assumes robot starts at (5, 65)

    public void buildPaths(){
        /*
        int num = 0;
        switch (num){
            case 0:
        }
         */
        toSubmersible = (newPath(31.5, 78, Point.CARTESIAN));
        backUp = newPath(19.2, 78, lastH);
        toSample1 = newPath(28.5, 118.2, lastH);
        toBucket = newPath(12, 124, -45);
        toSample2 = newPath(25, 128.2, 0);
        toBucketFromSample2 = newPath(12, 124, -45);
        toSample3 = newPath(29, 128, 37.5);
        toBucketFromSample3 = newPath(12.3, 124, -45);
        toParking = newPath(56, 98.5, -90);



    }
    public void init(){
        follower = new Follower(hardwareMap);
        robot.init(hardwareMap);
        lastH = follower.getPose().getHeading();
        follower.setStartingPose(starting);
        robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armVertical.setTargetPosition(1200);
        robot.armVertical.setPower(0.8);
        robot.rotateServo.setPosition(.726);
        timer = new Timer();
        armTimer = new ElapsedTime();
        buildPaths();

//        follower.followPath(toSubmersible());
    }
//   public PathChain toSubmersible(){
//        PathBuilder builder = new PathBuilder();
//        builder.addPath(newPath(31.2, 78, Point.CARTESIAN));
//        //                .addPath(new BezierLine(new Point(40, 65, Point.CARTESIAN), new Point(5, 65, Point.CARTESIAN)))
////                .setTangentHeadingInterpolation();
////                .addPath(new BezierLine(new Point(46, 30, Point.CARTESIAN), new Point(56, 24, Point.CARTESIAN)));
//        return builder.build();
//    }



    public void setPathState(State prevState){
        state = prevState;
        timer.resetTimer();
        autonomousPathUpdate();
    }
    public void autonomousPathUpdate(){
        switch (state) {
            case GO_TO_BAR:
                follower.followPath(toSubmersible);
                setPathState(State.GO_BACKWARDS);
                break;
            case GO_BACKWARDS:
                if (actionState == ActionState.GRAB_SAMPLE1){
                    follower.followPath(backUp);
                    setPathState(State.GO_TO_SAMPLE1);
                }
//                if (follower.getCurrentPath().isAtParametricEnd()) {
//                    setPathState(State.GO_TO_SAMPLE1);
//                }
                break;
            case GO_TO_SAMPLE1:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(toSample1);
                    closeClaw();
                    setPathState(State.GO_TO_BASKET);
                }
                break;
            case GO_TO_BASKET:
                if (actionState == ActionState.PUT_IN_BUCKET && !follower.isBusy()) {
                    rotateArmBackWards();
                    follower.followPath(toBucket);
                    setPathState(State.GO_TO_SAMPLE2);
                }
                break;
            case GO_TO_SAMPLE2:
                if (timer.getElapsedTimeSeconds() > 4 && !follower.isBusy()){
                    follower.followPath(toSample2);
                    setPathState(State.GO_TO_BASKET_FROM_SAMPLE_2);
                }
                if (actionState == ActionState.PUT_IN_BUCKET && !follower.isBusy() && armTimer.seconds() > 1) {

                }
                break;
            case GO_TO_BASKET_FROM_SAMPLE_2:
                if (actionState == ActionState.PUT_IN_BUCKET_2 && !follower.isBusy() && timer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(toBucketFromSample2);
                    setPathState(State.GO_TO_SAMPLE3);
                }
                break;
            case GO_TO_SAMPLE3:
                if (timer.getElapsedTimeSeconds() > 4 && !follower.isBusy()){
                    follower.followPath(toSample3);
                    rotateArmForwards();
                    setPathState(State.GO_TO_BASKET_FROM_SAMPLE_3);
                }
                break;
            case GO_TO_BASKET_FROM_SAMPLE_3:
                if (actionState == ActionState.PUT_IN_BUCKET_3 && follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(toBucketFromSample3);
                    setPathState(State.GO_TO_PARKING);
                }
                break;
            case GO_TO_PARKING:
                if (actionState == ActionState.PARK && follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(toParking);
                    break;
                }
                break;
//            default:
//                stop();
        }
    }

    public void setAction(ActionState prevState){
        actionState = prevState;
        armTimer.reset();
        autonomousActionUpdate();
    }

    public void closeClaw(){
        robot.leftServo.setPosition(.639);
        robot.rightServo.setPosition(.1);
    }
    public void openClaw(){
        robot.leftServo.setPosition(0.508);
        robot.rightServo.setPosition(0.24);
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

    public void rotateArmBackWards(){
        robot.rotateServo.setPosition(.174);
    }
    public void rotateArmForwards(){
        robot.rotateServo.setPosition(.726);
    }
    public void autonomousActionUpdate(){
        switch (actionState){
            case RAISE_ARMS:
                closeClaw();
                armExtend(-1200);
                armUp(2300);
                if (robot.armExtension.getCurrentPosition() < -1199 && robot.armVertical.getCurrentPosition() > 2290) {
                    setAction(ActionState.HANG_SPECIMEN);

                }
                break;
            case HANG_SPECIMEN:
                if (!follower.isBusy()){
                    armExtend(-350);
                    armUp(2100);
                }
                if (robot.armExtension.getCurrentPosition() > -351 && robot.armVertical.getCurrentPosition() < 2101) {
                    setAction(ActionState.GRAB_SAMPLE1);
                }
//                telemetry.update();
                break;
////                if (robot.armExtension.getCurrentPosition() < - && robot.armVertical.getCurrentPosition() > ) {
////                    setAction(ActionState.HANG_SPECIMEN);
////                }
//                break;
            case GRAB_SAMPLE1:
                openClaw();

                if (state == State.GO_TO_SAMPLE1){
                    // We need to figure out how to do this but for now Thread.sleep(300);
                    armUp(-50);
                    armExtend(-697);

                }
                if (follower.getPose().getY() > 117){
                    armUp(-420);
                }
                if(!follower.isBusy() && state == State.GO_TO_BASKET){
                    if (armTimer.seconds() > 1){
                        armTimer.reset();
                    }
                    closeClaw();
                    if (armTimer.seconds() > 0.3){
                        setAction(ActionState.PUT_IN_BUCKET);
                    }
                }

                break;
            case PUT_IN_BUCKET:
                closeClaw();
                if (state == State.GO_TO_SAMPLE2 && !follower.isBusy()){
                    armUp(4000);
                    armExtend(-2350);
                    if (!follower.isBusy() && robot.armVertical.getCurrentPosition() > 3000){
                        rotateArmBackWards();
                        if (armTimer.seconds() > 1){
                            armTimer.reset();
                        }
                        if (robot.armVertical.getCurrentPosition() > 3900){
                            openClaw();
                            if (armTimer.seconds() > 0.4){
                                setAction(ActionState.GRAB_SAMPLE2);
                            }

                        }
                        if (!robot.armVertical.isBusy() && !robot.armExtension.isBusy()){
                        }
                    }
                }
                break;
            case GRAB_SAMPLE2:
                openClaw();

                if (state == State.GO_TO_BASKET_FROM_SAMPLE_2){
                    // We need to figure out how to do this but for now Thread.sleep(300);
                    armUp(-420);
                    armExtend(-950);
                    rotateArmForwards();

                }
                if (armTimer.seconds() > 2.8){
                    setAction(ActionState.PUT_IN_BUCKET_2);
                }
                if(!follower.isBusy() && state == State.GO_TO_BASKET){
                    if (armTimer.seconds() > 1){
                        armTimer.reset();
                    }
                    closeClaw();
                    if (armTimer.seconds() > 0.3){
                        setAction(ActionState.PUT_IN_BUCKET_2);
                    }
                }
                break;
            case PUT_IN_BUCKET_2:
                closeClaw();
                if (state == State.GO_TO_SAMPLE3 && !follower.isBusy()){
                    rotateArmBackWards();
                    armUp(4100);
                    armExtend(-2350);
                    if (!follower.isBusy() && robot.armVertical.getCurrentPosition() > 3000) {
                        if (armTimer.seconds() > 1) {
                            armTimer.reset();
                        }
                        if (robot.armVertical.getCurrentPosition() > 3900) {
                            openClaw();
                            if (armTimer.seconds() > 0.4) {
                                setAction(ActionState.GRAB_SAMPLE3);
                            }

                        }
                    }
                }
                break;
            case GRAB_SAMPLE3:
                if (state == State.GO_TO_BASKET_FROM_SAMPLE_3){
                    // We need to figure out how to do this but for now Thread.sleep(300);
                    armUp(-420);
                    armExtend(-800);

                }

                if (robot.armVertical.getCurrentPosition() < 200){
                    armExtend(-1400);
                }
                if (armTimer.seconds() > 2.8){
                    setAction(ActionState.PUT_IN_BUCKET_3);
                }
                if(!follower.isBusy() && state == State.GO_TO_BASKET_FROM_SAMPLE_3 && !robot.armVertical.isBusy()){
                    if (armTimer.seconds() > 1){
                        armTimer.reset();
                    }
                    closeClaw();
                    if (armTimer.seconds() > 0.3){
                        setAction(ActionState.PUT_IN_BUCKET_3);
                    }
                }

            break;
            case PUT_IN_BUCKET_3:
                closeClaw();
                if (state == State.GO_TO_PARKING && !follower.isBusy()){
                    rotateArmBackWards();
                    armUp(4000);
                    armExtend(-2350);
                    if (!follower.isBusy() && robot.armVertical.getCurrentPosition() > 3000) {
                        if (armTimer.seconds() > 1) {
                            armTimer.reset();
                        }
                        if (robot.armVertical.getCurrentPosition() > 3900) {
                            openClaw();
                            if (armTimer.seconds() > 0.4) {
                                setAction(ActionState.PARK);
                            }

                        }
                    }
                }
                break;
            case PARK:
                if (armTimer.seconds() > 2){
                    rotateArmForwards();
                    armUp(600);
                    armExtend(-1000);
                }
                break;
            default:
                stop();

        }
    }

    public Path newPath(double targetX, double targetY, double targetH){
        Point startPoint = new Point(lastX, lastY, Point.CARTESIAN);
        Point endPoint = new Point(targetX, targetY, Point.CARTESIAN);
        Path path = new Path(new BezierLine(startPoint, endPoint));
        path.setLinearHeadingInterpolation(Math.toRadians(lastH), Math.toRadians(targetH));
        lastX = targetX;
        lastY = targetY;
        lastH = targetH;
        return path;

    }

    public void updateClaw(){

    }

    @Override
    public void loop(){
        telemetry.addData("X Position", follower.getPose());
        telemetry.addData("Position", robot.armExtension.getCurrentPosition());
        telemetry.addData("Arm Position", robot.armVertical.getCurrentPosition());
        telemetry.addData("Action State", actionState);
        telemetry.addData("Current Path is", state);
        telemetry.addData("Is Busy",  follower.isBusy());
        telemetry.addData("Seconds", armTimer.seconds());
        telemetry.update();
        autonomousPathUpdate();
        autonomousActionUpdate();
        follower.update();

    }
}
