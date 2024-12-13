package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class OnlySamplesAuto extends OpMode {
    Follower follower;
    Hardware robot = Hardware.getInstance();
    Pose starting = new Pose(7, 86, -90);
    Timer armTimer, pathTimer;
    Pose bucket = new Pose(15, 127, Point.CARTESIAN);
    Pose frontOfSubmersible = new Pose(34, 71, Point.CARTESIAN);
    Pose frontOfObservationZone = new Pose(9, 17, Point.CARTESIAN);

    double lastX = starting.getX();
    double lastY = starting.getY();
    double lastH = starting.getHeading();

//    ElapsedTime armTimer;
    Path preloadBucket, backUp, toSample1, toBucket, toSample2, toBucketFromSample2, toSample3, toBucketFromSample3, toParking;

    enum State{
        HANG_IN_BASKET,
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
    TestAuto.State state = TestAuto.State.GO_TO_BAR;
    TestAuto.ActionState actionState = TestAuto.ActionState.RAISE_ARMS;
//    PathBuilder toSubmersible;

    //Assumes robot starts at (5, 65)

    public void buildPaths(){
        /*
        int num = 0;
        switch (num){
            case 0:
        }
         */

        preloadBucket = newPath(7, 110, 0);
//         = (newPath(31.5, 78, Point.CARTESIAN));
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
        robot.rotateServo.setPosition(.741);
        pathTimer = new Timer();
        armTimer = new Timer();
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



    public void setPathState(TestAuto.State prevState){
        state = prevState;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }
    public void autonomousPathUpdate(){
        switch (state) {
            case GO_TO_BAR:
                follower.followPath(preloadBucket);
                setPathState(TestAuto.State.GO_BACKWARDS);
                break;
            case GO_BACKWARDS:
                if (actionState == TestAuto.ActionState.GRAB_SAMPLE1){
                    follower.followPath(backUp);
                    setPathState(TestAuto.State.GO_TO_SAMPLE1);
                }
//                if (follower.getCurrentPath().isAtParametricEnd()) {
//                    setPathState(State.GO_TO_SAMPLE1);
//                }
                break;
            case GO_TO_SAMPLE1:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(toSample1);
                    closeClaw();
                    setPathState(TestAuto.State.GO_TO_BASKET);
                }
                break;
            case GO_TO_BASKET:
                if (actionState == TestAuto.ActionState.PUT_IN_BUCKET && !follower.isBusy()) {
                    rotateArmBackWards();
                    follower.followPath(toBucket);
                    setPathState(TestAuto.State.GO_TO_SAMPLE2);
                }
                break;
            case GO_TO_SAMPLE2:
                if (armTimer.getElapsedTimeSeconds() > 4 && !follower.isBusy()){
                    follower.followPath(toSample2);
                    setPathState(TestAuto.State.GO_TO_BASKET_FROM_SAMPLE_2);
                }
                if (actionState == TestAuto.ActionState.PUT_IN_BUCKET && !follower.isBusy() && armTimer.getElapsedTimeSeconds() > 1) {

                }
                break;
            case GO_TO_BASKET_FROM_SAMPLE_2:
                if (actionState == TestAuto.ActionState.PUT_IN_BUCKET_2 && !follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(toBucketFromSample2);
                    setPathState(TestAuto.State.GO_TO_SAMPLE3);
                }
                break;
            case GO_TO_SAMPLE3:
                if (pathTimer.getElapsedTimeSeconds() > 4 && !follower.isBusy()){
                    follower.followPath(toSample3);
                    rotateArmForwards();
                    setPathState(TestAuto.State.GO_TO_BASKET_FROM_SAMPLE_3);
                }
                break;
            case GO_TO_BASKET_FROM_SAMPLE_3:
                if (actionState == TestAuto.ActionState.PUT_IN_BUCKET_3 && follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(toBucketFromSample3);
                    setPathState(TestAuto.State.GO_TO_PARKING);
                }
                break;
            case GO_TO_PARKING:
                if (actionState == TestAuto.ActionState.PARK && follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(toParking);
                    break;
                }
                break;
//            default:
//                stop();
        }
    }

    public void setAction(TestAuto.ActionState prevState){
        actionState = prevState;
        armTimer.resetTimer();
        autonomousActionUpdate();
    }

    public void closeClaw(){
        robot.leftServo.setPosition(.508);
        robot.rightServo.setPosition(.69);
    }
    public void openClaw(){
        robot.leftServo.setPosition(.639);
        robot.rightServo.setPosition(0.55);
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
        robot.rotateServo.setPosition(.153);
    }
    public void rotateArmForwards(){
        robot.rotateServo.setPosition(.741);
    }
    public void autonomousActionUpdate(){
        switch (actionState){
            case RAISE_ARMS:
                closeClaw();
                armExtend(-2350);
                armUp(2400);
                if (robot.armExtension.getCurrentPosition() < -2300 && robot.armVertical.getCurrentPosition() > 2350 && !follower.isBusy()) {
                    if (armTimer.getElapsedTimeSeconds() > 0.5){
                        openClaw();

                    }


                }
                break;
            case HANG_SPECIMEN:
                if (!follower.isBusy()){
                    armExtend(-350);
                    armUp(2100);
                }
                if (robot.armExtension.getCurrentPosition() > -351 && robot.armVertical.getCurrentPosition() < 2101) {
                    setAction(TestAuto.ActionState.GRAB_SAMPLE1);
                }
//                telemetry.update();
                break;
////                if (robot.armExtension.getCurrentPosition() < - && robot.armVertical.getCurrentPosition() > ) {
////                    setAction(ActionState.HANG_SPECIMEN);
////                }
//                break;
            case GRAB_SAMPLE1:
                openClaw();

                if (state == TestAuto.State.GO_TO_SAMPLE1){
                    // We need to figure out how to do this but for now Thread.sleep(300);
                    armUp(-50);
                    armExtend(-697);

                }
                if (follower.getPose().getY() > 117){
                    armUp(-420);
                }
                if(!follower.isBusy() && state == TestAuto.State.GO_TO_BASKET){
                    if (armTimer.getElapsedTimeSeconds() > 1){
                        armTimer.resetTimer();
                    }
                    closeClaw();
                    if (armTimer.getElapsedTimeSeconds() > 0.3){
                        setAction(TestAuto.ActionState.PUT_IN_BUCKET);
                    }
                }

                break;
            case PUT_IN_BUCKET:
                closeClaw();
                if (state == TestAuto.State.GO_TO_SAMPLE2 && !follower.isBusy()){
                    armUp(4000);
                    armExtend(-2350);
                    if (!follower.isBusy() && robot.armVertical.getCurrentPosition() > 3000){
                        rotateArmBackWards();
                        if (armTimer.getElapsedTimeSeconds() > 1){
                            armTimer.resetTimer();
                        }
                        if (robot.armVertical.getCurrentPosition() > 3900){
                            openClaw();
                            if (armTimer.getElapsedTimeSeconds() > 0.4){
                                setAction(TestAuto.ActionState.GRAB_SAMPLE2);
                            }

                        }
                        if (!robot.armVertical.isBusy() && !robot.armExtension.isBusy()){
                        }
                    }
                }
                break;
            case GRAB_SAMPLE2:
                openClaw();

                if (state == TestAuto.State.GO_TO_BASKET_FROM_SAMPLE_2){
                    // We need to figure out how to do this but for now Thread.sleep(300);
                    armUp(-420);
                    armExtend(-950);
                    rotateArmForwards();

                }
                if (armTimer.getElapsedTimeSeconds() > 2.8){
                    setAction(TestAuto.ActionState.PUT_IN_BUCKET_2);
                }
                if(!follower.isBusy() && state == TestAuto.State.GO_TO_BASKET){
                    if (armTimer.getElapsedTimeSeconds() > 1){
                        armTimer.resetTimer();
                    }
                    closeClaw();
                    if (armTimer.getElapsedTimeSeconds() > 0.3){
                        setAction(TestAuto.ActionState.PUT_IN_BUCKET_2);
                    }
                }
                break;
            case PUT_IN_BUCKET_2:
                closeClaw();
                if (state == TestAuto.State.GO_TO_SAMPLE3 && !follower.isBusy()){
                    rotateArmBackWards();
                    armUp(4100);
                    armExtend(-2350);
                    if (!follower.isBusy() && robot.armVertical.getCurrentPosition() > 3000) {
                        if (armTimer.getElapsedTimeSeconds() > 1) {
                            armTimer.resetTimer();
                        }
                        if (robot.armVertical.getCurrentPosition() > 3900) {
                            openClaw();
                            if (armTimer.getElapsedTimeSeconds() > 0.4) {
                                setAction(TestAuto.ActionState.GRAB_SAMPLE3);
                            }

                        }
                    }
                }
                break;
            case GRAB_SAMPLE3:
                if (state == TestAuto.State.GO_TO_BASKET_FROM_SAMPLE_3){
                    // We need to figure out how to do this but for now Thread.sleep(300);
                    armUp(0);
                    armExtend(-800);

                }

                if (robot.armVertical.getCurrentPosition() < 5){
                    armExtend(-1550);
                }
                if (robot.armExtension.getCurrentPosition() < -1200){
                    armUp(-420);
                }
                if (armTimer.getElapsedTimeSeconds() > 2.8){
                    setAction(TestAuto.ActionState.PUT_IN_BUCKET_3);
                }
                if(!follower.isBusy() && state == TestAuto.State.GO_TO_BASKET_FROM_SAMPLE_3 && !robot.armVertical.isBusy() && !robot.armExtension.isBusy()){
                    if (armTimer.getElapsedTimeSeconds() > 1){
                        armTimer.resetTimer();
                    }
                    closeClaw();
                    if (armTimer.getElapsedTimeSeconds() > 0.3){
                        setAction(TestAuto.ActionState.PUT_IN_BUCKET_3);
                    }
                }

                break;
            case PUT_IN_BUCKET_3:
                closeClaw();
                if (state == TestAuto.State.GO_TO_PARKING && !follower.isBusy()){
                    rotateArmBackWards();
                    armUp(4000);
                    armExtend(-2350);
                    if (!follower.isBusy() && robot.armVertical.getCurrentPosition() > 3000) {
                        if (armTimer.getElapsedTimeSeconds() > 1) {
                            armTimer.resetTimer();
                        }
                        if (robot.armVertical.getCurrentPosition() > 3900) {
                            openClaw();
                            if (armTimer.getElapsedTimeSeconds() > 0.4) {
                                setAction(TestAuto.ActionState.PARK);
                            }
                        }
                    }
                }
                break;
            case PARK:
                if (armTimer.getElapsedTimeSeconds() > 2){
                    rotateArmForwards();
                    armUp(750);
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
        telemetry.addData("Seconds", armTimer.getElapsedTimeSeconds());
        telemetry.addData("Claw Position", robot.leftServo.getPosition());
        telemetry.update();
        autonomousPathUpdate();
        autonomousActionUpdate();
        follower.update();

    }

}
