package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous (name = "Specimens Auto")
public class SpecimenAuto extends OpMode {
    Follower follower;
//727 741 4771
    int ARM_CONSTANT = 1200;
    Timer armTimer, pathTimer;
    Hardware robot = Hardware.getInstance();

    enum ActionState{
        RAISE_ARMS,
        HANG_PRELOAD,
        PICK_UP_SPECIMEN_1,
        RAISE_ARM_1,
        HANG_SPECIMEN_1,
        PICK_UP_SPECIMEN_2,
        BACK_UP_ONCE,
        STRAFE_TO_SPECIMEN_2,

        RAISE_ARM_2,
        HANG_SPECIMEN_2,
        PICK_UP_SPECIMEN_3,
        RAISE_ARM_3,
        HANG_SPECIMEN_3

    }
    enum PathState{
        GO_TO_SUBMERSIBLE,
        STRAFE_TO_SAMPLE1,
        BEHIND_SAMPLE1,
        PUSH_SAMPLE1,
        BACKWARDS_FROM_SAMPLE1,
        STRAFE_BEHIND_SAMPLE2,
        PUSH_SAMPLE2,
        GO_BACKWARDS,
        GO_FORWARDS,
        PICK_UP_SPECIMEN_1,
        STRAFE_TO_SUBMERSIBLE_1,
        FORWARDS_TO_SUBMERSIBLE_1,

        BACKKWARDS_FROM_SUBMERSIBLE_1,
        PICK_UP_SPECIMEN_2,
        STRAFE_TO_SUBMERSIBLE_2,
        FORWARDS_TO_SUBMERSIBLE_2,
        BACKWARDS_FROM_SUBMERSIBLE_2,
        PICK_UP_SPECIMEN_3,
        STRAFE_TO_SUBMERSIBLE_3,

        FORWARDS_TO_SUBMERSIBLE_3,
        BACKWARDS_FROM_SUBMERSIBLE_3,
        STRAFE_TO_PARK,
        DONE
    }
    PathState pathState = PathState.GO_TO_SUBMERSIBLE;
    ActionState actionState = ActionState.RAISE_ARMS;
    Pose starting = new Pose(5, 64, 0);
    double lastX = starting.getX();
    boolean extendAlready;
    double lastY = starting.getY();
    double lastH = starting.getHeading();

    Path toSubmersible, strafeToSample1, behindSample1, pushSample1, backwardsFromSample1, strafeBehindSample2, pushSample2, goBackWards, goForwards, strafeToSubmersible1, forwardToSubmersible1, backwardsFromSubmersible1, pickUpSpecimen2, strafeToSubmersible4, forwardToSubmersible2, backwardsFromSubmersible3, pickUpSpecimen3, strafeToSubmersible3, forwardsToSubmersible3, backwardsFromSubmersible2, pickUpSpecimen4, strafeToSubmersible2, forwardToSubmersible4;
    public void init(){
        follower = new Follower(hardwareMap);
        toSubmersible = newPath(21.1, (64), 0);
        strafeToSample1 = newPath(21.1, 49, lastH);
        robot.init(hardwareMap);
        follower.setStartingPose(starting);
        armTimer = new Timer();
        pathTimer = new Timer();
        extendAlready = false;

//        robot.rotateServo.setPosition(.741);

    }


    public void buildPaths(){
        toSubmersible = newPath(21.1, (64), 0);
        strafeToSample1 = newPath(21.1, 49, lastH);
//        behindSample1 = newPath(60, 28, lastH);
//        pushSample1 = newPath(25,26, lastH);
////        backwardsFromSample1 = newPath(65,24, lastH);
////        strafeBehindSample2 = newPath(65 , 14, lastH);
////        pushSample2 = newPath(12, 14, lastH);
//        goBackWards = newPath(30-10.5, 28, -179);
//        goForwards = newPath(6-22, 49, -179);
//        strafeToSubmersible1 = newPath(6-10.5, 64, 0);
//        forwardToSubmersible1 = newPath(31.6-10.5, 64, 0);
//        backwardsFromSubmersible1 = newPath(6-10.5, 64, 0);
//        pickUpSpecimen2 = newPath(6-10.5, 49, -179);
//        strafeToSubmersible2 = newPath(6-10.5, 64, 0);
//        forwardToSubmersible2 = newPath(31.6-10.5, 65, 0);
//        backwardsFromSubmersible2 = newPath(6-10.5, 64, 0);
//        pickUpSpecimen3 = newPath(6-10.5, 49-10.5, -179);
//        strafeToSubmersible3 = newPath(6-10.5, 66-10.5, 0);
//        forwardsToSubmersible3 = newPath(31.6-10.5, 66-10.5, 0);
//        backwardsFromSubmersible3 = newPath(6-10.5, 64-10.5, 0);
//        pickUpSpecimen4 = newPath(6-10.5, 49-10.5, -179);
//        strafeToSubmersible4 = newPath(6-10.5, 66-10.5, 0 );
//        forwardToSubmersible4 = newPath(31.6-10.5, 66-10.5, 0);






    }
//    public PathChain pushSpecimens(){
//
//        PathBuilder builder = new PathBuilder();
//        builder.addPath(huhuh);
//    }
    public Path newPath(double targetX, double targetY, double targetH){
            Point startPoint = new Point(lastX, lastY, Point.CARTESIAN);
            Point endPoint = new Point(targetX, targetY, Point.CARTESIAN);
            Path path = new Path(new BezierLine(startPoint, endPoint));
            path.setLinearHeadingInterpolation(Math.toRadians(lastH), Math.toRadians(targetH));
            path.setPathEndTValueConstraint(.94);
            lastX = targetX;
            lastY = targetY;
            lastH = targetH;
            return path;

        }


    public void setPathState(PathState state){
        pathState = state;
        pathTimer.resetTimer();
    }
    public void autonomousPathUpdate(){
        switch (pathState){
            case GO_TO_SUBMERSIBLE:
                follower.followPath(toSubmersible);
                setPathState(PathState.STRAFE_TO_SAMPLE1);
                break;
//            case STRAFE_TO_SAMPLE1:
//                if (follower.getCurrentPath().isAtParametricEnd() /*&& actionState == ActionState.PICK_UP_SPECIMEN_1*/){
//                    follower.followPath(strafeToSample1);
//                    setPathState(PathState.BEHIND_SAMPLE1);
//                }
//                break;
//            case BEHIND_SAMPLE1:
//                if (follower.getCurrentPath().isAtParametricEnd()){
//                    follower.followPath(behindSample1);
//                    setPathState(PathState.PUSH_SAMPLE1);
//                }
//            case PUSH_SAMPLE1:
//                if (follower.getCurrentPath().isAtParametricEnd()){
//                    follower.followPath(pushSample1);
//                    setPathState(PathState.GO_BACKWARDS);
//                }
//                break;
////            case BACKWARDS_FROM_SAMPLE1:
//                if (follower.getCurrentPath().isAtParametricEnd()){
//                    follower.followPath(backwardsFromSample1);
//                    setPathState(PathState.STRAFE_BEHIND_SAMPLE2);
//                }
//                break;
//            case STRAFE_BEHIND_SAMPLE2:
//                if (follower.getCurrentPath().isAtParametricEnd()){
//                    follower.followPath(strafeBehindSample2);
//                    setPathState(PathState.PUSH_SAMPLE2);
//                }
//                break;
//
////            case TURN:
////                if (follower.getCurrentPath().isAtParametricEnd()){
////                    follower.followPath(turnRo  bot);
////                    setPathState(PathState.PUSH_SAMPLE2);
////                }
//            case PUSH_SAMPLE2:
//                if (follower.getCurrentPath().isAtParametricEnd()){
//                    follower.followPath(pushSample2);
//                    setPathState(PathState.GO_BACKWARDS);
//                }
//                break;
            case GO_BACKWARDS:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(goBackWards);
                    setPathState(PathState.GO_FORWARDS);
                }
            case GO_FORWARDS:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(goForwards);
                    setPathState(PathState.STRAFE_TO_SUBMERSIBLE_1);
                }
            case STRAFE_TO_SUBMERSIBLE_1:
                if (actionState == ActionState.HANG_SPECIMEN_1){
                    follower.followPath(strafeToSubmersible1);
                    setPathState(PathState.FORWARDS_TO_SUBMERSIBLE_1);
                }
            case FORWARDS_TO_SUBMERSIBLE_1:
                if (actionState == ActionState.HANG_SPECIMEN_1){
                    follower.followPath(forwardToSubmersible1);
                    setPathState(PathState.BACKKWARDS_FROM_SUBMERSIBLE_1);
                    rotateServoDown();
                }
                break;
            case BACKKWARDS_FROM_SUBMERSIBLE_1:
                if(actionState == ActionState.PICK_UP_SPECIMEN_2){
                    follower.followPath(backwardsFromSubmersible1);
                    setPathState(PathState.PICK_UP_SPECIMEN_2);
                }
            case PICK_UP_SPECIMEN_2:
                if (!follower.isBusy()){
                    follower.followPath(pickUpSpecimen2);
                    rotateServoForward();
                    setPathState(PathState.STRAFE_TO_SUBMERSIBLE_2);
                }
            case STRAFE_TO_SUBMERSIBLE_2:
                if (actionState == ActionState.HANG_SPECIMEN_2){
                    follower.followPath(strafeToSubmersible2);
                    setPathState(PathState.FORWARDS_TO_SUBMERSIBLE_2);
                    rotateServoDown();
                }
                break;
            case FORWARDS_TO_SUBMERSIBLE_2:
                if (!follower.isBusy()){
                    follower.followPath(forwardToSubmersible2);
                    setPathState(PathState.BACKWARDS_FROM_SUBMERSIBLE_2);
                }
                break;
            case BACKWARDS_FROM_SUBMERSIBLE_2:
                if (actionState == ActionState.PICK_UP_SPECIMEN_3){
                    follower.followPath(backwardsFromSubmersible3);
                    rotateServoForward();
                    setPathState(PathState.PICK_UP_SPECIMEN_3);
                }
                break;
            case PICK_UP_SPECIMEN_3:
                if (!follower.isBusy()){
                    follower.followPath(pickUpSpecimen3);
                    setPathState(PathState.STRAFE_TO_SUBMERSIBLE_3);
                }
                break;
            case STRAFE_TO_SUBMERSIBLE_3:
                if (actionState == ActionState.RAISE_ARM_3){
                    follower.followPath(strafeToSubmersible3);
                    rotateServoDown();
                    setPathState(PathState.FORWARDS_TO_SUBMERSIBLE_3);
                }
            case FORWARDS_TO_SUBMERSIBLE_3:
                if (!follower.isBusy()){
                    follower.followPath(forwardsToSubmersible3);
                    setPathState(PathState.DONE);
                }
            case DONE:
                break;
            default:
                stop();

        }
    }

    public void rotateServoForward(){
        robot.rotateServo.setPosition(.425);
    }
    public void rotateServoDown() {robot.rotateServo.setPosition(.741);}
    public void openClaw(){
        robot.leftServo.setPosition(.639);
        robot.rightServo.setPosition(0.55);
    }
    public void closeClaw(){
        robot.leftServo.setPosition(.508);
        robot.rightServo.setPosition(.69);
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

    public void setActionState(ActionState state){
        actionState = state;
        armTimer.resetTimer();
    }
    public void autonomousActionUpdate(){
        switch(actionState){
            case RAISE_ARMS:
//                setActionState(ActionState.RAISE_ARMS);
                closeClaw();
                armExtend(-1200);
                armUp(2300-ARM_CONSTANT);
                if (robot.armExtension.getCurrentPosition() < -1199 && robot.armVertical.getCurrentPosition() > (2290-ARM_CONSTANT)) {
                    setActionState(ActionState.HANG_PRELOAD);

                }
                break;
            case HANG_PRELOAD:
                if (!follower.isBusy() || armTimer.getElapsedTimeSeconds() > 3.2){
                    armExtend(-350);
                    armUp(2100-ARM_CONSTANT);
                }
                if (robot.armExtension.getCurrentPosition() > -351 && robot.armVertical.getCurrentPosition() < (2101-ARM_CONSTANT)|| armTimer.getElapsedTimeSeconds() > 3) {
                    setActionState(ActionState.PICK_UP_SPECIMEN_1);
                }
//                telemetry.update();
                break;
            case PICK_UP_SPECIMEN_1:
                if (armTimer.getElapsedTimeSeconds() < 1  &&!extendAlready){
                    armExtend(-2);
                    armUp(-150-ARM_CONSTANT);
                    openClaw();
                    extendAlready = true;


                }
                rotateServoForward();

                if (pathState == PathState.STRAFE_TO_SUBMERSIBLE_1 && !follower.isBusy()){
                    armExtend(-263);
                    if (robot.armExtension.getCurrentPosition() < -200){
                        if (armTimer.getElapsedTimeSeconds() > 1){
                            armTimer.resetTimer();
                        }
                        closeClaw();
                        if(armTimer.getElapsedTimeSeconds() > 0.3){
                            setActionState(ActionState.RAISE_ARM_1);
                        }
                    };
                }
                break;
            case RAISE_ARM_1:
                closeClaw();
                armUp(2300-ARM_CONSTANT);
                if (robot.armVertical.getCurrentPosition() > (1000-ARM_CONSTANT)){
                    armExtend(-1200);
                    setActionState(ActionState.HANG_SPECIMEN_1);
                };

                break;
            case HANG_SPECIMEN_1:
                closeClaw();
                if (pathState == PathState.BACKKWARDS_FROM_SUBMERSIBLE_1 && follower.getCurrentPath().isAtParametricEnd()){
                    armExtend(-350);
                    armUp(2100);


                    if (robot.armExtension.getCurrentPosition() > -450 && robot.armVertical.getCurrentPosition() < (2150-ARM_CONSTANT)) {
                        openClaw();
                        setActionState(ActionState.PICK_UP_SPECIMEN_2);
                    }
                }
                break;
            case PICK_UP_SPECIMEN_2:
//                armExtend(-100);
//                armUp(0);
//                openClaw();
                if (pathState == PathState.STRAFE_TO_SUBMERSIBLE_2 && follower.getCurrentPath().isAtParametricEnd()){
                    armExtend(-263);
                    armUp(-150-ARM_CONSTANT);
                    if (robot.armExtension.getCurrentPosition() < -500){

                    }
                    if (robot.armExtension.getCurrentPosition() < -200 && robot.armVertical.getCurrentPosition() < -5-ARM_CONSTANT){
                        closeClaw();
                        if (armTimer.getElapsedTimeSeconds() > 1){
                            armTimer.resetTimer();
                        }
                        if(armTimer.getElapsedTimeSeconds() > 0.3){
                            setActionState(ActionState.RAISE_ARM_2);
                        }
                    };
                }
                break;
            case RAISE_ARM_2:
                closeClaw();
                armUp(2300-ARM_CONSTANT);
                if (robot.armVertical.getCurrentPosition() > (1000-ARM_CONSTANT)){
                    armExtend(-1200);
                    setActionState(ActionState.HANG_SPECIMEN_2);
                };
                break;
            case HANG_SPECIMEN_2:
                closeClaw();
                if (pathState == PathState.BACKWARDS_FROM_SUBMERSIBLE_2  && follower.getCurrentPath().isAtParametricEnd()){
                    armExtend(-350);
                    armUp(2100);

                    if (robot.armExtension.getCurrentPosition() > -450 && robot.armVertical.getCurrentPosition() < 2150-ARM_CONSTANT) {
                        openClaw();
                        setActionState(ActionState.PICK_UP_SPECIMEN_3);
                    }
                }
                break;
            case PICK_UP_SPECIMEN_3:
                if (pathState == PathState.STRAFE_TO_SUBMERSIBLE_3 && follower.getCurrentPath().isAtParametricEnd()){
                    armExtend(-263);
                    armUp(-150-ARM_CONSTANT);
                    if (robot.armExtension.getCurrentPosition() < 500){
                        rotateServoForward();
                    }
                    if (robot.armExtension.getCurrentPosition() < -200 && robot.armVertical.getCurrentPosition() < (-5-ARM_CONSTANT)){
                        closeClaw();
                        if (armTimer.getElapsedTimeSeconds() > 1){
                            armTimer.resetTimer();
                        }
                        if(armTimer.getElapsedTimeSeconds() > 0.3){
                            setActionState(ActionState.RAISE_ARM_3);
                        }
                    };
                }
                break;
            case RAISE_ARM_3:
                closeClaw();
                armExtend(-1200);
                armUp(2300-ARM_CONSTANT);
                setActionState(ActionState.HANG_SPECIMEN_3);
                break;
            case HANG_SPECIMEN_3:
//                closeClaw();
//                if (pathState == PathState.BACKWARDS_FROM_SUBMERSIBLE_2  && follower.getCurrentPath().isAtParametricEnd()){
//                    armExtend(-350);
//                    armUp(2100);
//
//                    if (robot.armExtension.getCurrentPosition() > -450 && robot.armVertical.getCurrentPosition() < 2150) {
//                        openClaw();
//                        setActionState(ActionState.PICK_UP_SPECIMEN_3);
//                    }
//                }
                break;
            default:
                stop();
        }
    }
    public void loop() {

        telemetry.addData("Position",follower.getPose());
        telemetry.addData("Current Path: ", pathState);
        telemetry.addData("Timer: ", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Path State", pathState);
//        telemetry.addData("Target Pose", follower.getCurrentPath().));
        telemetry.addData("Action State",actionState);
        telemetry.addData("Arm Extension Position", robot.armExtension.getCurrentPosition());
//        autonomousActionUpdate();
        telemetry.update();
        autonomousPathUpdate();
        follower.update();

    }
}
