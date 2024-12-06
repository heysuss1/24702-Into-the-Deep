package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous (name = "Specimens Auto")
public class SpecimenAuto extends OpMode {
    Follower follower;
//727 741 4771

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
        COLLECT_SPECIMEN,
        GO_FORWARDS,
        TO_SUBMERSIBLE_1,
        STRAFE_ONCE,
        GO_FORWARD,
        TO_SPECIMEN_2,
        BACKWARDS_FROM_SUBMERSIBLE,
        TO_SUBMERSIBLE_2,
        BACKWARDS_FROM_SUBMERSIBLE_3,

        STRAFE_TO_SUBMERSIBLE_3,
        TO_SPECIMEN_3,
        PICK_UP_4TH_SPECIMEN,
        TO_SUBMERSIBLE_3,
    }
    PathState pathState = PathState.GO_TO_SUBMERSIBLE;
    ActionState actionState = ActionState.RAISE_ARMS;
    Pose starting = new Pose(5, 64, 0);
    double lastX = starting.getX();
    boolean extendAlready;
    double lastY = starting.getY();
    double lastH = starting.getHeading();

    Path toSubmersible, strafeToSample1, behindSample1, pushSample1, backwardsFromSample1, strafeBehindSample2, pushSample2, goBackWards, goForwards, strafeOnce, toSubmersiblecool, backUp1, strafeTwice, strafeThrice, toSubmersible3rd, backwardsFromSubmersible, strafeToSpecimen3, strafeToSubmersible, forwardsToSubmersible, backwardsFromSubmersible2, strafeToLastSample, strafeToSubmersible2, forwardsToSubmersible2;
    public void init(){
        follower = new Follower(hardwareMap);
        robot.init(hardwareMap);
        follower.setStartingPose(starting);
        armTimer = new Timer();
        pathTimer = new Timer();
        extendAlready = false;
        buildPaths();
        robot.rotateServo.setPosition(.741);

    }


    public void buildPaths(){
        toSubmersible = newPath(31.6, 64, 0);
        strafeToSample1 = newPath(31.6, 40, lastH);
        behindSample1 = newPath(65, 26, lastH);
        pushSample1 = newPath(13,26, lastH);
        backwardsFromSample1 = newPath(70,24, lastH);
        strafeBehindSample2 = newPath(70 , 14, lastH);
        pushSample2 = newPath(12, 14, lastH);
        goBackWards = newPath(30, 28, -179);
        goForwards = newPath(6, 49, -179);
        strafeOnce = newPath(6, 64, 0);
        toSubmersiblecool = newPath(31.6, 64, 0);
        backUp1 = newPath(6, 64, 0);
        strafeTwice = newPath(6, 49, -179);
        strafeThrice = strafeOnce = newPath(6, 64, 0);
        toSubmersible3rd = newPath(31.6, 65, 0);
        backwardsFromSubmersible = newPath(6, 64, 0);
        strafeToSpecimen3 = newPath(6, 49, -179);
        strafeToSubmersible = newPath(6, 66, 0);
        forwardsToSubmersible = newPath(31.6, 66, 0);
        backwardsFromSubmersible2 = newPath(6, 64, 0);
        strafeToLastSample = newPath(6, 49, -179);
        strafeToSubmersible2 = newPath(6, 66, 0 );
        forwardsToSubmersible2 = newPath(31.6, 66, 0);






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
            case STRAFE_TO_SAMPLE1:
                if (follower.getCurrentPath().isAtParametricEnd() && actionState == ActionState.PICK_UP_SPECIMEN_1){
                    follower.followPath(strafeToSample1);
                    setPathState(PathState.BEHIND_SAMPLE1);
                }
                break;
            case BEHIND_SAMPLE1:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(behindSample1);
                    setPathState(PathState.PUSH_SAMPLE1);
                }
            case PUSH_SAMPLE1:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(pushSample1);
                    setPathState(PathState.BACKWARDS_FROM_SAMPLE1);
                }
                break;
            case BACKWARDS_FROM_SAMPLE1:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(backwardsFromSample1);
                    setPathState(PathState.STRAFE_BEHIND_SAMPLE2);
                }
                break;
            case STRAFE_BEHIND_SAMPLE2:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(strafeBehindSample2);
                    setPathState(PathState.PUSH_SAMPLE2);
                }
                break;

//            case TURN:
//                if (follower.getCurrentPath().isAtParametricEnd()){
//                    follower.followPath(turnRo  bot);
//                    setPathState(PathState.PUSH_SAMPLE2);
//                }
            case PUSH_SAMPLE2:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(pushSample2);
                    setPathState(PathState.GO_BACKWARDS);
                }
                break;
            case GO_BACKWARDS:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(goBackWards);
                    setPathState(PathState.GO_FORWARDS);
                }
            case GO_FORWARDS:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(goForwards);
                    setPathState(PathState.STRAFE_ONCE);
                }
            case STRAFE_ONCE:
                if (actionState == ActionState.HANG_SPECIMEN_1){
                    follower.followPath(strafeOnce);
                }
            case TO_SUBMERSIBLE_1:
                if (actionState == ActionState.HANG_SPECIMEN_1){
                    follower.followPath(toSubmersiblecool);
                    setPathState(PathState.BACKWARDS_FROM_SUBMERSIBLE);
                    rotateServoDown();
                }
                break;
            case BACKWARDS_FROM_SUBMERSIBLE:
                if(actionState == ActionState.PICK_UP_SPECIMEN_2){
                    follower.followPath(backUp1);
                    setPathState(PathState.TO_SPECIMEN_2);
                }
            case TO_SPECIMEN_2:
                if (!follower.isBusy()){
                    follower.followPath(strafeTwice);
                    setPathState(PathState.STRAFE_TO_SUBMERSIBLE_3);
                }
            case STRAFE_TO_SUBMERSIBLE_3:
                if (actionState == ActionState.HANG_SPECIMEN_2){
                    follower.followPath(strafeThrice);
                    setPathState(PathState.GO_FORWARD);
                }
                break;
            case GO_FORWARD:
                if (!follower.isBusy()){
                    follower.followPath(toSubmersible3rd);
                    setPathState(PathState.TO_SPECIMEN_3);
                }
                break;
            case TO_SPECIMEN_3:
                if (actionState == ActionState.PICK_UP_SPECIMEN_3){
                    follower.followPath(backwardsFromSubmersible);
                    setPathState(PathState.BACKWARDS_FROM_SUBMERSIBLE_3);
                }
                break;
            case BACKWARDS_FROM_SUBMERSIBLE_3:
                if (!follower.isBusy()){
                    follower.followPath(strafeToSpecimen3);
                    setPathState(PathState.PICK_UP_4TH_SPECIMEN);
                }
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
                closeClaw();
                armExtend(-1200);
                armUp(2300);
                if (robot.armExtension.getCurrentPosition() < -1199 && robot.armVertical.getCurrentPosition() > 2290) {
                    setActionState(ActionState.HANG_PRELOAD);

                }
                break;
            case HANG_PRELOAD:
                if (!follower.isBusy()){
                    armExtend(-350);
                    armUp(2100);
                }
                if (robot.armExtension.getCurrentPosition() > -351 && robot.armVertical.getCurrentPosition() < 2101) {
                    setActionState(ActionState.PICK_UP_SPECIMEN_1);
                }
//                telemetry.update();
                break;
            case PICK_UP_SPECIMEN_1:
                if (armTimer.getElapsedTimeSeconds() < 1  &&!extendAlready){
                    armExtend(-2);
                    armUp(-50);
                    openClaw();
                    extendAlready = true;


                }
                rotateServoForward();

                if (pathState == PathState.STRAFE_ONCE && !follower.isBusy()){
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
                armExtend(-1200);
                armUp(2300);
                setActionState(ActionState.HANG_SPECIMEN_1);
                break;
            case HANG_SPECIMEN_1:
                closeClaw();
                if (pathState == PathState.BACKWARDS_FROM_SUBMERSIBLE && follower.getCurrentPath().isAtParametricEnd()){
                    armExtend(-350);
                    armUp(2100);


                    if (robot.armExtension.getCurrentPosition() > -450 && robot.armVertical.getCurrentPosition() < 2150) {
                        openClaw();
                        setActionState(ActionState.PICK_UP_SPECIMEN_2);
                    }
                }

            case PICK_UP_SPECIMEN_2:
//                armExtend(-100);
//                armUp(0);
//                openClaw();
                if (pathState == PathState.GO_FORWARD && follower.getCurrentPath().isAtParametricEnd()){
                    armExtend(-263);
                    armUp(-50);
                    if (robot.armExtension.getCurrentPosition() < 500){
                        rotateServoForward();
                    }
                    if (robot.armExtension.getCurrentPosition() < -200 && robot.armVertical.getCurrentPosition() < -5){
                        closeClaw();
                        if (armTimer.getElapsedTimeSeconds() > 1){
                            armTimer.resetTimer();
                        }
                        if(armTimer.getElapsedTimeSeconds() > 0.3){
                            setActionState(ActionState.RAISE_ARM_2);
                        }
                    };
                }
            case RAISE_ARM_2:
                closeClaw();
                armExtend(-1200);
                armUp(2300);
                setActionState(ActionState.HANG_SPECIMEN_2);
            case HANG_SPECIMEN_2:
                closeClaw();
                if (pathState == PathState.TO_SPECIMEN_3  && follower.getCurrentPath().isAtParametricEnd()){
                    armExtend(-350);
                    armUp(2100);

                    if (robot.armExtension.getCurrentPosition() > -450 && robot.armVertical.getCurrentPosition() < 2150) {
                        openClaw();
                        setActionState(ActionState.PICK_UP_SPECIMEN_3);
                    }
                }
            case PICK_UP_SPECIMEN_3:
                if (pathState == PathState.PICK_UP_4TH_SPECIMEN && follower.getCurrentPath().isAtParametricEnd()){
                    armExtend(-263);
                    armUp(-50);
                    if (robot.armExtension.getCurrentPosition() < 500){
                        rotateServoForward();
                    }
                    if (robot.armExtension.getCurrentPosition() < -200 && robot.armVertical.getCurrentPosition() < -5){
                        closeClaw();
                        if (armTimer.getElapsedTimeSeconds() > 1){
                            armTimer.resetTimer();
                        }
                        if(armTimer.getElapsedTimeSeconds() > 0.3){
                            setActionState(ActionState.RAISE_ARM_2);
                        }
                    };
                }
            default:
                stop();
        }
    }
    public void loop() {

        telemetry.addData("Position",follower.getPose());
        telemetry.addData("Current Path: ", pathState);
        telemetry.addData("Timer: ", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Action State",actionState);
        telemetry.addData("Arm Extension Position", robot.armExtension.getCurrentPosition());
        autonomousActionUpdate();
        telemetry.update();
        autonomousPathUpdate();
        follower.update();

    }
}
