package org.firstinspires.ftc.teamcode.runmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Waiter;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "Diddy Claw Specimen Auto")
public class DiddySpecimenAuto extends OpMode {
    Follower follower;
    //727 741 4771
    int ARM_CONSTANT = 720;
    Timer armTimer, pathTimer;
    boolean hungSpecimen = false;
    Waiter waiter;
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
        HANG_SPECIMEN_3,
        PICK_UP_SPECIMEN_4,
        RAISE_ARM_4,
        HANG_SPECIMEN_4,
        PARK

    }
    enum PathState{
        GO_TO_SUBMERSIBLE,
        STRAFE_TO_SAMPLE1,
        BEHIND_SAMPLE1,
        PUSH_SAMPLE1,
        BACKWARDS_FROM_SAMPLE1,
        STRAFE_BEHIND_SAMPLE2,
        PUSH_SAMPLE2,
        STRAFE_TO_SPECIMEN,
        GO_BACKWARDS,
        GO_FORWARDS,
        BACKWARDS_FROM_SAMPLE2,
        STRAFE_BEHIND_SAMPLE3,
        PUSH_SAMPLE3,
        PICK_UP_SPECIMEN_1,
        STRAFE_TO_SUBMERSIBLE_1,
        FORWARDS_TO_SUBMERSIBLE_1,

        BACKWARDS_FROM_SUBMERSIBLE_1,
        PICK_UP_SPECIMEN_2,
        STRAFE_TO_SUBMERSIBLE_2,
        FORWARDS_TO_SUBMERSIBLE_2,
        BACKWARDS_FROM_SUBMERSIBLE_2,
        PICK_UP_SPECIMEN_3,
        STRAFE_TO_SUBMERSIBLE_3,

        FORWARDS_TO_SUBMERSIBLE_3,
        BACKWARDS_FROM_SUBMERSIBLE_3,
        PICK_UP_SPECIMEN_4,
        FORWARD_TO_SUBMERSIBLE_4,
        STRAFE_TO_PARK,

        DONE
    }
    PathState pathState = PathState.GO_TO_SUBMERSIBLE;
    ActionState actionState = ActionState.RAISE_ARMS;
    Pose starting = new Pose(5, 64, 0);
    double lastX = starting.getX();
    boolean extendAlready;
    double lastY = starting.getY();
    double counter;
    int roll, pitch;
    double lastH = starting.getHeading();

    Path toSubmersible, strafeToSample1, behindSample1, pushSample1, backwardsFromSample1, strafeBehindSample2, pushSample2, backwardsFromSample2, strafeBehindSample3, pushSample3, goBackWards, goForwards, pickUpSpecimen1, strafeToSubmersible1, forwardToSubmersible1, backwardsFromSubmersible1, pickUpSpecimen2, strafeToSubmersible4, forwardToSubmersible2, backwardsFromSubmersible3, pickUpSpecimen3, strafeToSubmersible3, forwardsToSubmersible3, backwardsFromSubmersible2, pickUpSpecimen4, strafeToSubmersible2, forwardToSubmersible4, park;
    public void init(){
        follower = new Follower(hardwareMap);
//        toSubmersible = new Path(new BezierLine (new Point (5, 64,  Point.CARTESIAN), new Point(30, 64,  Point.CARTESIAN)));
//        toSubmersible.setConstantHeadingInterpolation(0);
//        strafeToSample1 = new Path(new BezierLine(new Point(30, 64, Point.CARTESIAN), new Point(30, 40, Point.CARTESIAN)));
//        strafeToSample1.setConstantHeadingInterpolation(0);

//        behindSample1 = newPath(60, 24, 0);
//        pushSample1 = newPath(13,26, 0);
        buildPaths();
        waiter = new Waiter();
        robot.init(hardwareMap);
        follower.setStartingPose(starting);
//        follower.followPath(toSubmersible);
        follower.setMaxPower(1);
        armTimer = new Timer();
        pathTimer = new Timer();
        extendAlready = false;


    }


    public void buildPaths(){
        toSubmersible = newPath(28.2, 61, 0);
        toSubmersible.setPathEndTimeoutConstraint(500);
        strafeToSample1 = newPath(29, 40, 0);
//        strafeToSample1.setPathEndTimeoutConstraint(100);
        behindSample1 = newPath(61, 27, 0);
//        behindSample1.setPathEndTimeoutConstraint(100);
        pushSample1 = newPath(17,26, 0);
//        pushSample1.setPathEndTValueConstraint(.94);
//        pushSample1.setPathEndTimeoutConstraint(100);
        backwardsFromSample1 = newPath(58,24, lastH);
//        backwardsFromSample1.setPathEndTValueConstraint(.94);

//        backwardsFromSample1.setPathEndTimeoutConstraint(100);
        strafeBehindSample2 = newPath(58 , 15.5, lastH);
//        strafeBehindSample2.setPathEndTValueConstraint(.94);

//        strafeBehindSample2.setPathEndTimeoutConstraint(100);
        pushSample2 = newPath(17, 15.5, lastH);
        pushSample2.setPathEndTValueConstraint(.94);

        backwardsFromSample2 = newPath(58, 18, lastH);
//        backwardsFromSample2.setPathEndTValueConstraint(.94);

        strafeBehindSample3 = newPath(58, 12, lastH);
        pushSample3 = newPath(17, 12, lastH );
        pickUpSpecimen1 = newPath(9.3, 49, 179);
//        goBackWards = newPath(30, 35, -179);
//        goForwards = newPath(8.2, 49, -179);
//        strafeToSubmersible1 = newPath(6, 64, 0);
        forwardToSubmersible1 = newPath(30.4, 65, 0);
//        backwardsFromSubmersible1 = newPath(20, 64, 0);
        pickUpSpecimen2 = newPath(10, 49, 179);
//        strafeToSubmersible2 = newPath(7, 64, 0);
        forwardToSubmersible2 = newPath(30.3, 66, 5);
//        backwardsFromSubmersible2 = newPath(20, 64, 0);
        pickUpSpecimen3 = newPath(10, 49, 179);
//        strafeToSubmersible3 = newPath(6, 68, 0);
        forwardsToSubmersible3 = newPath(30.3, 68, 3);
//        backwardsFromSubmersible3= newPath(20, 64, 0);
        pickUpSpecimen4 = newPath(10, 49, 179);
//        strafeToSubmersible4 = newPath(6, 66, 0);
        forwardToSubmersible4 = newPath(30, 70, 0);
        park = newPath(7, 49, 0);






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
        path.setPathEndTValueConstraint(.95);
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
//                follower.setMaxPower(0.9);
                follower.followPath(toSubmersible);
                setPathState(PathState.STRAFE_TO_SAMPLE1);
                break;
            case STRAFE_TO_SAMPLE1:
//                follower.setMaxPower(1);
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
                break;
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

            case PUSH_SAMPLE2:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(pushSample2);
                    setPathState(PathState.BACKWARDS_FROM_SAMPLE2);
                }
                break;
            case BACKWARDS_FROM_SAMPLE2:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(backwardsFromSample2);
                    setPathState(PathState.STRAFE_BEHIND_SAMPLE3);
                }
                break;
            case STRAFE_BEHIND_SAMPLE3:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(strafeBehindSample3);
                    setPathState(PathState.PUSH_SAMPLE3);
                }
                break;
            case PUSH_SAMPLE3:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(pushSample3);
                    setPathState(PathState.STRAFE_TO_SPECIMEN);
                }
                break;
            case STRAFE_TO_SPECIMEN:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(pickUpSpecimen1);
                    setPathState(PathState.FORWARDS_TO_SUBMERSIBLE_1);
                }
                break;
//            case GO_FORWARDS:
//                if (follower.getCurrentPath().isAtParametricEnd()){
//                    follower.followPath(goForwards);
//                    setPathState(PathState.STRAFE_TO_SUBMERSIBLE_1);
//                }
//                break;
            case STRAFE_TO_SUBMERSIBLE_1:
                if (actionState == ActionState.HANG_SPECIMEN_1){
                    follower.followPath(strafeToSubmersible1);
                    setPathState(PathState.FORWARDS_TO_SUBMERSIBLE_1);
                }
                break;
            case FORWARDS_TO_SUBMERSIBLE_1:
                if (actionState == ActionState.HANG_SPECIMEN_1){
                    follower.followPath(forwardToSubmersible1);
                    setPathState(PathState.PICK_UP_SPECIMEN_2);
                    rotateServoDown();
                }
                break;
            case BACKWARDS_FROM_SUBMERSIBLE_1:
                rotateServoDown();
                if(actionState == ActionState.PICK_UP_SPECIMEN_2 && !follower.isBusy()){
                    follower.followPath(backwardsFromSubmersible1);
                    setPathState(PathState.PICK_UP_SPECIMEN_2);
                }
                break;
            case PICK_UP_SPECIMEN_2:
//                openClaw();
                if (follower.getCurrentPath().isAtParametricEnd() && actionState == ActionState.PICK_UP_SPECIMEN_2){
                    follower.followPath(pickUpSpecimen2);
                    rotateServoForward();
                    setPathState(PathState.FORWARDS_TO_SUBMERSIBLE_2);
                }
                break;
            case STRAFE_TO_SUBMERSIBLE_2:
                if (actionState == ActionState.HANG_SPECIMEN_2){
                    follower.followPath(strafeToSubmersible2);
                    setPathState(PathState.FORWARDS_TO_SUBMERSIBLE_2);
                }
                break;
            case FORWARDS_TO_SUBMERSIBLE_2:
                if (actionState == ActionState.HANG_SPECIMEN_2){
                    follower.followPath(forwardToSubmersible2);
                    setPathState(PathState.PICK_UP_SPECIMEN_3);
                    rotateServoDown();
                }
                break;
//            case BACKWARDS_FROM_SUBMERSIBLE_2:
//                rotateServoDown();
//                if(actionState == ActionState.PICK_UP_SPECIMEN_3 && !follower.isBusy()){
//                    follower.followPath(backwardsFromSubmersible2);
//                    setPathState(PathState.PICK_UP_SPECIMEN_3);
//                }
//                break;
            case PICK_UP_SPECIMEN_3:
                if (!follower.isBusy() && actionState == ActionState.PICK_UP_SPECIMEN_3){
                    follower.followPath(pickUpSpecimen3);
                    setPathState(PathState.FORWARDS_TO_SUBMERSIBLE_3);
                }
                break;
            case STRAFE_TO_SUBMERSIBLE_3:
                if (actionState == ActionState.HANG_SPECIMEN_3){
                    follower.followPath(strafeToSubmersible3);
                    setPathState(PathState.FORWARDS_TO_SUBMERSIBLE_3);
                }
                break;
            case FORWARDS_TO_SUBMERSIBLE_3:
                if (actionState == ActionState.HANG_SPECIMEN_3){
                    follower.followPath(forwardsToSubmersible3);
                    setPathState(PathState.PICK_UP_SPECIMEN_4);
                    rotateServoDown();
                }
                break;
            case BACKWARDS_FROM_SUBMERSIBLE_3:
                rotateServoDown();
                if(actionState == ActionState.PICK_UP_SPECIMEN_3 && !follower.isBusy()){
                    follower.followPath(backwardsFromSubmersible3);
                    setPathState(PathState.PICK_UP_SPECIMEN_4);
                }
                break;
            case PICK_UP_SPECIMEN_4:
                if (!follower.isBusy() && actionState == ActionState.PICK_UP_SPECIMEN_4){
                    openClaw();
                    follower.followPath(pickUpSpecimen4);
                    setPathState(PathState.FORWARD_TO_SUBMERSIBLE_4);
                }
                break;
            case FORWARD_TO_SUBMERSIBLE_4:
                if (actionState == ActionState.HANG_SPECIMEN_4){
                    follower.followPath(forwardToSubmersible4);
                    setPathState(PathState.STRAFE_TO_PARK);
                }
                break;
            case STRAFE_TO_PARK:
                if (actionState == ActionState.PARK){
                    follower.followPath(park);
                    setPathState(PathState.DONE);
                }
                break;
            default:
                stop();

        }
    }

    public void rotateServoForward(){
        pitch = 90;
        roll = 0;
    }
    public void rotateServoDown() {pitch = 175;}
    public void openClaw(){
        robot.claw.setPosition(0.1);
    }

    public void closeClaw(){
        robot.claw.setPosition(0.4);
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
                rotateServoDown();
                armExtend(-1200);
                armUp(1436-ARM_CONSTANT);//may be 2400
                if (robot.armExtension.getCurrentPosition() < -1199 && robot.armVertical.getCurrentPosition() > (150-ARM_CONSTANT)) {
                    setActionState(ActionState.HANG_PRELOAD);

                }
                break;
            case HANG_PRELOAD:
                if (!follower.isBusy() || armTimer.getElapsedTimeSeconds() > 3.2){
                    armExtend(-200);
                    armUp(1100-ARM_CONSTANT);
                }
                if (robot.armExtension.getCurrentPosition() > -305 && robot.armVertical.getCurrentPosition() < (1101-ARM_CONSTANT)|| armTimer.getElapsedTimeSeconds() > 3) {
                    setActionState(ActionState.PICK_UP_SPECIMEN_1);
                }
//                telemetry.update();
                break;
            case PICK_UP_SPECIMEN_1:
                if (armTimer.getElapsedTimeSeconds() < 1  &&!extendAlready){
                    armExtend(-2);
                    armUp(-130-ARM_CONSTANT);
                    openClaw();
                    extendAlready = true;


                }
                rotateServoForward();

                if (pathState == PathState.FORWARDS_TO_SUBMERSIBLE_1 && !follower.isBusy()){
                    armExtend(-700);
                    if (robot.armExtension.getCurrentPosition() < -695){
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
                extendAlready = false;
                closeClaw();
                armUp(1436-ARM_CONSTANT);
                if (robot.armVertical.getCurrentPosition() > (200-ARM_CONSTANT)){
                    armExtend(-1200);
                    setActionState(ActionState.HANG_SPECIMEN_1);

                };
                if (pathState == PathState.BACKWARDS_FROM_SAMPLE1){
//                    armExtend(-1200);
                }
                if (robot.armVertical.getCurrentPosition() > (1298-ARM_CONSTANT) && robot.armExtension.getCurrentPosition() < -1196){
                    setActionState(ActionState.HANG_SPECIMEN_1);
                    rotateServoDown();
                }
                break;
            case HANG_SPECIMEN_1:
                closeClaw();
                rotateServoDown();
                if (follower.getCurrentPath().isAtParametricEnd() ){
                    armExtend(-200);
                    armUp(1100-ARM_CONSTANT);
                    hungSpecimen = true;
                    waiter.start(2000);
                }
                if (robot.armVertical.getCurrentPosition() < 1101-ARM_CONSTANT && robot.armExtension.getCurrentPosition() > -201 && hungSpecimen) {
                    counter++;
                    openClaw();
                    setActionState(ActionState.PICK_UP_SPECIMEN_2);
                }
                break;
            case PICK_UP_SPECIMEN_2:
                hungSpecimen = false;

                if (armTimer.getElapsedTimeSeconds() < 1  &&!extendAlready){
                    armExtend(-2);
                    armUp(-130-ARM_CONSTANT);
                    openClaw();
                    extendAlready = true;
                }
                rotateServoForward();
                if (pathState == PathState.FORWARDS_TO_SUBMERSIBLE_2 && !follower.isBusy()){
                    armExtend(-660);
                    if (robot.armExtension.getCurrentPosition() < -655){
                        if (armTimer.getElapsedTimeSeconds() > 1){
                            armTimer.resetTimer();
                        }
                        closeClaw();
                        if(armTimer.getElapsedTimeSeconds() > 0.3){
                            setActionState(ActionState.RAISE_ARM_2);
                        }
                    };
                }
                break;
            case RAISE_ARM_2:
                extendAlready = false;
                closeClaw();
                armUp(1436-ARM_CONSTANT);
                if (robot.armVertical.getCurrentPosition() > (200-ARM_CONSTANT)){
                    armExtend(-1200);
                    setActionState(ActionState.HANG_SPECIMEN_2);
                };
                if (pathState == PathState.BACKWARDS_FROM_SAMPLE1){
//                    armExtend(-1200);
                }
                if (robot.armVertical.getCurrentPosition() > (1298-ARM_CONSTANT) && robot.armExtension.getCurrentPosition() < -1196){
                    setActionState(ActionState.HANG_SPECIMEN_2);
                    rotateServoDown();
                }
                break;
            case HANG_SPECIMEN_2:
                closeClaw();
                rotateServoDown();
                if (follower.getCurrentPath().isAtParametricEnd() && pathState == PathState.PICK_UP_SPECIMEN_3){
                    armExtend(-200);
                    armUp(1100-ARM_CONSTANT);
                    hungSpecimen = true;
                    waiter.start(2000);
                }
                if (robot.armVertical.getCurrentPosition() < 1101-ARM_CONSTANT && robot.armExtension.getCurrentPosition() > -201 && hungSpecimen) {
                    counter++;
                    openClaw();
                    setActionState(ActionState.PICK_UP_SPECIMEN_3);
                }
                break;
            case PICK_UP_SPECIMEN_3:
                hungSpecimen = false;

                if (armTimer.getElapsedTimeSeconds() < 1  &&!extendAlready){
                    armExtend(-2);
                    armUp(-120-ARM_CONSTANT);
                    openClaw();
                    extendAlready = true;
                }
                rotateServoForward();

                if (pathState == PathState.FORWARDS_TO_SUBMERSIBLE_3 && !follower.isBusy()){
                    armExtend(-665);
                    if (robot.armExtension.getCurrentPosition() < -660){
                        if (armTimer.getElapsedTimeSeconds() > 1){
                            armTimer.resetTimer();
                        }
                        closeClaw();
                        if(armTimer.getElapsedTimeSeconds() > 0.3){
                            setActionState(ActionState.RAISE_ARM_3);
                        }
                    };
                }
                break;
            case RAISE_ARM_3:
                extendAlready = false;
                closeClaw();
                armUp(1436-ARM_CONSTANT);
                if (robot.armVertical.getCurrentPosition() > (200-ARM_CONSTANT)){
                    armExtend(-1200);
                    setActionState(ActionState.HANG_SPECIMEN_3);

                };
                if (pathState == PathState.BACKWARDS_FROM_SAMPLE1){
//                    armExtend(-1200);
                }
                if (robot.armVertical.getCurrentPosition() > (1298-ARM_CONSTANT) && robot.armExtension.getCurrentPosition() < -1196){
                    setActionState(ActionState.HANG_SPECIMEN_3);
                    rotateServoDown();
                }
                break;
            case HANG_SPECIMEN_3:
                closeClaw();
                rotateServoDown();
                if (follower.getCurrentPath().isAtParametricEnd()){
                    armExtend(-200);
                    armUp(1100-ARM_CONSTANT);
                    hungSpecimen = true;
                    waiter.start(2000);
                }
                if (robot.armVertical.getCurrentPosition() < 1101-ARM_CONSTANT && robot.armExtension.getCurrentPosition() > -201 && hungSpecimen) {
                    counter++;
                    openClaw();
                    setActionState(ActionState.PICK_UP_SPECIMEN_4);
                }
                break;
            case PICK_UP_SPECIMEN_4:
                hungSpecimen = false;

                if (armTimer.getElapsedTimeSeconds() < 1  &&!extendAlready){
                    armExtend(-2);
                    armUp(-120-ARM_CONSTANT);
                    openClaw();
                    extendAlready = true;
                }
                rotateServoForward();
                if (pathState == PathState.FORWARD_TO_SUBMERSIBLE_4 && !follower.isBusy()){
                    armExtend(-660);
                    if (robot.armExtension.getCurrentPosition() < -655){
                        if (armTimer.getElapsedTimeSeconds() > 1){
                            armTimer.resetTimer();
                        }
                        closeClaw();
                        if(armTimer.getElapsedTimeSeconds() > 0.3){
                            setActionState(ActionState.RAISE_ARM_4);
                        }
                    };
                }
                break;
            case RAISE_ARM_4:
                extendAlready = false;
                closeClaw();
                armUp(1436-ARM_CONSTANT);
                if (robot.armVertical.getCurrentPosition() > (200-ARM_CONSTANT)){
                    armExtend(-1200);
                    setActionState(ActionState.HANG_SPECIMEN_4);
                };
                if (pathState == PathState.BACKWARDS_FROM_SAMPLE1){
//                    armExtend(-1200);
                }
                if (robot.armVertical.getCurrentPosition() > (1298-ARM_CONSTANT) && robot.armExtension.getCurrentPosition() < -1196){
                    setActionState(ActionState.HANG_SPECIMEN_4);
                    rotateServoDown();
                }
                break;
            case HANG_SPECIMEN_4:
                closeClaw();
                rotateServoDown();
                if (follower.getCurrentPath().isAtParametricEnd() && pathState == PathState.STRAFE_TO_PARK){
                    armExtend(-200);
                    armUp(1050-ARM_CONSTANT);
                    hungSpecimen = true;
                    waiter.start(2000);
                }
                if (robot.armVertical.getCurrentPosition() < 1051-ARM_CONSTANT && robot.armExtension.getCurrentPosition() > -201 && hungSpecimen) {
                    counter++;
                    openClaw();
                    setActionState(ActionState.PARK);
                }
                break;
            default:
                stop();
        }
    }
    public void loop() {
        follower.update();
        telemetry.addData("Position",follower.getPose());
        telemetry.addData("Current Path: ", pathState);
        telemetry.addData("Timer: ", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Action State",actionState);
        telemetry.addData("Arm Extension Position", robot.armExtension.getCurrentPosition());
        telemetry.addData("Power of rf", robot.rf.getPower());
//        telemetry.addData("Path timer", pathTimer);
        autonomousActionUpdate();
        robot.diddylate(pitch, roll);
        telemetry.update();
        autonomousPathUpdate();


    }
}
