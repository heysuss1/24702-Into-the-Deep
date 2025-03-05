package org.firstinspires.ftc.teamcode.runmodes.Autos.RewrittenAutos;

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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "States Specimen Auto - that works")
public class RewrittenSpecimenAuto extends OpMode {
    Follower follower;
    //727 741 4771
    int ARM_CONSTANT = 570;
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
        PUSH_SPECIMENS,
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
    public static Pose preloadPose = new Pose(30.2, 61, 0);
//        toSubmersible.setPathEndTimeoutConstraint(500);
//        toSubmersible.setZeroPowerAccelerationMultiplier(2.8);
    public static Pose strafeToSample1Pose = new Pose( 29, 40, 0);
    public static Pose behindSample1Pose = new Pose(61, 27, 0);
    public static Pose pushSample1Pose = new Pose(17,26, 0);
    public static Pose backwardsFromSample1Pose = new Pose(58,24, 0);
    public static Pose strafeBehindSample2Pose = new Pose(58, 15.5, 0);
    public static Pose pushSample2Pose = new Pose(17, 15.5, 0);
//        pushSample2.setPathEndTValueConstraint(.94);
    public static Pose backwardsFromSample2Pose = new Pose(58, 18, 0);
    public static Pose strafeBehindSample3Pose = new Pose(58, 11.5, 0);
    public static Pose pushSample3Pose = new Pose(17, 12, 0 );
    public static Pose pickUpSpecimen1Pose = new Pose(9.3, 49, Math.toRadians(179));
    public static Pose forwardToSubmersible1Pose = new Pose(30.65, 60, 0);
    public static Pose pickUpSpecimen2Pose = new Pose(10, 49, Math.toRadians(179));
    public static Pose forwardToSubmersible2Pose = new Pose(30.65, 65, 0);
    public static Pose pickUpSpecimen3Pose = new Pose(10, 49, Math.toRadians(179));
    public static Pose forwardsToSubmersible3Pose = new Pose(30.45, 67, 0);
    public static Pose pickUpSpecimen4Pose = new Pose(10, 49, Math.toRadians(179));
    public static Pose forwardToSubmersible4Pose = new Pose(31, 69, 0);
    public static Pose parkPose = new Pose(10, 30, 0);
    double lastX = starting.getX();
    boolean extendAlready;
    double lastY = starting.getY();
    double counter;
    int roll, pitch;
    double lastH = starting.getHeading();

    public PathChain toSubmersible, pushSpecimens, strafeToSample1, behindSample1, pushSample1, backwardsFromSample1, strafeBehindSample2, pushSample2, backwardsFromSample2, strafeBehindSample3, pushSample3, goBackWards, goForwards, pickUpSpecimen1, strafeToSubmersible1, forwardToSubmersible1, backwardsFromSubmersible1, pickUpSpecimen2, strafeToSubmersible4, forwardToSubmersible2, backwardsFromSubmersible3, pickUpSpecimen3, strafeToSubmersible3, forwardsToSubmersible3, backwardsFromSubmersible2, pickUpSpecimen4, strafeToSubmersible2, forwardToSubmersible4, park;
    public void init(){
        follower = new Follower(hardwareMap);
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
        toSubmersible = follower.pathBuilder()
                .addPath(new BezierLine(new Point(starting), new Point(preloadPose)))
                        .setLinearHeadingInterpolation(starting.getHeading(), preloadPose.getHeading())
                .setZeroPowerAccelerationMultiplier(2.7)
                                .build();
//        toSubmersible.setZeroPowerAccelerationMultiplier(2.8);
        strafeToSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadPose), new Point(strafeToSample1Pose)))
                .setLinearHeadingInterpolation(preloadPose.getHeading(), strafeToSample1Pose.getHeading())
                .build();
         behindSample1 = follower.pathBuilder().addPath(new BezierLine(new Point(strafeToSample1Pose), new Point(behindSample1Pose)))
                .setLinearHeadingInterpolation(strafeToSample1Pose.getHeading(), behindSample1Pose.getHeading())
                 .build();
         pushSample1 = follower.pathBuilder().addPath(new BezierLine(new Point(behindSample1Pose), new Point(pushSample1Pose)))
                .setLinearHeadingInterpolation(behindSample1Pose.getHeading(), pushSample1Pose.getHeading())
                 .build();
         backwardsFromSample1 = follower.pathBuilder().addPath(new BezierLine(new Point(pushSample1Pose), new Point(backwardsFromSample1Pose)))
                .setLinearHeadingInterpolation(pushSample1Pose.getHeading(), backwardsFromSample1Pose.getHeading())
                 .build();
         strafeBehindSample2 = follower.pathBuilder().addPath(new BezierLine(new Point(backwardsFromSample1Pose), new Point(strafeBehindSample2Pose)))
                .setLinearHeadingInterpolation(backwardsFromSample1Pose.getHeading(), strafeBehindSample2Pose.getHeading())
                 .build();
         pushSample2 = follower.pathBuilder()       .addPath(new BezierLine(new Point(strafeBehindSample2Pose), new Point(pushSample2Pose)))
                .setLinearHeadingInterpolation(strafeBehindSample2Pose.getHeading(), (pushSample2Pose.getHeading()))
                 .build();
         backwardsFromSample2 = follower.pathBuilder()       .addPath(new BezierLine(new Point(pushSample2Pose), new Point(backwardsFromSample2Pose)))
                .setLinearHeadingInterpolation(pushSample2Pose.getHeading(), backwardsFromSample2Pose.getHeading())
                 .build();
         strafeBehindSample3 = follower.pathBuilder().addPath(new BezierLine(new Point(backwardsFromSample2Pose), new Point(strafeBehindSample3Pose)))
                .setLinearHeadingInterpolation(backwardsFromSample2Pose.getHeading(), strafeBehindSample3Pose.getHeading())
                 .build();
         pushSample3 = follower.pathBuilder().addPath(new BezierLine(new Point(strafeBehindSample3Pose), new Point(pushSample3Pose)))
                .setLinearHeadingInterpolation(strafeBehindSample3Pose.getHeading(), pushSample3Pose.getHeading())
                .build();
        pickUpSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushSample3Pose), new Point(pickUpSpecimen1Pose)))
                .setLinearHeadingInterpolation(pushSample3Pose.getHeading(), pickUpSpecimen1Pose.getHeading())
                .build();
        forwardToSubmersible1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpSpecimen1Pose), new Point(forwardToSubmersible1Pose)))
                .setZeroPowerAccelerationMultiplier(3.8)
                .setLinearHeadingInterpolation(pickUpSpecimen1Pose.getHeading(), forwardToSubmersible1Pose.getHeading())
                .build();
        pickUpSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(forwardToSubmersible1Pose), new Point(pickUpSpecimen2Pose)))
                .setLinearHeadingInterpolation(forwardToSubmersible1Pose.getHeading(), pickUpSpecimen2Pose.getHeading())
                .build();
        forwardToSubmersible2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpSpecimen2Pose), new Point(forwardToSubmersible2Pose)))
                .setLinearHeadingInterpolation(pickUpSpecimen2Pose.getHeading(), forwardToSubmersible2Pose.getHeading())
                .build();
        pickUpSpecimen3 =follower.pathBuilder()
                .addPath(new BezierLine(new Point(forwardToSubmersible2Pose), new Point(pickUpSpecimen3Pose)))
                .setLinearHeadingInterpolation(forwardToSubmersible2Pose.getHeading(), pickUpSpecimen3Pose.getHeading())
                .build();
        forwardsToSubmersible3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpSpecimen3Pose), new Point(forwardsToSubmersible3Pose)))
                .setLinearHeadingInterpolation(pickUpSpecimen3Pose.getHeading(), forwardsToSubmersible3Pose.getHeading())
                .build();
        pickUpSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(forwardsToSubmersible3Pose), new Point(pickUpSpecimen4Pose)))
                .setLinearHeadingInterpolation(forwardsToSubmersible3Pose.getHeading(), pickUpSpecimen4Pose.getHeading())
                .build();
        forwardToSubmersible4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpSpecimen4Pose), new Point(forwardToSubmersible4Pose)))
                .setLinearHeadingInterpolation(pickUpSpecimen4Pose.getHeading(), forwardToSubmersible4Pose.getHeading())
//                .setZeroPowerAccelerationMultiplier(3)
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(forwardToSubmersible4Pose), new Point(parkPose)))
                .setLinearHeadingInterpolation(forwardToSubmersible4Pose.getHeading(), parkPose.getHeading())
                .build();
    }


    public void setPathState(PathState state){
        pathState = state;
        pathTimer.resetTimer();
    }
        public void autonomousPathUpdate(){
        switch (pathState){
            case GO_TO_SUBMERSIBLE:
                follower.setMaxPower(0.8);
                follower.followPath(toSubmersible);
                setPathState(PathState.STRAFE_TO_SAMPLE1);
                break;
            case STRAFE_TO_SAMPLE1:
                if (!follower.isBusy() && actionState == ActionState.PICK_UP_SPECIMEN_1){
                    follower.setMaxPower(1);
                    follower.followPath(strafeToSample1);
                    setPathState(PathState.BEHIND_SAMPLE1);
                }
                break;
            case BEHIND_SAMPLE1:
                if (!follower.isBusy()){
                    follower.followPath(behindSample1);
                    setPathState(PathState.PUSH_SAMPLE1);
                }
                break;
            case PUSH_SAMPLE1:
                if (!follower.isBusy()){
                    follower.followPath(pushSample1);
                    setPathState(PathState.BACKWARDS_FROM_SAMPLE1);
                }
                break;
            case BACKWARDS_FROM_SAMPLE1:
                if (!follower.isBusy()){
                    follower.followPath(backwardsFromSample1);
                    setPathState(PathState.STRAFE_BEHIND_SAMPLE2);
                }
                break;
            case STRAFE_BEHIND_SAMPLE2:
                if (!follower.isBusy()){
                    follower.followPath(strafeBehindSample2);
                    setPathState(PathState.PUSH_SAMPLE2);
                }
                break;

            case PUSH_SAMPLE2:
                if (!follower.isBusy()){
                    follower.followPath(pushSample2);
                    setPathState(PathState.BACKWARDS_FROM_SAMPLE2);
                }
                break;
            case BACKWARDS_FROM_SAMPLE2:
                if (!follower.isBusy()){
                    follower.followPath(backwardsFromSample2);
                    setPathState(PathState.STRAFE_BEHIND_SAMPLE3);
                }
                break;
            case STRAFE_BEHIND_SAMPLE3:
                if (!follower.isBusy()){
                    follower.followPath(strafeBehindSample3);
                    setPathState(PathState.PUSH_SAMPLE3);
                }
                break;
            case PUSH_SAMPLE3:
                if (!follower.isBusy()){
                    follower.followPath(pushSample3);
                    setPathState(PathState.STRAFE_TO_SPECIMEN);
                }
                break;
            case STRAFE_TO_SPECIMEN:
                if (!follower.isBusy()){
                    follower.followPath(pickUpSpecimen1);
                    setPathState(PathState.FORWARDS_TO_SUBMERSIBLE_1);
                }
                break;
            case FORWARDS_TO_SUBMERSIBLE_1:
                if (actionState == ActionState.HANG_SPECIMEN_1){
                    follower.followPath(forwardToSubmersible1, true);
                    setPathState(PathState.PICK_UP_SPECIMEN_2);
                    rotateServoDown();
                }
                break;
            case PICK_UP_SPECIMEN_2:
//                openClaw();
                if (!follower.isBusy() && actionState == ActionState.PICK_UP_SPECIMEN_2){
                    follower.followPath(pickUpSpecimen2);
                    rotateServoForward();
                    setPathState(PathState.FORWARDS_TO_SUBMERSIBLE_2);
                }
                break;
            case FORWARDS_TO_SUBMERSIBLE_2:
                if (actionState == ActionState.HANG_SPECIMEN_2){
                    follower.followPath(forwardToSubmersible2, true);
                    setPathState(PathState.PICK_UP_SPECIMEN_3);
                    rotateServoDown();
                }
                break;
            case PICK_UP_SPECIMEN_3:
                if (!follower.isBusy() && actionState == ActionState.PICK_UP_SPECIMEN_3){
                    follower.followPath(pickUpSpecimen3);
                    setPathState(PathState.FORWARDS_TO_SUBMERSIBLE_3);
                }
                break;
            case FORWARDS_TO_SUBMERSIBLE_3:
                if (actionState == ActionState.HANG_SPECIMEN_3){
                    follower.followPath(forwardsToSubmersible3, true);
                    setPathState(PathState.PICK_UP_SPECIMEN_4);
                    rotateServoDown();
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
                    follower.followPath(forwardToSubmersible4, true);
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
        robot.pitch = 90;
        robot.roll = 0;
    }
    public void rotateServoDown() {robot.pitch = 175;}
    public void openClaw(){
        robot.claw.setPosition(0.15);
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
    public void raiseArms(ActionState nextState){
        extendAlready = false;
        closeClaw();
        armUp(1436-ARM_CONSTANT);
        if (robot.armVertical.getCurrentPosition() > (200-ARM_CONSTANT)){
            armExtend(-876);
            setActionState(nextState);

        };
        if (robot.armVertical.getCurrentPosition() > (1298-ARM_CONSTANT) && robot.armExtension.getCurrentPosition() < -858){
            setActionState(nextState);
            rotateServoDown();
        }
    }

    public void hangSpecimen(ActionState nextState){
        closeClaw();
        rotateServoDown();
        if (!follower.isBusy()){
            armExtend(-140);
            armUp(1100-ARM_CONSTANT);
            hungSpecimen = true;
        }
        if (robot.armVertical.getCurrentPosition() < 1120-ARM_CONSTANT && robot.armExtension.getCurrentPosition() > -210 && hungSpecimen) {
            counter++;
            openClaw();
            setActionState(nextState);
        }}

    public void autonomousActionUpdate(){
        switch(actionState){
            case RAISE_ARMS:
//                setActionState(ActionState.RAISE_ARMS);
                closeClaw();
                rotateServoDown();
                armExtend(-840);
                armUp(1436-ARM_CONSTANT);//may be 2400
                if (robot.armExtension.getCurrentPosition() < -835 && robot.armVertical.getCurrentPosition() > (150-ARM_CONSTANT)) {
                    robot.pitch = 175;
                    setActionState(ActionState.HANG_PRELOAD);
                }
                break;
            case HANG_PRELOAD:
                if (!follower.isBusy() || armTimer.getElapsedTimeSeconds() > 3.2){
                    armExtend(-140);
                    armUp(1100-ARM_CONSTANT);
                }
                if (robot.armExtension.getCurrentPosition() > -205 && robot.armVertical.getCurrentPosition() < (1120-ARM_CONSTANT)|| armTimer.getElapsedTimeSeconds() > 3) {
                    setActionState(ActionState.PICK_UP_SPECIMEN_1);
                }
//                telemetry.update();
                break;
            case PICK_UP_SPECIMEN_1:
                if (armTimer.getElapsedTimeSeconds() < 0.3  &&!extendAlready){
                    armExtend(-2);
                    armUp(-130-ARM_CONSTANT);
                    openClaw();
                    extendAlready = true;
                }
                rotateServoForward();
                if (pathState == PathState.FORWARDS_TO_SUBMERSIBLE_1 && !follower.isBusy()){
                    armExtend(-455);
                    if (robot.armExtension.getCurrentPosition() < -415){
                        if (armTimer.getElapsedTimeSeconds() > 1){
                            armTimer.resetTimer();
                        }
                        closeClaw();
                        if(armTimer.getElapsedTimeSeconds() > 0.05){
                            setActionState(ActionState.RAISE_ARM_1);
                        }
                    };
                }
                break;
            case RAISE_ARM_1:
                raiseArms(ActionState.HANG_SPECIMEN_1);
                break;
            case HANG_SPECIMEN_1:
                hangSpecimen(ActionState.PICK_UP_SPECIMEN_2);
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
                    armExtend(-455);
                    if (robot.armExtension.getCurrentPosition() < -415){
                        if (armTimer.getElapsedTimeSeconds() > 1){
                            armTimer.resetTimer();
                        }
                        closeClaw();
                        if(armTimer.getElapsedTimeSeconds() > 0.05){
                            setActionState(ActionState.RAISE_ARM_2);
                        }
                    };
                }
                break;
            case RAISE_ARM_2:
                raiseArms(ActionState.HANG_SPECIMEN_2);
                break;
            case HANG_SPECIMEN_2:
                hangSpecimen(ActionState.PICK_UP_SPECIMEN_3);
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
                    armExtend(-455);
                    if (robot.armExtension.getCurrentPosition() < -415){
                        if (armTimer.getElapsedTimeSeconds() > 1){
                            armTimer.resetTimer();
                        }
                        closeClaw();
                        if(armTimer.getElapsedTimeSeconds() > 0.05){
                            setActionState(ActionState.RAISE_ARM_3);
                        }
                    };
                }
                break;
            case RAISE_ARM_3:
                raiseArms(ActionState.HANG_SPECIMEN_3);
                break;
            case HANG_SPECIMEN_3:
                hangSpecimen(ActionState.PICK_UP_SPECIMEN_4);
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
                    armExtend(-420);
                    if (robot.armExtension.getCurrentPosition() < -380){
                        if (armTimer.getElapsedTimeSeconds() > 1){
                            armTimer.resetTimer();
                        }
                        closeClaw();
                        if(armTimer.getElapsedTimeSeconds() > 0.05){
                            setActionState(ActionState.RAISE_ARM_4);
                        }
                    };
                }
                break;
            case RAISE_ARM_4:
               raiseArms(ActionState.HANG_SPECIMEN_4);
                break;
            case HANG_SPECIMEN_4:
                hangSpecimen(ActionState.PARK);
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
        robot.diddylate(robot.pitch, robot.roll);
        telemetry.update();
        autonomousPathUpdate();


    }
}

