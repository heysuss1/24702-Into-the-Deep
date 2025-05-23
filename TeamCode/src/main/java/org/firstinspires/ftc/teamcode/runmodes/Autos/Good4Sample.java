package org.firstinspires.ftc.teamcode.runmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous (name = "HOPEFULLY! consistent auto")
public class Good4Sample extends OpMode {
        Follower follower;
        Hardware robot = Hardware.getInstance();
        Pose starting = new Pose(5, 126, 0);
        Timer timer;
        int pitch, roll;
        int clawCloser = 0;
        int ARM_CONSTANT = 570;
        Pose bucket = new Pose(5, 127, Point.CARTESIAN);
        Pose frontOfSubmersible = new Pose(34, 71, Point.CARTESIAN);
        Pose frontOfObservationZone = new Pose(9, 17, Point.CARTESIAN);
        boolean clawClosed = false;

        double lastX = starting.getX();
        double distance = 0;
        boolean hasSample = (distance < 0.5);
        double lastY = starting.getY();
        double lastH = starting.getHeading();

        ElapsedTime armTimer;
        int counter = 0;

        boolean isClawClosed;
        Path preloadBasket, backUp, toSample1, toBucket, toSample2, toBucketFromSample2, toSample3, toBucketFromSample3, toParking;

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
        org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State state = org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.SCORE_PRELOAD_BASKET;
        org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState actionState = org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.RAISE_ARMS;
//    PathBuilder toSubmersible;

        //Assumes robot starts at (5, 65)

        public void buildPaths(){
        /*
        int num = 0;
        switch (num){
            case 0:
        }
         */
            preloadBasket = (newPath(18, 127, 0));
            backUp = newPath(12, 127, 0);
//        backUp = newPath(19, 78, lastH);
            toSample1 = newPath(4.4, 88, -90);
            toBucket = newPath(10.5, 122, -130);
            toSample2 = newPath(12.4, 88, 0);
            toBucketFromSample2 = newPath(10, 123, -130);
            toSample3 = newPath(22.7, 88, 0);
            toBucketFromSample3 = newPath(11.5, 123, -130);
            toParking = newPath(-6, 80, -179);

        }
        public void init(){
            follower = new Follower(hardwareMap);
            robot.init(hardwareMap);
            lastH = follower.getPose().getHeading();
            follower.setStartingPose(starting);
            follower.setMaxPower(0.6985);
            isClawClosed = false;
            robot.armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotateArmForwards();
            closeClaw();
            timer = new Timer();
            armTimer = new ElapsedTime();
            buildPaths();
        }




        public void setPathState(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State prevState){
            state = prevState;
            timer.resetTimer();
            autonomousPathUpdate();
        }
        public void autonomousPathUpdate(){
            switch (state) {
                case SCORE_PRELOAD_BASKET:
                    if (actionState == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.SCORE_SAMPLE){
                        follower.followPath(preloadBasket);
                        setPathState(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_BACKWARDS);
                    }
                    break;
                case GO_BACKWARDS:
                    if (actionState == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.GRAB_SAMPLE1){
                        follower.followPath(backUp);
                        setPathState(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_TO_SAMPLE1);
                    }
                    break;
                case GO_TO_SAMPLE1:
                    if (follower.getCurrentPath().isAtParametricEnd() && actionState == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.GRAB_SAMPLE1 ){
                        follower.followPath(toSample1);
                        closeClaw();
                        setPathState(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_TO_BASKET);
                    }
                    break;
                case GO_TO_BASKET:
                    if (actionState == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.PUT_IN_BUCKET && !follower.isBusy()&& timer.getElapsedTimeSeconds() > 3) {
                        rotateArmBackWards();
                        follower.followPath(toBucket);
                        setPathState(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_TO_SAMPLE2);
                    }
                    break;
                case GO_TO_SAMPLE2:
                    if (actionState == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.GRAB_SAMPLE2 && follower.getCurrentPath().isAtParametricEnd()){
                        follower.followPath(toSample2);
                        setPathState(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_TO_BASKET_FROM_SAMPLE_2);
                    }
                    break;
                case GO_TO_BASKET_FROM_SAMPLE_2:
                    if (actionState == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.PUT_IN_BUCKET_2 && !follower.isBusy() && timer.getElapsedTimeSeconds() > 3) {
                        follower.followPath(toBucketFromSample2);
                        setPathState(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_TO_SAMPLE3);
                    }
                    break;
                case GO_TO_SAMPLE3:
                    if (actionState == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.GRAB_SAMPLE3 && !follower.isBusy()){
                        follower.followPath(toSample3);
                        rotateArmForwards();
                        setPathState(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_TO_BASKET_FROM_SAMPLE_3);
                    }
                    break;
                case GO_TO_BASKET_FROM_SAMPLE_3:
                    if (actionState == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.PUT_IN_BUCKET_3 && follower.getCurrentPath().isAtParametricEnd() & timer.getElapsedTimeSeconds() > 2){
                        follower.followPath(toBucketFromSample3);
                        setPathState(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_TO_PARKING);
                    }
                    break;
                case GO_TO_PARKING:
                    if (actionState == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.PARK && follower.getCurrentPath().isAtParametricEnd()){
                        follower.followPath(toParking);
                    }
                    break;
//            default:
//                stop();
            }
        }

        public void setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState prevState){
            actionState = prevState;
            armTimer.reset();
            autonomousActionUpdate();
        }
        public boolean hasTheSample(){
            distance = robot.colorSensor.getDistance(DistanceUnit.INCH);
            hasSample = (distance < 0.5);
            return hasSample;
        }
        public void rotateArmBackWards(){
            robot.pitch = 15;
            robot.roll = 0;
        }
        public void rotateArmForwards() {robot.pitch = 175;}
        public void sidewaysClaw(){
            robot.roll = 45;
        }
        public void normalClaw(){
            robot.roll = 0;
        }
        public void openClaw(){
            robot.claw.setPosition(0.1);
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

        public void autonomousActionUpdate(){
            clawCloser = 0;
            switch (actionState){
                case RAISE_ARMS:
                    rotateArmForwards();
                    armExtend(-2650);
                    armUp(2050-ARM_CONSTANT);
                    if (robot.armExtension.getCurrentPosition() < -2640 && robot.armVertical.getCurrentPosition() > (2040-ARM_CONSTANT)) {
                        setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.SCORE_SAMPLE);
                    }
                    break;
                case SCORE_SAMPLE:
                    if (state == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_BACKWARDS && !follower.isBusy() && armTimer.seconds() > 1){
                        openClaw();
                        setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.OPEN_CLAW);
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
                    if (armTimer.seconds() > 0.3) setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.GRAB_SAMPLE1);
                    break;
                case GRAB_SAMPLE1:
                    openClaw();
                    if (/*state == State.G
                O_TO_BASKET && follower.getCurrentPath().isAtParametricEnd()*/ follower.getPose().getY() < 118.5){
                        // We need to figure out how to do this but for now Thread.sleep(300);
                        sidewaysClaw();
                        armUp(-700 -ARM_CONSTANT);
                        armExtend(-720);
                    }
//                if (follower.getPose().getY() > 113){
//                    armUp(-470-ARM_CONSTANT);
//                }
                    if(!follower.isBusy() && state == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_TO_BASKET && robot.armVertical.getCurrentPosition() < (-648-ARM_CONSTANT) && robot.armVertical.getCurrentPosition() < -648){
                        if (armTimer.seconds() > 2.6){
                            closeClaw();
                            setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.CLOSE_CLAW);
                            armTimer.reset();
                        }
//resetreset                    if (armTimer.seconds() > 0.3 && clawClosed){
//                        setAction(ActionState.PUT_IN_BUCKET);
//                    }
                    }
                    break;
                case CLOSE_CLAW:
                    closeClaw();
                    setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.PUT_IN_BUCKET);
                    break;
                case PUT_IN_BUCKET:
                    clawCloser = 0;
                    closeClaw();
                    if (state == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_TO_SAMPLE2 && !follower.isBusy()){
                        normalClaw();
                        armUp(2750-ARM_CONSTANT);
                        armExtend(-2300);
                        rotateArmBackWards();
                        if (follower.getCurrentPath().isAtParametricEnd() && robot.armVertical.getCurrentPosition() > (2720-ARM_CONSTANT) && robot.armExtension.getCurrentPosition() < -2280 && !robot.armVertical.isBusy()){
                            if (armTimer.seconds() > 1){
                                armTimer.reset();
                            }
//                        if (robot.armVertical.getCurrentPosition() > (2650-ARM_CONSTANT)){
                            openClaw();
                            if (armTimer.seconds() > 0.4){
                                setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.OPEN_CLAW_2);
                            }
                        }
                    }
                    break;
                case OPEN_CLAW_2:
                    openClaw();
                    setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.GRAB_SAMPLE2);
                    break;
                case GRAB_SAMPLE2:
                    openClaw();
                    rotateArmForwards();
                    sidewaysClaw();
                    if (/*state == State.GO_TO_BASKET && follower.getCurrentPath().isAtParametricEnd()*/ follower.getPose().getY() < 115){
                        // We need to figure out how to do this but for now Thread.sleep(300);
                        armUp(-600-ARM_CONSTANT);
                        armExtend(-770);

                    }
//                if (follower.getPose().getY() > 113){
//                    armUp(-470-ARM_CONSTANT);
//                }
                    if(!follower.isBusy() && state == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_TO_BASKET_FROM_SAMPLE_2 && robot.armVertical.getCurrentPosition() < (-575-ARM_CONSTANT) && robot.armVertical.getCurrentPosition() < -765){
                        if (armTimer.seconds() > 3){
                            closeClaw();
                            setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.CLOSE_CLAW_2);
                            armTimer.reset();
                        }
//                    if (armTimer.seconds() > 0.3 && clawClosed){
//                        setAction(ActionState.PUT_IN_BUCKET);
//                    }
                    }
                    break;
                case CLOSE_CLAW_2:
                    closeClaw();
                    setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.PUT_IN_BUCKET_2);
                    break;
                case PUT_IN_BUCKET_2:
                    clawCloser = 0;
                    closeClaw();
                    normalClaw();
                    if (state == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_TO_SAMPLE3 && !follower.isBusy()){
                        armUp(2750-ARM_CONSTANT);
                        armExtend(-2260);
                        rotateArmBackWards();

                        if (follower.getCurrentPath().isAtParametricEnd() && robot.armExtension.getCurrentPosition() < -2250 && !robot.armVertical.isBusy()){
                            if (armTimer.seconds() > 1){
                                armTimer.reset();
                            }
//                        if (robot.armVertical.getCurrentPosition() > (2650-ARM_CONSTANT)){
                            openClaw();
                            if (armTimer.seconds() > 0.4){
                                setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.GRAB_SAMPLE3);
                            }
                        }
                    }
                    break;
                case GRAB_SAMPLE3:
                    if (clawCloser == 0){
                        openClaw();
                    }
                    sidewaysClaw();
                    rotateArmForwards();
                    if (/*state == State.GO_TO_BASKET && follower.getCurrentPath().isAtParametricEnd()*/ follower.getPose().getY() < 115 && !isClawClosed){
                        // We need to figure out how to do this but for now Thread.sleep(300);
                        armUp(-300-ARM_CONSTANT);
                        isClawClosed = true;
                        armExtend(-5);

                    }
//                if (follower.getPose().getY() > 113){
//                    armUp(-470-ARM_CONSTANT);
//                }
                    if(follower.getCurrentPath().isAtParametricEnd() && state == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_TO_BASKET_FROM_SAMPLE_3){
                        if (robot.armVertical.getCurrentPosition() < -295-ARM_CONSTANT){
                            armExtend(-720);
                        }

                        if (robot.armExtension.getCurrentPosition() < -485){
                            armUp(-750-ARM_CONSTANT);

                        }
//                        if (robot.armVertical.getCurrentPosition() < -200-ARM_CONSTANT){
//                            armExtend(-725);
//                        }
                        if (armTimer.seconds() > 3 && robot.armExtension.getCurrentPosition() < -495 && robot.armVertical.getCurrentPosition() < -720-ARM_CONSTANT){
                            closeClaw();
                            setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.CLOSE_CLAW_3);
                            armTimer.reset();
                        }
//                        closeClaw();
//                        if (armTimer.seconds() > 0.3){
//                            setAction(ActionState.PUT_IN_BUCKET_3);
//                        }
                    }
                    break;
                case CLOSE_CLAW_3:
                    closeClaw();
                    if (armTimer.seconds() > 2){
                        setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.PUT_IN_BUCKET_3);
                    }
                    break;
                case PUT_IN_BUCKET_3:
                    normalClaw();
                    clawCloser = 0;
                    closeClaw();
                    if (state == org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.State.GO_TO_PARKING && !follower.isBusy()){
                        armUp(2700-ARM_CONSTANT);
                        armExtend(-2200);
                        rotateArmBackWards();

                        if (!follower.isBusy() && robot.armVertical.getCurrentPosition() > (2720-ARM_CONSTANT)){
                            if (armTimer.seconds() > 1){
                                armTimer.reset();
                            }
//                        if (robot.armVertical.getCurrentPosition() > (2650-ARM_CONSTANT)){
                            openClaw();
                            if (armTimer.seconds() > 0.4){
                                setAction(org.firstinspires.ftc.teamcode.runmodes.Autos.OnlySamplesAuto.ActionState.PARK);
                            }
                        }
                    }
                    break;
                case PARK:
                    if (armTimer.seconds() > 1){
                        rotateArmForwards();
                        armUp(580-ARM_CONSTANT);
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
            distance = robot.colorSensor.getDistance(DistanceUnit.INCH);
            autonomousPathUpdate();
            robot.diddylate(robot.pitch, robot.roll);
            autonomousActionUpdate();
            follower.update();

        }
}
