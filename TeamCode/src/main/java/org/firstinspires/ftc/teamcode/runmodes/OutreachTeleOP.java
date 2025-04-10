package org.firstinspires.ftc.teamcode.runmodes;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;


//close value right = .154, open = .298, close left = .684, open  = .538
@TeleOp(name = "Outreach TeleOP (maxSpeed=0.2)")
public class OutreachTeleOP extends LinearOpMode {
    Hardware robot = Hardware.getInstance();
    Follower follower;
    boolean isAligned;
    PIDFController extensionPID;
    PIDFController verticalPID;

    //    int pitch, roll;
//    int addRoll, addPitch;
    public static Pose forwardToSubmersible2Pose = new Pose(30.65, 65, 0);
    public static Pose toHumanPlayerZonePose = new Pose(13, 49, 0);

    enum ArmState {
        PARALLEL_ARM,
        HANG_SPECIMEN,
        HIGH_BASKET,
        DONE
    }

    ArmState armState = ArmState.DONE;
    double currentHeading;
    ElapsedTime elapsedTime;
    double output;
    boolean goToPosition = false;
    boolean autoDrive = false;

    //PHUHS
    public void runOpMode() {
        int position = 0;
        robot.init(hardwareMap);
        telemetry.addData("" + "Status", "Hello, Drivers!");
        robot.setSpeed(1);
        follower = new Follower(hardwareMap);
//        follower.setStartingPose(new Pose(63, 95));
        follower.setStartingPose(new Pose(10, 49, 0));
        telemetry.update();
//        Path toSubmersible, toObservationZone, toBuckets, pickUpSpecimen;
        Path toSubmersible, toHumanPlayerZone;

        double forward, sideways, turning, max;
        double scaleFactor = 0;
        boolean showTelemetry = false;
//         outlining locations of game parts
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        extensionPID = new PIDFController(0.02, 0, 0.001, 0);
        verticalPID = new PIDFController(0.0219, 0, 0.001, 0);
//        Pose pickUpSpecimen = new Pose(12, 8, -90);
        Pose hangSpecimen = new Pose(42, 30, 0);
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        Gamepad.LedEffect redAlliance = new Gamepad.LedEffect.Builder()
                .addStep(.4, 0, 1, 1000)
                .addStep(1, 0, 0, 200)
                .build();

        Gamepad.LedEffect blueAlliance = new Gamepad.LedEffect.Builder()
                .addStep(.4, 0, 1, 1000)
                .addStep(0, 0, 1, 200)
                .build();
        Pose starting = new Pose(robot.startX, robot.startY);
        robot.setSpeed(0.33);
        waitForStart();
        robot.setSpeed(0.33);
        elapsedTime = new ElapsedTime();
        elapsedTime.startTime();
        boolean clawIsOpen = false;
        int[] clawPositions = {45, 20, 0, -15, -45};
        int currentClawPosition = 0;
        currentClawPosition = Range.clip(currentClawPosition, 0, 3);
        boolean pressingLT = false;
        boolean pressingRT = false;
        boolean useExtension = false;
        boolean clawForwards = false;
//        boolean parallelArm = false;
//        boolean highBasket = false;
//        boolean hangSpecimen = false;
        boolean armVerticalTooFar = false;
        boolean tooFar = false;
        boolean isStalling = false;
        boolean usePID = false;
        double clawAmount = 0;
        double ticks = 0;
        String colorPrediction = "", currentAlliance = "red";
        double red, green, blue, distance;
        boolean hasSample;
        int armVerticalTarget = 0;
        int armExtensionTarget = 0;
        while (opModeIsActive()) {
            toSubmersible = new Path(new BezierLine(new Point(follower.getPose()), new Point(forwardToSubmersible2Pose)));
            toHumanPlayerZone = new Path(new BezierLine(new Point(follower.getPose()), new Point(toHumanPlayerZonePose)));
            double currentXpose = follower.getPose().getX();
            double currentYpose = follower.getPose().getY();
            int currentHeading = (int) Math.round(follower.getPose().getHeading());

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            isAligned = Math.toDegrees(follower.getPose().getHeading()) < 2 && Math.toDegrees(follower.getPose().getHeading()) > 0;

            //gamepad1 = Driver 1

            if (elapsedTime.seconds() > 90 && elapsedTime.seconds() < 91) {
                gamepad1.rumble(0.5, 0.5, 1000);
                gamepad2.rumble(0.5, 0.5, 1000);
            } else if (elapsedTime.seconds() > 113 && elapsedTime.seconds() < 120) {
                gamepad1.rumble(1000, 1000, 7000);
                gamepad2.rumble(1000, 1000, 7000);
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                toSubmersible.setLinearHeadingInterpolation(follower.getPose().getHeading(), 0);
                autoDrive = true;
                follower.followPath(toSubmersible);
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                toHumanPlayerZone.setLinearHeadingInterpolation(follower.getPose().getHeading(), -179);
                autoDrive = true;
                ;
                follower.followPath(toHumanPlayerZone);
            }

//            if (gamepad1.dpad_up){
//                robot.setPower(1, 1, 1, 1);
//            }
//            else if (gamepad1.dpad_right){
//                robot.setPower(-1, 1, -1, 1);
//            }
//            else if (gamepad1.dpad_left){
//                robot.setPower(1, -1, -1, 1);
//            }
//            else if (gamepad1.dpad_down){
//                robot.setPower(-1, -1, -1, -1);
//            }
//            else{
//                robot.setPower(0,0,0 ,0);
//            }


//            switch (armState){
//                case PARALLEL_ARM:
//                    robot.armVertical.setPower(verticalPID.calculate(robot.armVertical.getCurrentPosition(), -80));
//                    if (robot.armVertical.getCurrentPosition() > -81){
////                    armState = ArmState.DONE;
//                    }
//                    telemetry.addData("state", armState);
//                    break;
//                case HANG_SPECIMEN:
//                    robot.armExtension.setPower(extensionPID.calculate(robot.armExtension.getCurrentPosition(), -1200));
//                    robot.armVertical.setPower(verticalPID.calculate(robot.armVertical.getCurrentPosition(), 2400));
//                    if (robot.armExtension.getCurrentPosition() <-1198 && robot.armVertical.getCurrentPosition() > 2398){
////                        armState = ArmState.DONE;
//                    }
//                    break;
//                case HIGH_BASKET:
//                    robot.armExtension.setPower(extensionPID.calculate(robot.armExtension.getCurrentPosition(),-2350));
//                    robot.armVertical.setPower(verticalPID.calculate(robot.armVertical.getCurrentPosition(), 3000));
//                    if (robot.armExtension.getCurrentPosition() <-2348 && robot.armVertical.getCurrentPosition() > 2995) {
////                        armState = ArmState.DONE;
//                    }
//                    break;
//                case DONE:
//                    break;
//                default:
//                    break;
//            }
//            robot.armExtension.setPower(extensionPID.calculate(robot.armExtension.getCurrentPosition(), -1200));
//            robot.armVertical.setPower(verticalPID.calculate(robot.armVertical.getCurrentPosition(), 2400));


//            gamepad1.setLedColor();
            // math for mechanum wheels

            forward = -(Math.atan(5 * gamepad1.left_stick_y) / Math.atan(5));
            sideways = (Math.atan(5 * gamepad1.left_stick_x) / Math.atan(5));
            turning = (Math.atan(5 * gamepad1.right_stick_x) / Math.atan(5));
            max = Math.max(Math.abs(forward - sideways - turning), Math.max(Math.abs(forward + sideways - turning), Math.max(Math.abs(forward + sideways + turning), Math.abs(forward + turning - sideways))));
            if (max > robot.maxSpeed) {
                scaleFactor = robot.maxSpeed / max;
            } else {
                scaleFactor = robot.maxSpeed;
            }
            scaleFactor *= Math.max(Math.abs(1), 0.2);
            if (!autoDrive) {
                robot.setPower((forward - sideways - turning) * scaleFactor, (forward + sideways - turning) * scaleFactor, (forward + sideways + turning) * scaleFactor, (forward + turning - sideways) * scaleFactor);
            }
            //only runs if the game button is he  ld down
            //gamepad 2 = driver 2


            if (currentGamepad2.y && !previousGamepad2.y) {
//                robot.rotateServo.setPosition(0.174);
                currentClawPosition = 2;
                robot.pitch = 20;
            }
            if (currentGamepad2.b && !previousGamepad2.y && !currentGamepad2.start) {
//                robot.rotateServo.setPosition(0.38);
                currentClawPosition = 2;
                robot.pitch = 90;
                robot.roll = 0;
            }
            if (currentGamepad2.a && !previousGamepad2.a) {
//                robot.rotateServo.setPosition(0.726);
                currentClawPosition = 2;
                robot.pitch = 175;
            }
            if (robot.armVertical.getCurrentPosition() > 3130) {
                armVerticalTooFar = true;
            } else {
                armVerticalTooFar = false;
                // stops from going too far and tipping
            }
            if (!follower.isBusy()) {
                autoDrive = false;
            }

            if (currentGamepad2.x && !previousGamepad2.x) {
//                robot.armVertical.setTargetPosition(0);
                armVerticalTarget = 0;
                currentClawPosition = 2;
                robot.pitch = 90;
                robot.roll = 0;
                robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                goToPosition = true;
                useExtension = false;
                // back to parallel
            }

            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                armVerticalTarget = 1100;
                armExtensionTarget = -140;
                useExtension = true;
                goToPosition = true;
            }

            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
                armExtensionTarget = -900;//originally was -840, decreased by 60
                armVerticalTarget = 1436;
                goToPosition = true;
                useExtension = true;
            }
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                robot.armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // manual reset
            }
//            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
//                toObservationZone = newPath(frontOfSubmersible.getX(), frontOfObservationZone.getY(), 0);
//                follower.followPath(toObservationZone);
//                // go to human player
//            }
//            output = extensionPID.calculate(robot.armExtension.getCurrentPosition(), -1200);

//            robot.armExtension.setPower(output);
//            output = verticalPID.calculate(robot.armVertical.getCurrentPosition(), 2400);
//            robot.armVertical.setPower(output);
            if (gamepad1.right_trigger > 0.1) {
                robot.setSpeed(0.35);
            } else {
                robot.setSpeed(.35);
                // slow robot down based on right trigger preasure
            }
            if (robot.armExtension.getCurrentPosition() > -2600) {
                tooFar = false;
            } else {
                tooFar = true;
                // extension position
            }
            if (gamepad2.left_stick_y < -0.1 && (!tooFar)) {
                telemetry.addData("Status", "This is going, should be going forward");
                robot.armExtension.setPower(-1);
                usePID = false;
            } else if (gamepad2.left_stick_y > 0.1) {
                telemetry.addData("Status", "This is going, should be going backwards");
                robot.armExtension.setPower(1);
                usePID = false;
            } else {
                if (!goToPosition) {
//                    robot.armExtension.setPower(-0.0009*Math.sin(Math.PI*robot.armVertical.getCurrentPosition()/7600));
                    robot.armExtension.setPower(0);
                }
            }
            if (currentGamepad1.y && !previousGamepad1.y) {
                alignHeading();
            }
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                if (currentClawPosition > 0) {
                    currentClawPosition -= 1;
                }
                robot.roll = clawPositions[currentClawPosition];
            }
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                if (currentClawPosition < clawPositions.length - 1) {
                    currentClawPosition += 1;
                }
                robot.roll = clawPositions[currentClawPosition];
            }

//            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up){
//                roll = 0;
//            }if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down){
//                roll = 45;
//            }if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left){
//                roll = -15;
//            }if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right){
//                roll = 20;
//            }
            // press y, align so can pick up specimen from human player
//            if ((robot.colorSensor.getDistance(DistanceUnit.INCH) < 5) && !hasSensed){
//                robot.armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                hasSensed = true;
//            } else if (!(robot.colorSensor.getDistance(DistanceUnit.INCH) < 5)){
//                hasSensed = false;
//            }
//            if (robot.extensionLimiter.getState() && !extensionLimiterPressed){
//                robot.armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                extensionLimiterPressed = true;
//            } else if (!robot.extensionLimiter.getState()){
//                extensionLimiterPressed = false;
//            }
            if (gamepad2.right_stick_y > 0.1) {
                robot.armVertical.setPower(-1);
                usePID = false;
                // pull y stick down, move arm down
            } else if (gamepad2.right_stick_y < -0.1 && !armVerticalTooFar) {
                robot.armVertical.setPower(1);
                usePID = false;
                // push y stick up, move arm up
            } else {
                if (!goToPosition && !usePID) {
                    robot.armVertical.setPower(0);
                }
                // if your not goin to a set position don't move the arm
            }
            if ((gamepad2.left_trigger > 0.1) && !pressingLT) {
                if (!clawIsOpen) {
                    //Open claw
                    robot.claw.setPosition(0.15);
                    clawIsOpen = true;
                } else {
                    //Close claw
                    robot.claw.setPosition(0.4);
                    clawIsOpen = false;
                }
                pressingLT = true;
            } else if (!(gamepad2.left_trigger > 0.1)) {
                pressingLT = false;
            }
            // press left trigger once to open and again to close


            //right trigger = half-closed
            if ((gamepad2.right_trigger > 0.1) && !pressingRT) {
//                robot.openClaw(0.5);
                robot.armVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.armVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                clawIsOpen = false;
                pressingRT = true;
            } else if (!(gamepad2.right_trigger > 0.1)) {
                pressingRT = false;
            }

            //maxSpeed stuff by Carter:

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down && (robot.getSpeed() > 0)) {
                robot.setSpeed(robot.getSpeed() - 0.1);
            }
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up && (robot.getSpeed() < 1)) {
                robot.setSpeed(robot.getSpeed() + 0.1);
            }

            if (currentGamepad1.back && !previousGamepad1.back) {
                if (currentAlliance == "red") {
                    currentAlliance = "blue";
                    gamepad1.runLedEffect(blueAlliance);
                    gamepad2.runLedEffect(blueAlliance);
                } else {
                    currentAlliance = "red";
                    gamepad1.runLedEffect(redAlliance);
                    gamepad2.runLedEffect(redAlliance);
                }
            }

            //color sensor
            red = robot.colorSensor.red();
            green = robot.colorSensor.green();
            blue = robot.colorSensor.blue();
            distance = robot.colorSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("RGB", red + ", " + green + ", " + blue);
            telemetry.addData("Distance", distance);
//        if (robot.colorSensor.getDistance(DistanceUnit.INCH) < distanceThreshold){
//            TelemetryA.addLine("Has something!");
//            TelemetryA.addData("Current Color is (aka Raw Light)", robot.colorSensor.getRawLightDetected());
//        }
            hasSample = (distance < 0.5);

            //Note this is carter's programming
            if (hasSample) {
                if (green > red && red > blue) {
                    colorPrediction = "yellow";
                    if (clawIsOpen) gamepad2.rumble(100, 100, 100);
                } else if (red > green && red > blue) {
                    colorPrediction = "red";
                    if (clawIsOpen && currentAlliance == "red") gamepad2.rumble(100, 100, 100);
                } else if (blue > red && blue > green) {
                    colorPrediction = "blue";
                    if (clawIsOpen && currentAlliance == "blue") gamepad2.rumble(100, 100, 100);
                } else {
                    colorPrediction = "idfk";
                }
            } else {
                colorPrediction = "no sample";
            }

            if (currentGamepad2.back && !previousGamepad2.back) {
                showTelemetry = !showTelemetry;
            }

//            robot.armExtension.setPower(1);
//            robot.armExtension.setTargetPosition();
//            robot.armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//
//            if(robot.armExtension.getCurrent(CurrentUnit.AMPS) > 5){
//                isStalling = true;
//            } else{
//                isStalling = false;
//            }
////            if(robot.armExtension.getCurrent(CurrentUnit.AMPS ) < 0.5){
////                isStalling = false;
////            }
//            if(isStalling && (position < 11)){
//                robot.armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            } else{
//                if (robot.armExtension.getCurrent(CurrentUnit.AMPS) < 0.3){
//                    robot.armExtension.setPower(1)
//                }
//            }
            robot.diddylate(robot.pitch, robot.roll);
            follower.update();
            if (showTelemetry) {
                telemetry.addData("Current Alliance", currentAlliance);
                telemetry.addData("Arm Vertical", robot.armVertical.getCurrentPosition());
                telemetry.addData("Arm Extension Position", robot.armExtension.getCurrentPosition());
                telemetry.addData("Pitch", robot.pitch);
                telemetry.addData("Roll", robot.roll);
                telemetry.addData("Claw counter", clawAmount);
                telemetry.addData("Current heading is", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.addData("Current position", follower.getPose());
                telemetry.addData("Currenet x: ", currentXpose);
                telemetry.addData("Next x", currentXpose + 20);
                telemetry.addData("Max Speed", robot.getSpeed());
                telemetry.addData("Has Sample?", hasSample);
                telemetry.addData("Color", colorPrediction);
                telemetry.addLine("Time Elapsed:" + elapsedTime + " Time Remaining " + (120 - elapsedTime.seconds()));
            }
            telemetry.update();
        }
    }


        public Path newPath(double targetX, double targetY, double targetH){
            Point startPoint = new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN);
            Point endPoint = new Point(targetX, targetY, Point.CARTESIAN);
            Path path = new Path(new BezierLine(startPoint, endPoint));
            path.setLinearHeadingInterpolation(Math.toRadians(follower.getPose().getHeading()), Math.toRadians(targetH));
            return path;
            // path outline

        }
        public void alignHeading(){
        boolean isAligned = false;;
        double heading, targetHeading = 0;
            while (!isAligned){
                heading = Math.toDegrees(follower.getPose().getHeading());

                if (heading <= 138  && heading > 45){
                    isAligned = heading > 88 && heading < 90;
                    targetHeading = 90;
                } else if (heading > 135 && heading <= 215){
                    isAligned = heading > 178 && heading < 180;
                    targetHeading = 180;
                } else if (heading > 215 && heading <= 315){
                    isAligned = heading > 268 && heading < 270;
                    targetHeading = 270;
                } else if (heading > 315  || heading <=45){
                    isAligned = heading < 2 && heading > 0;
                    targetHeading = 360;
                }
                follower.update();
//                isAligned = heading < 2 && heading > 0;
                if (heading < targetHeading && targetHeading != 0){
                    robot.setPower(0.3 , 0.3, -0.3, -0.3);
                } else if (heading > targetHeading && targetHeading != 0)  {
                    robot.setPower(-0.3, -0.3, 0.3, 0.3);
                } else if (heading > 180 && targetHeading == 0){
                    robot.setPower(0.3 , 0.3, -0.3, -0.3);
                } else if (heading < 180 && targetHeading == 0){
                    robot.setPower(-0.3, -0.3, 0.3, 0.3);
                }
            }
            // if robot is at angle greater than 180, quickest way to zero is turning counterclockwise, otherwise going clockwise is quickest way

            /* welcome to carter pseudocode:
            if (45 < heading <= 135) {
                set heading to 90
                } else if (135 < heading <= 215) {
                set heading to 180
                } else if (215 < heading <= 315) {
                set heading to 270
                } else if (315 < heading || heading <= 45) {
                set heading to 0
             */

        }
        public void updateArm(){

        }
//    public void pickUpRobot(){
//        robot.armVertical.setTargetPosition(0);
//        robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.armVertical.setPower(1);
//    }
}