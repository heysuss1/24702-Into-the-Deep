package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;


//close value right = .154, open = .298, close left = .684, open  = .538

//VERY BAD NUMBERS (heading):
//P: 0.002
//D: 0.00055

//TEMPORARY D AND P:
//D: 0.0005
//P:0.08
@TeleOp(name = "Demon TeleOP")
public class TeleOP extends LinearOpMode {
    Hardware robot = Hardware.getInstance();
    Follower follower;

    double kP, kI, kD, kF;
    SquidPID squid = new SquidPID(kP, kI, kD, kF);
    //PHUHS
    public void runOpMode(){
        int position = 0;
        robot.init(hardwareMap);
        telemetry.addData("Status", "Hello, Drivers!");
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(63, 95));
        telemetry.update();
        Pose bucket = new Pose(18, 127, Point.CARTESIAN);
        Pose frontOfSubmersible = new Pose(31.6, 78, Point.CARTESIAN);
        Pose frontOfObservationZone = new Pose(30, 17, Point.CARTESIAN);
        Path toSubmersible, toObservationZone, toBuckets;
        double forward, sideways, turning, max;
        double scaleFactor = 0.8;
        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();
        boolean clawIsOpen = false;
        boolean pressingB = false;
        boolean pressingLT = false;
        boolean pressingRT = false;
        boolean hasSensed = false;
        boolean pressingY = false;
        boolean pressingA = false;
        boolean pressingX = false;
        boolean pressingLBumper = false;
        boolean clawForwards = false;
        boolean pressingRB = false;
        boolean pressingX2 = false;
        boolean goToPosition = false;
        boolean pressingRB2 = false;
        boolean pressingG1Y = false; // gamepad1's Y position
        boolean armVerticalTooFar = false;
        boolean extensionLimiterPressed = false;
        boolean tooFar = false;
        boolean isStalling = false;
        double ticks = robot.armExtension.getCurrentPosition();
        while (opModeIsActive()){
            //gamepad1 = Driver 1
            if (gamepad1.dpad_left){
               forward = -0.4;
            }
            if (gamepad1.dpad_right){
                forward = 0.4;
            }

            forward = -(Math.atan(5 * gamepad1.left_stick_y) / Math.atan(5));
            sideways = (Math.atan(5 * gamepad1.left_stick_x) / Math.atan(5));
            turning = (Math.atan(5 * gamepad1.right_stick_x) / Math.atan(5)) * 0.5;
            max = Math.max(Math.abs(forward - sideways - turning), Math.max(Math.abs(forward + sideways - turning), Math.max(Math.abs(forward + sideways + turning), Math.abs(forward + turning - sideways))));
            if (max > robot.maxSpeed) {
                scaleFactor = robot.maxSpeed / max;
            } else {
                scaleFactor = robot.maxSpeed;
            }
            scaleFactor *= Math.max(Math.abs(1 - gamepad1.right_trigger), 0.2);
            robot.setPower((forward - sideways - turning)*scaleFactor, (forward + sideways - turning) * scaleFactor, (forward + sideways + turning) * scaleFactor, (forward + turning - sideways) * scaleFactor);
            //only runs if the game button is he  ld down
            //gamepad 2 = driver 2
            if (gamepad2.y && !pressingY){
                robot.rotateServo.setPosition(.153);

                pressingY = true;
            } else if (!gamepad2.y){
                pressingY = false;
            }
            if (gamepad1.y && !pressingG1Y){
                alignHeading();
                pressingG1Y = true;
            } else if (!gamepad1.y){
                pressingG1Y = false;
            }
            if (gamepad2.b && !pressingB && !gamepad2.start){
                robot.rotateServo.setPosition(.425);
                pressingB = true;
            } else if (!gamepad2.b){
                pressingB = false;
            }
            if (gamepad2.a && !pressingA){
                robot.rotateServo.setPosition(0.741);
//                robot.rotateServo.setPosition(0.437);

                pressingA = true;
            } else {
                pressingA = false;
            }


            if (gamepad2.dpad_left){
            }

            if (robot.armVertical.getCurrentPosition() > 5000){
                armVerticalTooFar = true;
            } else{
                armVerticalTooFar = false;
            }
            if (gamepad1.x && !pressingX){
                toSubmersible = newPath(bucket.getX(), bucket.getY(), 45);
//                follower.followPath(toSubmersible);
                pressingX = true;
            } else if (!gamepad1.x){
                pressingX = false;
            }

            if (gamepad2.x && !pressingX2){
                robot.armVertical.setTargetPosition(0);
                robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.rotateServo.setPosition(0.437);
                goToPosition = true;

                pressingX2 = true;
            } else if (!gamepad2.x){
                pressingX2 = false;
            }
            if (gamepad2.right_bumper && !pressingRB2){
                robot.armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pressingRB2 = true;
            } else if (!gamepad2.right_bumper){
                pressingRB2 = false;
            }

            if (goToPosition){
                robot.armVertical.setPower(1);
                if (robot.armVertical.getCurrentPosition() > -5 && robot.armVertical.getCurrentPosition() < 5){
                    goToPosition = false;
                }
            } else{
                robot.armVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad1.right_bumper && !pressingRB){
                toObservationZone = newPath(frontOfObservationZone.getX(), frontOfSubmersible.getY(), 0);
//                follower.followPath(toObservationZone);
                pressingRB = true;
            } else if (!gamepad1.right_bumper){
                pressingRB = false;
            }
            if (gamepad1.right_trigger > 0.1){
                robot.setSpeed(1-0.9 * gamepad1.right_trigger);
            } else {
                robot.setSpeed(1);
            }
            if(gamepad2.left_stick_y < -0.1 && (!tooFar)) {
                telemetry.addData("Status", "This is going, should be going forward");
                robot.armExtension.setPower(-1);
            } else if(gamepad2.left_stick_y > 0.1){
                telemetry.addData("Status", "This is going, should be going backwards");
                robot.armExtension.setPower(1);
            } else {
                robot.armExtension.setPower(0);
//                if (ticks < 2850) {
//                    robot.armExtension.setTargetPosition(2700);
//                } else if (ticks > 100) {
//                    robot.armExtension.setTarge'tPosition(500);
//                } else {
//                    robot.armExtension.setPower(0);
//                }
            }


            if(ticks < 2700){
                tooFar = false;
            } else{
                tooFar = true;
            }

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

            if(gamepad2.right_stick_y > 0.1  ) {
                robot.armVertical.setPower(-1);
            }
            else if(gamepad2.right_stick_y < -0.1 && !armVerticalTooFar){
                robot.armVertical.setPower(1);
            } else{
                if (!goToPosition){
                    robot.armVertical.setPower(0);
                }
            }
            /*if(gamepad2.b && !pressingB){
                telemetry.addData("Extention Position", robot.armExtension.getCurrentPosition());
                telemetry.addData("Horizontal Position", robot.armVertical.getCurrentPosition());
                telemetry.update();
                pressingB = true;
            } else if (!gamepad2.b){
                pressingB = false;
            }*/
            //only for using trigger as a button
            if ((gamepad2.left_trigger > 0.1)&& !pressingLT){
                if(!clawIsOpen){
                    //Open claw
                    //robot.leftServo.setPosition(0.64);
                    //robot.rightServo.setPosition(0.55);//may be wrong position
                    robot.openClaw(1);
                    clawIsOpen = true;
                } else {
                    //Close claw
                    //robot.leftServo.setPosition(0.49);
                    //robot.rightServo.setPosition(0.71);
                    robot.openClaw(0);
                    clawIsOpen = false;
                }
                pressingLT = true;
            }
            else if(!(gamepad2.left_trigger >0.1)){
                pressingLT = false;
            }

            //right trigger = half-closed
            if ((gamepad2.right_trigger > 0.1) && !pressingRT) {
                robot.openClaw(0.5);
                clawIsOpen = false;
                pressingRT = true;
            } else if (!(gamepad2.right_trigger > 0.1)) {
                pressingRT = false;
            }

            if (gamepad2.left_bumper && !pressingLBumper){
                if (!clawForwards){
                    robot.rotateServo.setPosition(.726);
                    clawForwards = true;
                } else{
                    robot.rotateServo.setPosition(.174);
                    clawForwards = false;
                }
                pressingLBumper = true;

            }
            else  if (!gamepad2.left_bumper){
                pressingLBumper = false;
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
//                    robot.armExtension.setPower(1);
//                }
//            }

            follower.update();
            telemetry.addData("Position", ticks);
            telemetry.addData("Arm Vertical", robot.armVertical.getCurrentPosition());
            telemetry.addData("Arm Horizontal Position", ticks);
            telemetry.addData("Speed", robot.getSpeed());
            telemetry.addData("Extension Voltabge", robot.armExtension.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Hello", position);
            telemetry.addData("Forwards", clawForwards);
            telemetry.addData("Robot Position", follower.getPose());
//            telemetry.addData("Color Sensor Distance", robot.colorSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Color Sensor Status", robot.colorSensor.getClass());
            telemetry.addData("Conttrol Toggle", goToPosition);
            telemetry.addData("Current heading is", Math.toDegrees(follower.getPose().getHeading()));
            //            telemetry.addData("Arm Vertical Position", robot.armVertical.getCurrentPosition());
            telemetry.update();
        }
    }

        public Path newPath(double targetX, double targetY, double targetH){
            Point startPoint = new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN);
            Point endPoint = new Point(targetX, targetY, Point.CARTESIAN);
            Path path = new Path(new BezierLine(startPoint, endPoint));
            path.setLinearHeadingInterpolation(Math.toRadians(follower.getPose().getHeading()), Math.toRadians(targetH));
            return path;

        }
        public void alignHeading(){

            while (Math.toDegrees(follower.getPose().getHeading()) > 0 && !(Math.toDegrees(follower.getPose().getHeading()) < 2)){
                robot.setPower(-0.8, -0.8, 0.8, 0.8);
                follower.update();

            }
            while (Math.toDegrees(follower.getPose().getHeading()) < 0 && !(Math.toDegrees(follower.getPose().getHeading()) > -2)){
                robot.setPower(0.8, 0.8, -0.8, -0.8);
                follower.update();
            }
        }
//    public void pickUpRobot(){
//        robot.armVertical.setTargetPosition(0);
//        robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.armVertical.setPower(1);
//    }
}