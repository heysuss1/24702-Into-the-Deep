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
@TeleOp(name = "Demon TeleOP")
public class TeleOP extends LinearOpMode {
    Hardware robot = Hardware.getInstance();
    Follower follower;

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
        waitForStart();
        boolean clawIsOpen = false;
        boolean pressingB = false;
        boolean pressingLT = false;
        boolean hasSensed = false;
        boolean pressingY = false;
        boolean pressingA = false;
        boolean pressingX = false;
        boolean pressingLBumper = false;
        boolean clawForwards = false;
        boolean pressingRB = false;
        boolean pressingX2 = false;
        boolean goToPosition = false;
        boolean extensionLimiterPressed = false;
        boolean tooFar = false;
        boolean isStalling = false;
        double ticks = 0;
        while (opModeIsActive()){
            //gamepad1 = Driver 1
            double movement = -gamepad1.left_stick_y;
            double strafing = gamepad1.left_stick_x;
            double turning = gamepad1.right_stick_x;

            double rf = movement - strafing - turning;
            double rb = movement + strafing - turning;
            double lf = movement + strafing + turning;
            double lb = movement - strafing + turning;
            ticks = -(robot.armExtension.getCurrentPosition());
            double max = Math.max(Math.abs(rf), Math.max(Math.abs(rb), Math.max(Math.abs(lf), Math.abs(lb))));
            if (max < robot.maxSpeed) {
                robot.setPower(rf, rb, lf, lb);
            } else {
                double scaleFactor = max / robot.maxSpeed;
                robot.setPower((rf) * scaleFactor,
                        (rb) * scaleFactor,
                        (lf) * scaleFactor,
                        (lb) * scaleFactor);
            }
            //only runs if the game button is he  ld down
            //gamepad 2 = driver 2
            if (gamepad2.y && !pressingY){
                robot.rotateServo.setPosition(.153);

                pressingY = true;
            } else if (!gamepad2.y){
                pressingY = false;
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
            } else{
                pressingA = false;
            }


            if (gamepad2.dpad_left){

            }
            if (gamepad1.x && !pressingX){
                toSubmersible = newPath(bucket.getX(), bucket.getY(), 45);
                follower.followPath(toSubmersible);
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
                follower.followPath(toObservationZone);
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

            if(gamepad2.right_stick_y > 0.1) {
                robot.armVertical.setPower(-1);
            }
            else if(gamepad2.right_stick_y < -0.1){
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
                    robot.leftServo.setPosition(.639);
                    robot.rightServo.setPosition(0.55);//may be wrong position
                    clawIsOpen = true;
                } else {
                    //Close claw
                    robot.leftServo.setPosition(.508);
                    robot.rightServo.setPosition(.69);
                    clawIsOpen = false;
                }
                pressingLT = true;
            }
            else if(!(gamepad2.left_trigger >0.1)){
                pressingLT = false;
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
//    public void pickUpRobot(){
//        robot.armVertical.setTargetPosition(0);
//        robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.armVertical.setPower(1);
//    }
}