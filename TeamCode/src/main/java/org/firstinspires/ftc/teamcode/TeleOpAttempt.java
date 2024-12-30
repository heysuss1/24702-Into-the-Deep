package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@TeleOp (name = "TeleOpAttempt")

public class TeleOpAttempt extends LinearOpMode {
    Hardware robot = Hardware.getInstance();
    Follower follower;

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

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        waitForStart();
        double ticks = 0;
        boolean goToPosition = false;// False?
        boolean tooFar;
        boolean clawForwards = false;
        boolean clawIsOpen = false;

        while (opModeIsActive()){
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
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);//?

            if(ticks < 2700){
                tooFar = false;
            } else{
                tooFar = true;
            }

            if (currentGamepad2.y && !previousGamepad2.y) {
                robot.rotateServo.setPosition(0.174);


            }
            if (currentGamepad2.b && !previousGamepad2.y && !currentGamepad2.start) {
                robot.rotateServo.setPosition(0.437);
            }
            if (currentGamepad2.a && !previousGamepad2.a) {
                robot.rotateServo.setPosition(0.726);
            }
            if (currentGamepad1.x && !previousGamepad1.x) {
                toSubmersible = newPath(bucket.getX(), bucket.getY(), 45);
            }
            if (currentGamepad2.x && !previousGamepad2.x) {
                robot.armVertical.setTargetPosition(0);
                robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                goToPosition = true;
            }
            if (goToPosition){
                robot.armVertical.setPower(1);
                if (robot.armVertical.getCurrentPosition() > -5 && robot.armVertical.getCurrentPosition() < 5){
                    goToPosition = false;
                }
            } else{
                robot.armVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                toObservationZone = newPath(frontOfSubmersible.getX(), frontOfObservationZone.getY(), 0);
                follower.followPath(toObservationZone);
            }
            if (gamepad1.right_trigger > 0.1){
                robot.setSpeed(1-0.9 * gamepad1.right_trigger);
            } else {
                robot.setSpeed(1);
            }
            //start checking here
            if (currentGamepad2.left_stick_y < -0.1 && (!tooFar)){
                telemetry.addData("Status", "Forward");
                robot.armExtension.setPower(-1);
            } else if(currentGamepad2.left_stick_y > 0.1){
                telemetry.addData("Status", "Backward");
                robot.armExtension.setPower(1);
            } else {
                robot.armExtension.setPower(0);
            }


            if(currentGamepad2.right_stick_y > 0.1) {
                robot.armVertical.setPower(-1);
            }
            else if(currentGamepad2.right_stick_y < -0.1){
                robot.armVertical.setPower(1);
            } else{
                if (!goToPosition){
                    robot.armVertical.setPower(0);
                }
            }
            if((currentGamepad2.left_trigger > 0.1)&& !(previousGamepad2.left_trigger > 0.1)){
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

            }
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                if (!clawForwards) {
                    robot.rotateServo.setPosition(.726);
                    clawForwards = true;
                } else {
                    robot.rotateServo.setPosition(.174);
                    clawForwards = false;
                }
            }
            follower.update();
            telemetry.addData("Position", ticks);
            telemetry.addData("Arm Vertical", robot.armVertical.getCurrentPosition());
            telemetry.addData("Arm Horizontal Position", ticks);
            telemetry.addData("Speed", robot.getSpeed());
            telemetry.addData("Extension Voltabge", robot.armExtension.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Hello", position);
//            telemetry.addData("Forwards", clawForwards);
            telemetry.addData("Too far?", tooFar);
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
}
