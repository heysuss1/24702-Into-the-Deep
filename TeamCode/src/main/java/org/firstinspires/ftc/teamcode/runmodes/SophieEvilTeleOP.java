package org.firstinspires.ftc.teamcode.runmodes;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "this one or i crash out pmo")
public class SophieEvilTeleOP extends LinearOpMode {
    Hardware robot = Hardware.getInstance();
    PIDFController extensionPID;
    PIDFController verticalPID;

    ElapsedTime clawTimer;

    ElapsedTime elapsedTime;
    boolean clawIsOpen = false;
    int armVerticalTarget;
    int armExtensionTarget;
    boolean clawJustMoved = false;

    int currentClawPosition = 0;
    int[] clawPositions = {45, 20, 0, -15, -45};

    int currentExtension = 0;
    int currentRotation = 0;

    int addExtension = 0;

    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        extensionPID = new PIDFController(0.02, 0, 0.001, 0);
        verticalPID = new PIDFController(0.0219, 0, 0.001, 0);

        robot.setSpeed(1);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        elapsedTime = new ElapsedTime();
        clawTimer = new ElapsedTime();

        robot.armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.armVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            drive();
            controlClaw();
            if (!(gamepad1.start && gamepad1.back)) {
                controlSlides();
            } else {
                robot.armVertical.setPower(1);
            }
            updateTelemetry();
            if (gamepad1.right_trigger > 0.1f) {
                addExtension-=20;
            }
//            if (clawTimer.milliseconds() > 100) {
//                clawJustMoved = false;
//                clawTimer.reset();
//            }
            extendSlides(currentExtension + addExtension);
            rotateSlides(currentRotation);
            robot.diddylate(robot.pitch, robot.roll);
        }
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        robot.rf.setZeroPowerBehavior(behavior);
        robot.lf.setZeroPowerBehavior(behavior);
        robot.rb.setZeroPowerBehavior(behavior);
        robot.lb.setZeroPowerBehavior(behavior);
    }

    private void drive() {
        double forward = -gamepad1.left_stick_y;
        double sideways = gamepad1.left_stick_x;
        double turning = gamepad1.right_stick_x;

        double max = Math.max(Math.abs(forward - sideways - turning),
                Math.max(Math.abs(forward + sideways - turning),
                        Math.max(Math.abs(forward + sideways + turning),
                                Math.abs(forward + turning - sideways))));

        double scaleFactor = max > 1 ? 1 / max : 1;
        robot.setPower((forward - sideways - turning) * scaleFactor,
                (forward + sideways - turning) * scaleFactor,
                (forward + sideways + turning) * scaleFactor,
                (forward + turning - sideways) * scaleFactor);

        if (gamepad1.left_stick_button) {
            robot.setSpeed(0.35);
        } else {
            robot.setSpeed(1);
        }
    }

    private void controlClaw() {
        if (gamepad1.left_trigger > 0.1f && !clawJustMoved) {
            clawIsOpen = !clawIsOpen;
            robot.claw.setPosition(clawIsOpen ? 0.4 : 0.15);
//            clawJustMoved = true;
//            clawTimer.reset();
        }
        if (gamepad1.right_bumper) {
            if (currentClawPosition > 0) {
                currentClawPosition -= 1;
            }
            robot.roll = clawPositions[currentClawPosition];
        }
        if (gamepad1.left_bumper) {
            if (currentClawPosition < clawPositions.length - 1) {
                currentClawPosition += 1;
            }
            robot.roll = clawPositions[currentClawPosition];
        }
    }

    private void controlSlides() {
        if (gamepad1.y) {
            currentExtension = 0;
            currentRotation = 3400;
            robot.pitch = 90;
            robot.roll = 0;
            addExtension = 0;
        }
        if (gamepad1.b) {
            robot.pitch = 90;
            robot.roll = 0;
        }

        if (gamepad1.a) {
            currentExtension = 200;
            currentRotation = 150;
            robot.pitch = 90;
            robot.roll = 0;
            addExtension = 0;
        }
        if (gamepad1.x) {
            currentRotation = 1300; // fully vertical
            currentExtension = 500;
            robot.pitch = 90;
            robot.roll = 00;
            addExtension = 0;
        }
    }

    private void extendSlides(int targetPosition) {
        robot.armExtension.setTargetPosition(targetPosition);
        robot.armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armExtension.setPower(1);
    }

    private void rotateSlides(int targetPosition) {
        robot.armVertical.setTargetPosition(targetPosition);
        robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armVertical.setPower(1);
    }

    private void updateTelemetry() {
        telemetry.addData("Arm Vertical", robot.armVertical.getCurrentPosition());
        telemetry.addData("Arm Extension", robot.armExtension.getCurrentPosition());
        telemetry.addData("Claw Open", clawIsOpen);
        telemetry.addData("Time Remaining", (120 - elapsedTime.seconds()));
        telemetry.update();
    }
}
