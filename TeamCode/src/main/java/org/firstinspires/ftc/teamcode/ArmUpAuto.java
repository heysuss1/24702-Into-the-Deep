package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmUpAuto extends LinearOpMode {
    Hardware robot = Hardware.getInstance();

    public void runOpMode() {

        telemetry.addData("WSG DRIVERS!!!", "THE ARM IS MOVING SLIGHTLY UP!!! HOORAY!!!");
        robot.init(hardwareMap);
        waitForStart();

        robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armVertical.setTargetPosition(1200);
        robot.armVertical.setPower(0.8);
        robot.rotateServo.setPosition(.726);
    }

    //please loooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooop();
    //sphinx of black quartz, judge my vow.
}