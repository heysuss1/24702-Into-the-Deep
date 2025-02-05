package org.firstinspires.ftc.teamcode.runmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous (name = "Aerm Up Auto")
public class ArmUpAuto extends LinearOpMode {
    Hardware robot = Hardware.getInstance();

    public void runOpMode() {

        telemetry.addData("WSG DRIVERS!!!", "THE ARM IS MOVING SLIGHTLY UP!!! HOORAY!!!");
        robot.init(hardwareMap);
        waitForStart();

        robot.armVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armVertical.setTargetPosition(570);
        robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armVertical.setPower(0.8);
//        robot.rotateServo.setPosition(.726);
        while (robot.armVertical.isBusy() && opModeIsActive()){

        }
    }

    //please loooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooop();
    //sphinx of black quartz, judge my vow.
}
