package org.firstinspires.ftc.teamcode.runmodes;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;


@TeleOp (name = "PID Debugging")
public class PIDTeleOP extends OpMode {
    PIDFController extensionPID;
    PIDFController controller;
    double output, armOutput;
    Hardware robot = Hardware.getInstance();
    public void init(){
        controller = new PIDFController(0.02, 0, 0.0001, 0);

        robot.init(hardwareMap);
    }
    public void loop(){
            output = controller.calculate(robot.armExtension.getCurrentPosition(), -1200);

            robot.armExtension.setPower(output);
            armOutput = controller.calculate(robot.armVertical.getCurrentPosition(), 2400);
            robot.armVertical.setPower(armOutput);
            telemetry.addData("Current pos: ",robot.armExtension.getCurrentPosition());
            telemetry.addData("target", 2400);
            telemetry.addData("output", output);

    }
}
