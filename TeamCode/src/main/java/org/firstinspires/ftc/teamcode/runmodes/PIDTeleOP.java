package org.firstinspires.ftc.teamcode.runmodes;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;


@TeleOp (name = "PID Debugging")
public class PIDTeleOP extends OpMode {
    PIDFController extensionPID;
    PIDFController verticalPID;
    double output, armOutput;
    Hardware robot = Hardware.getInstance();
    public void init(){
        extensionPID = new PIDFController(0.02, 0, 0.0001, 0);
        verticalPID = new PIDFController(0.0219, 0, 0.0001, 0);
        robot.init(hardwareMap);
    }
    public void loop(){
            output = extensionPID.calculate(robot.armExtension.getCurrentPosition(), -1200);

            robot.armExtension.setPower(output);
            output = verticalPID.calculate(robot.armVertical.getCurrentPosition(), 2400);
            robot.armVertical.setPower(armOutput);

    }
}
