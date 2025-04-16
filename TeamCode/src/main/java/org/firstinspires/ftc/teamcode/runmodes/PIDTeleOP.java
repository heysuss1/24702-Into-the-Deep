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
    int extensionTarget;
    int verticalTarget;
    int armTarget;
    boolean usePID;
    int extensionError = 0;
    int verticalError = 0;
    Hardware robot = Hardware.getInstance();
    public void init(){
        extensionPID = new PIDFController(0.02, 0, 0.001, 0);

        extensionPID.setPIDF(0.02, 0, 0.001, 0);
        controller = new PIDFController(0.02, 0, 0.001, 0);
        controller.setPIDF(0.02, 0, 0.001, 0);
        usePID = false;
        robot.init(hardwareMap);
    }
    public void loop(){
            if (gamepad2.dpad_left){
                extensionTarget = -1200;
                armTarget = 1436;
                usePID = true;

            }
            if (gamepad2.dpad_right){
                usePID = true;
            }
            if (gamepad2.left_stick_y > 0.1){
                robot.armExtension.setPower(1);
                usePID = false;
            } else if (gamepad2.left_stick_y < -0.1){
                robot.armExtension.setPower(-1);
            } else{
                if (!usePID){
                    robot.armExtension.setPower(0);
                }
            }
        if (gamepad2.right_stick_y > 0.1){
            robot.armVertical.setPower(-1);
            usePID = false;
        } else if (gamepad2.right_stick_y < -0.1){
            robot.armVertical.setPower(1);
        } else{
            if (!usePID){
                robot.armVertical.setPower(0);
            }
        }
        if (usePID){
                extensionError = extensionTarget-robot.armExtension.getCurrentPosition();
                verticalError = verticalTarget-robot.armVertical.getCurrentPosition();
                output = extensionPID.calculate(0, extensionError);
                robot.armExtension.setPower(output);
                robot.armVertical.setPower(controller.calculate(0, verticalError));
            }

            telemetry.addData("Current pos: ",robot.armExtension.getCurrentPosition());
            telemetry.addData("target", extensionTarget);
            telemetry.addData("output", output);

    }
}
