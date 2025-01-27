package org.firstinspires.ftc.teamcode.runmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Hardware;


@Config
@TeleOp (name = "Diddy Claw Test")
public class DiddyTime extends OpMode {

    public static double pitch, roll;
    public static double clawPosition;
    Gamepad currentGamepad1;
    Gamepad currentGamepad2;

    Gamepad previousGamepad1;
    Gamepad previousGamepad2;

    Hardware robot = Hardware.getInstance();
    public void init() {
        robot.init(hardwareMap);
//        robot.diffy1.setPosition(0);
//        robot.diffy2.setPosition(1);
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();


    }

    public void loop(){

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
//            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
                robot.diddylate(pitch, roll);

//            }0o
        }
    }
