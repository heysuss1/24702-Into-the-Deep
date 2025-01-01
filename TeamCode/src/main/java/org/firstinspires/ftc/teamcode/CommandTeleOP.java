package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DualServoClaw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;


@TeleOp(name = "Command TeleOP")
public class CommandTeleOP extends NextFTCOpMode {
    Hardware robot = Hardware.getInstance();

    public CommandTeleOP() {
        super(Wrist.INSTANCE, Arm.INSTANCE, DualServoClaw.INSTANCE);
    }

    Commands commands = new Commands();
    Wrist wrist = new Wrist();

    public void onUpdate() {
        robot.init(hardwareMap);
        waitForStart();

        Gamepad currentGamePad1 = new Gamepad();
        Gamepad currentGamePad2 = new Gamepad();

        Gamepad previousGamePad1 = new Gamepad();
        Gamepad previousGamePad2 = new Gamepad();
        int counter = 0;
        while (opModeIsActive()) {
            previousGamePad1.copy(currentGamePad1);
            previousGamePad2.copy(currentGamePad2);

            currentGamePad1.copy(gamepad1);
            currentGamePad2.copy(gamepad2);

            if (gamepad2.y) Wrist.INSTANCE.up().invoke();
            if (currentGamePad2.a && !previousGamePad2.a) DualServoClaw.INSTANCE.open().start();
            if (currentGamePad2.b && !previousGamePad2.b) Wrist.INSTANCE.straight();

            if (currentGamePad2.x && !previousGamePad2.x) {
                counter++;
                commands.straightArm().invoke();
            }

            telemetry.addData("Counter", counter);
            telemetry.update();
            if (currentGamePad2.dpad_up && !previousGamePad2.dpad_up) {

            }

        }
    }

    public void onStartButtonPressed() {
        Wrist.INSTANCE.up().start();
    }
}