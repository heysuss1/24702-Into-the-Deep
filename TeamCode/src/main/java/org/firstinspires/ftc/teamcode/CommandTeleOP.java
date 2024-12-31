package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DualServoClaw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;


@TeleOp (name = "Command TeleOP")
public class CommandTeleOP extends LinearOpMode {
    DualServoClaw claw = new DualServoClaw();
    Arm arm = new Arm();
    Commands commands = new Commands();
    Wrist wrist = new Wrist();
    public void runOpMode(){
        claw.initialize();
        arm.initialize();
        wrist.initialize();
        waitForStart();

        Gamepad currentGamePad1 = new Gamepad();
        Gamepad currentGamePad2 = new Gamepad();

        Gamepad previousGamePad1 = new Gamepad();
        Gamepad previousGamePad2 = new Gamepad();
        while (opModeIsActive()){
            currentGamePad1.copy(gamepad1);
            currentGamePad2.copy(gamepad2);
            previousGamePad1.copy(currentGamePad1);
            previousGamePad2.copy(currentGamePad2);

            if (currentGamePad1.y && !previousGamePad2.y) wrist.INSTANCE.up();
            if (currentGamePad2.a && !previousGamePad2.x) wrist.INSTANCE.down();
            if (currentGamePad1.b && !previousGamePad2.y) wrist.INSTANCE.straight();

            if (currentGamePad2.x && !previousGamePad2.x) {
                commands.straightArm().invoke();
            }

            if (currentGamePad2.dpad_up && !previousGamePad2.dpad_up){

            }


        }
    }
}
