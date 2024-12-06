package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Auto {
    Hardware robot = Hardware.getInstance();
    Timer timer;
    Pose starting;

    public void closeClaw(){
        robot.leftServo.setPosition(.639);
        robot.rightServo.setPosition(.1);
    }
    public void openClaw(){
        robot.leftServo.setPosition(0.508);
        robot.rightServo.setPosition(0.24);
    }
    public void armExtend(int ticks){
        robot.armExtension.setPower(1);
        robot.armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armExtension.setTargetPosition(ticks);
    }
    public void armUp(int ticks){
        robot.armVertical.setPower(1);
        robot.armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armVertical.setTargetPosition(ticks);
    }

    public void rotateArmBackWards(){
        robot.rotateServo.setPosition(.174);
    }
    public void rotateArmForwards(){
        robot.rotateServo.setPosition(.726);
    }
}
