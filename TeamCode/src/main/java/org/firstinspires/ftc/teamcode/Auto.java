package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Auto {
    Hardware robot = Hardware.getInstance();
    Pose starting;
    Timer timer;

    public void init(){

    }
    public void setStartPosition(double setY, double setX, double setH){
        double lastX = starting.getX();
        double lastY = starting.getY();
        double lastH = starting.getHeading();
        Pose starting = new Pose(setX, setY, setH);
    }
    public void closeClaw(){
        robot.leftServo.setPosition(.508);
        robot.rightServo.setPosition(.69);
    }
    public void openClaw(){
        robot.leftServo.setPosition(.639);
        robot.rightServo.setPosition(0.55);
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
        robot.rotateServo.setPosition(.153);
    }
    public void rotateArmForwards(){
        robot.rotateServo.setPosition(.741);
    }

}



