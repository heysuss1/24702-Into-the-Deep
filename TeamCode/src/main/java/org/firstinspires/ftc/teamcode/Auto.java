package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Auto {
    Hardware robot = Hardware.getInstance();
    Timer armTimer, pathTimer;
    Follower follower;
    Pose starting;
    Pose bucket = new Pose(15, 127, Point.CARTESIAN);
    Pose frontOfSubmersible = new Pose(34, 71, Point.CARTESIAN);
    Pose frontOfObservationZone = new Pose(9, 17, Point.CARTESIAN);

    double lastX = starting.getX();
    double lastY = starting.getY();
    double lastH = starting.getHeading();


public void init (double setX, double setY, double setH) {
    armTimer = new Timer();
    pathTimer = new Timer();
    follower = new Follower(hardwareMap);
    robot.init(hardwareMap);
    lastH = follower.getPose().getHeading();
    follower. setStartingPose(starting);
    starting = new Pose(setX, setY,  setH);

    }
    public void buildPath(){

    }

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

    public Path newPath(double targetX, double targetY, double targetH) {
        Point startPoint = new Point(lastX, lastY, Point.CARTESIAN);
        Point endPoint = new Point(targetX, targetY, Point.CARTESIAN);
        Path path = new Path(new BezierLine(startPoint, endPoint));
        path.setLinearHeadingInterpolation(Math.toRadians(lastH), Math.toRadians(targetH));
        lastX = targetX;
        lastY = targetY;
        lastH = targetH;
    return path;
    }
}
