package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous (name = "Specimens Auto")
public class SpecimenAuto extends OpMode {
    Follower follower;

    Timer armTimer, pathTimer;
    Hardware robot = Hardware.getInstance();

    enum ActionState{
        HANG_SPECIMEN
    }
    enum PathState{
        GO_TO_SUBMERSIBLE,
        STRAFE_TO_SAMPLE1,
        BEHIND_SAMPLE1,
        PUSH_SAMPLE1,
        BACKWARDS_FROM_SAMPLE1,
        STRAFE_BEHIND_SAMPLE2,
        PUSH_SAMPLE2,
        GO_BACKWARDS,
        COLLECT_SPECIMEN,
        GO_FORWARDS
    }
    PathState pathState = PathState.GO_TO_SUBMERSIBLE;
    ActionState actionState = ActionState.HANG_SPECIMEN;
    Pose starting = new Pose(5, 64, 0);
    double lastX = starting.getX();
    double lastY = starting.getY();
    double lastH = starting.getHeading();

    Path toSubmersible, strafeToSample1, behindSample1, pushSample1, backwardsFromSample1, strafeBehindSample2, pushSample2, goBackWards, goForwards;
    public void init(){
        follower = new Follower(hardwareMap);
        robot.init(hardwareMap);
        follower.setStartingPose(starting);
        armTimer = new Timer();
        pathTimer = new Timer();
        buildPaths();
    }


    public void buildPaths(){
        toSubmersible = newPath(31.6, 64, 0);
        strafeToSample1 = newPath(31.6, 40, lastH);
        behindSample1 = newPath(65, 26, lastH);
        pushSample1 = newPath(13,26, lastH);
        backwardsFromSample1 = newPath(70,24, lastH);
        strafeBehindSample2 = newPath(70 , 14, lastH);
        pushSample2 = newPath(12, 14, lastH);
        goBackWards = newPath(30, 28, -179);
        goForwards = newPath(5, 28, -179);



    }
//    public PathChain pushSpecimens(){
//
//        PathBuilder builder = new PathBuilder();
//        builder.addPath(huhuh);
//    }
    public Path newPath(double targetX, double targetY, double targetH){
            Point startPoint = new Point(lastX, lastY, Point.CARTESIAN);
            Point endPoint = new Point(targetX, targetY, Point.CARTESIAN);
            Path path = new Path(new BezierLine(startPoint, endPoint));
            path.setLinearHeadingInterpolation(Math.toRadians(lastH), Math.toRadians(targetH));
            lastX = targetX;
            lastY = targetY;
            lastH = targetH;
            return path;

        }


    public void setPathState(PathState state){
        pathState = state;
        pathTimer.resetTimer();
    }
    public void autonomousPathUpdate(){
        switch (pathState){
            case GO_TO_SUBMERSIBLE:
                follower.followPath(toSubmersible);
                setPathState(PathState.STRAFE_TO_SAMPLE1);
                break;
            case STRAFE_TO_SAMPLE1:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(strafeToSample1);
                    setPathState(PathState.BEHIND_SAMPLE1);
                }
                break;
            case BEHIND_SAMPLE1:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(behindSample1);
                    setPathState(PathState.PUSH_SAMPLE1);
                }
            case PUSH_SAMPLE1:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(pushSample1);
                    setPathState(PathState.BACKWARDS_FROM_SAMPLE1);
                }
                break;
            case BACKWARDS_FROM_SAMPLE1:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(backwardsFromSample1);
                    setPathState(PathState.STRAFE_BEHIND_SAMPLE2);
                }
                break;
            case STRAFE_BEHIND_SAMPLE2:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(strafeBehindSample2);
                    setPathState(PathState.PUSH_SAMPLE2);
                }
                break;

//            case TURN:
//                if (follower.getCurrentPath().isAtParametricEnd()){
//                    follower.followPath(turnRo  bot);
//                    setPathState(PathState.PUSH_SAMPLE2);
//                }
            case PUSH_SAMPLE2:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(pushSample2);
                    setPathState(PathState.GO_BACKWARDS);
                }
                break;
            case GO_BACKWARDS:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(goBackWards);
                    setPathState(PathState.GO_FORWARDS);
                }
            case GO_FORWARDS:
                if (follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(goForwards);
                    setPathState(PathState.COLLECT_SPECIMEN);
                }
            default:
                stop();

        }
    }
    public void setActionState(ActionState state){
        actionState = state;
        armTimer.resetTimer();
    }
    public void autonomousActionUpdate(){
        switch(actionState){
            case HANG_SPECIMEN:
                  break;
            default:
                stop();
        }
    }
    public void loop() {

        telemetry.addData("Position",follower.getPose());
        telemetry.addData("Current Path: ", pathState);
        telemetry.addData("Timer: ", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
        autonomousPathUpdate();
        follower.update();

    }
}
