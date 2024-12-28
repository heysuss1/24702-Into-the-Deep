package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "From Scratch")
public class PedroAuto extends OpMode {
    Follower follower;
    //727 741 4771
    int ARM_CONSTANT = 1200;
    Timer armTimer, pathTimer;
    Boolean forward = true;
    Hardware robot = Hardware.getInstance();

    enum ActionState{
        RAISE_ARMS,
        HANG_PRELOAD,
        PICK_UP_SPECIMEN_1,
        RAISE_ARM_1,
        HANG_SPECIMEN_1,
        PICK_UP_SPECIMEN_2,
        BACK_UP_ONCE,
        STRAFE_TO_SPECIMEN_2,

        RAISE_ARM_2,
        HANG_SPECIMEN_2,
        PICK_UP_SPECIMEN_3,
        RAISE_ARM_3,
        HANG_SPECIMEN_3

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
        GO_FORWARDS,
        PICK_UP_SPECIMEN_1,
        STRAFE_TO_SUBMERSIBLE_1,
        FORWARDS_TO_SUBMERSIBLE_1,

        BACKKWARDS_FROM_SUBMERSIBLE_1,
        PICK_UP_SPECIMEN_2,
        STRAFE_TO_SUBMERSIBLE_2,
        FORWARDS_TO_SUBMERSIBLE_2,
        BACKWARDS_FROM_SUBMERSIBLE_2,
        PICK_UP_SPECIMEN_3,
        STRAFE_TO_SUBMERSIBLE_3,

        FORWARDS_TO_SUBMERSIBLE_3,
        BACKWARDS_FROM_SUBMERSIBLE_3,
        STRAFE_TO_PARK,
        DONE
    }
    SpecimenAuto.PathState pathState = SpecimenAuto.PathState.GO_TO_SUBMERSIBLE;
    SpecimenAuto.ActionState actionState = SpecimenAuto.ActionState.RAISE_ARMS;
    Pose starting = new Pose(0, 0, 0);
    Path forwards, backwards;
    double lastX = starting.getX();
    boolean extendAlready;
    double lastY = starting.getY();
    double lastH = starting.getHeading();

    Path toSubmersible, Backwards;
    public void init(){
        follower = new Follower(hardwareMap);

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(30,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(30,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);

        follower.followPath(forwards);

    }
    //    public PathChain pushSpecimens(){
//
//        PathBuilder builder = new PathBuilder();
//        builder.addPath(huhuh);
//

    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }
        follower.update();

    }
}
