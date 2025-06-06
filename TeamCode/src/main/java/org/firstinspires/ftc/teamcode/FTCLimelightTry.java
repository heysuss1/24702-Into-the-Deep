package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.LimeLightPipeline;

@TeleOp(name = "Limelight3A Test")
public class FTCLimelightTry extends LinearOpMode {

    private LimeLightPipeline limelight;
    Follower follower;
    Hardware robot = Hardware.getInstance();
    public Path convertToPedro(double lateralDistance){
//        double pedroLateralDistance = -(lateralDistance);
//        double updatedForward = forwardDistance - (distanceToWheels);
        Pose currentPosition = new Pose(follower.getPose().getX(), follower.getPose().getY());
        Pose targetSamplePosition = new Pose(currentPosition.getX(), currentPosition.getY()-lateralDistance);
        Path newPath = new Path(new BezierLine(new Point(currentPosition), new Point(targetSamplePosition)));
        return newPath;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = new LimeLightPipeline(hardwareMap);
        robot.init(hardwareMap);
        limelight.setLimelightDetectorEnabled(true);
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        follower = new Follower(hardwareMap);
        double distanceX = 0;
        double distanceY = 0;

        waitForStart();
        while (opModeIsActive()) {
            LimeLightPipeline.DetectedObject detectedObject = limelight.getBestDetectedTarget(LimeLightPipeline.SampleType.RedAllianceSamples, false);
            if (detectedObject != null ){
                distanceX = detectedObject.targetPose.getX(DistanceUnit.INCH);
                distanceY =  detectedObject.targetPose.getY(DistanceUnit.INCH);
                if (gamepad1.y){
                    follower.followPath(convertToPedro(limelight.getObjectPose(detectedObject.targetPose).getX(DistanceUnit.INCH)));
                }
            }

            telemetry.addData("Distance x", distanceX);
            telemetry.addData("Distance y", distanceY);

            follower.update();
            robot.diddylate(175, robot.roll);
            telemetry.update();

        }
        limelight.setLimelightDetectorEnabled(false);
    }
    public void armExtend(int ticks){
        robot.armExtension.setPower(1);
        robot.armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armExtension.setTargetPosition(ticks);
    }
}

