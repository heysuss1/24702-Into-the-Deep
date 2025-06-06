package org.firstinspires.ftc.teamcode.UnusedPackages;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@TeleOp (name = "Camera Test")
public class UnusedLogitechC270 extends OpMode {
    VisionPortal visionPortal;
    final static String CAMERA_NAME = "camera";
    RedSampleDetectionPipeLine pipeline = new RedSampleDetectionPipeLine();
    Hardware robot = Hardware.getInstance();
    boolean pressedLeft = false;
    boolean pressedA = false;
//    OpenCvCamera camera;
    Vision VisionHandler = Vision.getInstance();
    Follower follower;
    OpenCvCamera camera;
    public void init() {
        robot.init(hardwareMap);
        follower = new Follower(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280, 720);
                telemetry.addLine("Erm, it worked!");
                telemetry.update();
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Erm, it didn't work");
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        FtcDashboard.getInstance().startCameraStream(camera, 60);
    }

    public void loop() {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        camera.setPipeline(pipeline);
//        VisionHandler.changeAngle(pipeline.ge)
        telemetry.addData("Distance", pipeline.getDistance());
        telemetry.addData("Angle", pipeline.getAngle());
        if (gamepad2.a && !pressedA){
            robot.roll = VisionHandler.changeAngle(robot, pipeline.getAngle());
            pressedA = true;
        } else {
            pressedA = false;
        }
        if (gamepad2.dpad_left && !pressedLeft){
            double targetX = pipeline.getTargetX();
            double targetY = pipeline.getTargetY();
            follower.followPath(new Path(new BezierLine(
                    new Point(x, y, 0), new Point(x+targetX, y+targetY)
            )));
            pressedLeft = true;
        } else {
            pressedLeft = false;
        }
//        telemetry.addData("quadrant: ", pipeline.getObjectPos());
        robot.diddylate(175, robot.roll);
        telemetry.update();
        follower.update();
    }

    public void stop(){
        camera.stopStreaming();
    }
    int[] clawPositions = {45, 20, 0, -15, -45};


//    public void initalizeCamera() {
//        camera.setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME));
//        builder.setCameraResolution(new Size(640, 480));
//        visionPortal = builder.build();
//
//
//    }
}
