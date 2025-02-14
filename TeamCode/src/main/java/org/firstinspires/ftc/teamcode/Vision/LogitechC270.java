package org.firstinspires.ftc.teamcode.Vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp (name = "Camera Test")
public class LogitechC270 extends OpMode {
    VisionPortal visionPortal;
    final static String CAMERA_NAME = "camera";
    RedSampleDetectionPipeLine pipeline;
    OpenCvCamera camera;


    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
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
        camera.startStreaming(1280, 720);
        FtcDashboard.getInstance().startCameraStream(camera, 60);
        camera.setPipeline(pipeline);

    }

    public void loop() {
        camera.setPipeline(pipeline);
        telemetry.addData("Distance", RedSampleDetectionPipeLine.getDistance());
    }

    @Override
    public void stop() {
        visionPortal.close();
    }

//    public void initalizeCamera() {
//        camera.setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME));
//        builder.setCameraResolution(new Size(640, 480));
//        visionPortal = builder.build();
//
//
//    }
}
