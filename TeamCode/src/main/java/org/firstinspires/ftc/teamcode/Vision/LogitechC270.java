package org.firstinspires.ftc.teamcode.Vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

public class LogitechC270 extends OpMode {
    VisionPortal visionPortal;
    final static String CAMERA_NAME = "camera";
    VisionPortal.Builder builder = new VisionPortal.Builder();
    public void init(){
//        visionPortal = new;
//        public void init(){
//        visionPortal = new ;
        initalizeCamera();
    }
    public void loop(){

        visionPortal.getActiveCamera();
    }

    @Override
    public void stop() {
        visionPortal.close();
    }

    public void initalizeCamera(){
        builder.setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME));
        builder.setCameraResolution(new Size(640, 480));
        visionPortal = builder.build();


    }
}
