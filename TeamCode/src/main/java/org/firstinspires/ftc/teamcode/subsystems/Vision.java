package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class Vision{
    public OpenCvCamera camera;
    private static Vision instance = null;
    public static Vision getInstance(){
        if (instance == null){
            instance = new Vision();
        }
        return instance;
    }
    public void init(HardwareMap hwMap){

    }
    int[] clawPositions = {45, 20, 0, -15, -45};
    public int changeAngle(Hardware robot, double angle) {
        if (angle >= 0 && angle <= 30) {
            return 45;
        } else if (angle > 30 && angle <= 70) {
            return 20;
        } else if (angle > 70 && angle <= 120) {
            return 0;
        } else if (angle > 120 && angle <= 155) {
            return -15;
        } else {
            return 0;
        }
    }
}
