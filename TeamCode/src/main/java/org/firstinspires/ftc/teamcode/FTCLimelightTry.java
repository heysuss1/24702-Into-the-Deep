package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.VisionHardware;

import java.util.List;

@TeleOp(name = "Limelight3A Test")
public class LimeLightTestOpmode extends LinearOpMode {

    private VisionHardware limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = new VisionHardware();

        limelight.setLimelightDetectorEnabled(true);
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getLimeLightStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
            telemetry.addData("isStreamEnabled", limelight.isLimelightDetectorEnabled());
            telemetry.update();


            VisionHardware.DetectedObject detectedObjects = limelight.getBestDetectedTarget(VisionHardware.SampleType.RedAllianceSamples, false);
            detectedObjects.targetPose.
        }
        limelight.setLimelightDetectorEnabled(false);
    }
}
