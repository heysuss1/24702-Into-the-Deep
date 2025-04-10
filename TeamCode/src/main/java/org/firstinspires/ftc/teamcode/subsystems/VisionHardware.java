package org.firstinspires.ftc.teamcode.subsystems;
import android.util.Log;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This class implements a specifed neural detector using Limelight 3A.
 */
public class VisionHardware {

    public static class DetectedObject {
        public final LLResult llResult;
        public Object result;
        public final String objectName;
        public final Pose2D targetPose;
        public final Point[] vertices;
        public final double rotatedRectAngle;

        public DetectedObject(LLResult llResult, Object result, double cameraPitch, double cameraHeight)
        {
            this.llResult = llResult;
            this.result = result;
            objectName = ((LLResultTypes.DetectorResult) result).getClassName();
            this.targetPose = getTargetPose(cameraPitch, cameraHeight);
            this.vertices = getRotatedRectVertices();
            double side1 = Math.abs(vertices[1].x - vertices[0].x);
            double side2 = Math.abs(vertices[3].y - vertices[0].y);
            if (side2 >= side1)
            {
                rotatedRectAngle = 90; //may need to be switched
            }
            else
            {
                rotatedRectAngle = 0;
            }
        }   //DetectedObject

        public String getObjectName() {
            return objectName;
        }   //getObjectName

        public Double getRotatedRectAngle() {
            return rotatedRectAngle;
        }   //getRotatedRectAngle

        public Pose2D getObjectPose() {
            return targetPose;
        }   //getObjectPose


        private Point[] getRotatedRectVertices() {
            Point[] vertices = null;
            List<List<Double>> corners;

            corners = ((LLResultTypes.DetectorResult) result).getTargetCorners();

            if (corners != null && !corners.isEmpty()) {
                vertices = new Point[corners.size()];
                for (int i = 0; i < vertices.length; i++) {
                    List<Double> vertex = corners.get(i);
                    vertices[i] = new Point(vertex.get(0), vertex.get(1));
                }
            }

            return vertices;
        }   //getRotatedRectVertices

        private Pose2D getTargetPose(double cameraPitch, double cameraHeight) {
            Pose2D targetPose;
            double camPitchRadians = Math.toRadians(cameraPitch);
            double targetPitchDegrees = llResult.getTy();
            double targetYawDegrees = llResult.getTx();
            double targetPitchRadians = Math.toRadians(targetPitchDegrees);
            double targetYawRadians = Math.toRadians(targetYawDegrees);

            //Assuming your object is at a height of 0.
            double targetDepth = cameraHeight / Math.tan(camPitchRadians + targetPitchRadians);
            targetPose = new Pose2D(DistanceUnit.INCH, targetDepth * Math.sin(targetYawRadians), targetDepth * Math.cos(targetYawRadians), AngleUnit.DEGREES, targetYawDegrees);

            return targetPose;
        }

        @Override
        @NonNull
        public String toString() {
            return "ObjectName=" + getObjectName() +
                    ", targetPose=" + pose2DString(getObjectPose()) +
                    ", vertices=" + Arrays.toString(getRotatedRectVertices()) +
                    ", rotatedRectAngle=" + getRotatedRectAngle();
        }
    }

    public enum SampleType {
        RedSample,
        BlueSample,
        YellowSample,
        RedAllianceSamples,
        BlueAllianceSamples
    }   //enum SampleType

    public final Limelight3A limelight;
    private Double lastResultTimestamp = null;
    //Camera Pose (These could also be stored in a global constants class)
    private double cameraPitch = 0.0;  //Calculate pitch using atan2(cameraHeight / distances). Distances is form the lens to the crosshair center
    private double cameraHeight = 0.0; //From the center of the lens to the ground.


    public LimelightNeuralDetector(HardwareMap hardwareMap) {
        {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight3a");
            limelight.pipelineSwitch(0); //Assuming your neural detector is on pipeline 0
        }
    }

    public void setLimelightDetectorEnabled(boolean enabled) {
        // Check Limelight has been initialized
        if (limelight != null) {
            boolean isActive = isLimelightDetectorEnabled();
            if (!isActive && enabled) {
                limelight.start();
            } else if (isActive && !enabled) {
                limelight.stop();
            }
        }
    }

    public boolean isLimelightDetectorEnabled() {
        return limelight.isConnected() && limelight.isRunning();
    }

    public LLStatus getLimeLightStatus() {
        return limelight.getStatus();
    }

    private ArrayList<DetectedObject> getDetectedObjects()
    {
        ArrayList<DetectedObject> detectedObjs = null;
        LLResult llResult = limelight.getLatestResult();
        Log.i("Limelight", "getDetectedObjects: llResult= " + llResult + ", isValid= " + llResult.isValid());
        if(llResult != null && llResult.isValid())
        {
            double resultTimestamp = llResult.getTimestamp();
            Log.i("Limelight", "timestampCheck: resultTimestamp= " + resultTimestamp + ", lastResultTimestamp= " + lastResultTimestamp);
            if (lastResultTimestamp == null || resultTimestamp != lastResultTimestamp)
            {
                List<?> resultList = llResult.getDetectorResults();
                lastResultTimestamp = resultTimestamp;

                Log.i("Limelight", "resultListSize= " + (resultList != null ? resultList.size() : 0));
                if (resultList != null && !resultList.isEmpty())
                {
                    detectedObjs = new ArrayList<>();
                    for (Object result: resultList)
                    {
                        DetectedObject object = new DetectedObject(llResult, result, cameraPitch, cameraHeight);
                        Log.i("Limelight", "DetectedObject: " + object);
                        detectedObjs.add(object);
                    }
                }
            }
        }

        return detectedObjs;
    }

    private ArrayList<DetectedObject> sortDetectedTargets(String[] targetSampleNames)
    {
        ArrayList<DetectedObject> validTargets = null;
        ArrayList<DetectedObject> detectedObjects = getDetectedObjects();

        if (detectedObjects != null && !detectedObjects.isEmpty())
        {
            if (targetSampleNames != null)
            {
                validTargets = new ArrayList<>();
                for (DetectedObject object : detectedObjects)
                {
                    for (String sampleName : targetSampleNames)
                    {
                        if (object.getObjectName().equals(sampleName))
                            validTargets.add(object);
                    }
                }
            }
            else
            {
                validTargets = detectedObjects;
            }
        }

        Log.i("Limelight", "sortDetectedTarget: possibleTargets= " + validTargets);
        return validTargets;
    }

    public DetectedObject getBestDetectedTarget(SampleType sampleType, boolean validateObjectsLocation)
    {
        String[] targetColors = null;
        DetectedObject bestTarget = null;
        ArrayList<DetectedObject> possibleTargets;

        switch (sampleType)
        {
            case RedSample:
                targetColors = new String[]{"red"}; //make sure these are the same as your detector class labels
                break;
            case BlueSample:
                targetColors = new String[]{"blue"};
                break;
            case YellowSample:
                targetColors = new String[]{"yellow"};
                break;
            case RedAllianceSamples:
                targetColors = new String[]{"red", "yellow"};
                break;
            case BlueAllianceSamples:
                targetColors = new String[]{"blue", "yellow"};
                break;
        }
        Log.i("Limelight", "targetColors= " + Arrays.toString(targetColors) + ", validateObjectsLocation= " + validateObjectsLocation);
        possibleTargets = sortDetectedTargets(targetColors);

        if (possibleTargets != null && !possibleTargets.isEmpty())
        {
            if (validateObjectsLocation)
            {
                for (DetectedObject object: possibleTargets)
                {
                    if (validateTarget(object))
                    {
                        // return the first valid object.
                        bestTarget = object;
                        break;
                    }
                }
            }
            else
            {
                // return the first object.
                bestTarget = possibleTargets.get(0);
            }
        }

        Log.i("Limelight", "bestTarget= " + bestTarget);
        return bestTarget;
    }

    private boolean validateTarget(DetectedObject a) {
        // Inside the limelight dashboard, object are partially sorted by location.
        // But depending on how they are sorted your best detected sample could be position to close to a wall etc.
        // So we do one final check to make sure the target is not in a bad spot.
        double minY = 2;  // insure sample is not to close to wall
        double maxY = 20; // insure sample is not out of working area
        double maxX = 3.25; // insure sample is not to close to sub wall

        boolean aInvalid = a.targetPose.getY(DistanceUnit.INCH) < minY || a.targetPose.getY(DistanceUnit.INCH) > maxY || a.targetPose.getX(DistanceUnit.INCH) > maxX;

        return !aInvalid;
    }

    //Pose2D does not have a toString
    public static String pose2DString(Pose2D pose2D)
    {

        return " (" + pose2D.getX(DistanceUnit.INCH) + "," + pose2D.getY(DistanceUnit.INCH) + "," + pose2D.getHeading(AngleUnit.DEGREES) + ")";
    }
}
