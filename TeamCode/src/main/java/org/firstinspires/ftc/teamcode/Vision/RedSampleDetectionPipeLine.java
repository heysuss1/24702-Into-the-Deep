package org.firstinspires.ftc.teamcode.Vision;

import static org.opencv.core.Core.inRange;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class RedSampleDetectionPipeLine extends OpenCvPipeline {
    static final double SAMPLE_HEIGHT = 3.47;
    static final double SAMPLE_LENGTH = 1.49;
    static final double FOCAL_LENGTH = 1600;
    Scalar lowerRed1 = new Scalar(0, 100, 100);
    Scalar upperRed1 = new Scalar(10, 255, 255);
    Scalar lowerRed2 = new Scalar(170, 100, 100);
    Scalar upperRed2 = new Scalar(180, 255, 255);
    Scalar lowerBlue = new Scalar(100, 100, 100);
    Scalar upperBlue = new Scalar(140, 255, 255);
    Mat redMask1 = new Mat();
    Mat redMask2 = new Mat();
    Mat blueMask = new Mat();
    Mat hsvImage = new Mat();

    Mat leftMap, rightMap, middleMap;
    double distance;
    double angle = 0;
    double leftAvg, middleAvg, rightAvg;
    String objectPos = "";
    Rect leftRect= new Rect(1,1, 426, 719);
    Rect middleRect= new Rect(427,1, 426, 719);
    Rect rightRect= new Rect(854,1, 426, 719);
    Mat image;
    public Mat processFrame(Mat frame){
        Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_BGR2HSV);
//        Imgproc.inRange(hsvImage, lowerRed1, upperRed1, mask1);
//        Imgproc.(hsvImage, lowerRed2, upperRed2, mask2);
        inRange(hsvImage, lowerRed1, upperRed1, redMask1);
        inRange(hsvImage, lowerRed2, upperRed2, redMask2);

        Core.bitwise_or(redMask1, redMask2, redMask1);  // Combine both red masks
        inRange(hsvImage, lowerBlue, upperBlue, blueMask);

        // Find contours for blue regions
        List<MatOfPoint> blueContours = new java.util.ArrayList<>();
        //Amelia is a monkey!!!!!!!!! jiya is so hot
        Imgproc.findContours(blueMask, blueContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


        leftMap = hsvImage.submat(leftRect);
        middleMap = hsvImage.submat(middleRect);
        rightMap = hsvImage.submat(rightRect);

        double leftAvg = Core.mean(leftMap).val[0];
        double rightAvg = Core.mean(rightMap).val[0];
        double middleAvg = Core.mean(middleMap).val[0];

        if (leftAvg > rightAvg && leftAvg > middleAvg) objectPos = "left";
        else if (rightAvg > leftAvg && rightAvg > middleAvg) objectPos = "right";
        else objectPos = "middle";
        // Draw contours and bounding boxes
        for (MatOfPoint contour : blueContours) {
            double area = Imgproc.contourArea(contour);
            if (area > 700){
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                Point[] points = new Point[4];
                rect.points(points);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(frame, points[i], points[(i+1)%4], new Scalar(0, 255, 0), 2);
                }

                // Get width, height, and angle of the bounding box
                double width = rect.size.width;
                double height = rect.size.height;
                angle = rect.angle;
                distance = checkDistance(width);
            }

//             = angle;

        }

        // Display the frame// You can use OpenCV's imshow to display the frame on-screen in a GUI application.
        // Imgproc.imshow("Camera", frame);
//        if (blueContours.isEmpty()){
//            distance = 69420;
//        }
//        Imgproc.rectangle(frame, new Point(50, 50), new Point(100, 100), new Scalar(0, 0, 255), 2);

        return frame;
    }

    public double getDistance(){
        return distance;
    }
    public double getAngle(){
        return angle;
    }

//    public double getAverages(){
//        return leftAvgrightAvg, middleAvg;
//    }
    public String getObjectPos(){return objectPos;}
    public double checkDistance(double pixelWidth){
        return (SAMPLE_HEIGHT * FOCAL_LENGTH)/ pixelWidth;
    }
}
