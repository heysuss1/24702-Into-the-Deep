//package org.firstinspires.ftc.teamcode.Vision;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//
//import java.util.List;
//
//public class Vision {
//
//
//    //Pipeline 0- Red only
//    //Pipeline 1- Yellow Only
//    //Pipeline 2- Blue Only
//    //Pipeline 3 - Red & Yellow
//    //Pipeline 4 - Blue & Yellow
//    private Limelight3A limelight;
//    private LLResult result;
//    private Follower follower;
//    private PathChain toTarget;
//    private double angle;
//    private double distance;
//    boolean blueTeam = false;
//
//    public void init(HardwareMap hardwareMap){
//        follower = new Follower(hardwareMap);
//        limelight.start();
//
//    }
//
//    public void findSample(){
//        double robotX = follower.getPose().getX();
//        double robotY = follower.getPose().getY();
//        result = limelight.getLatestResult();
////        List<LLResultTypes.DetectorResult> results = result.getDetectorResults();
////        for (LLResultTypes.DetectorResult c: results){
////            c.getTargetCorners();
////        }
//        if (result != null){
//
//        }
//
//        toTarget = follower.pathBuilder()
//                .addPath(new Path(new BezierLine( new Point(robotX, robotY), new Point(robotX + result.getTx(), robotY + result.getTy()))))
//                .setConstantHeadingInterpolation(follower.getPose().getHeading())
//                .build();
//
//        }
//    }
//    public void toSample(){
//
//    }
//
//
