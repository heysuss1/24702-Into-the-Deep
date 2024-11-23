//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//
//public class LinearAuto extends LinearOpMode {
//    Hardware robot = Hardware.getInstance();
//    Follower follower = new Follower(hardwareMap);
//    public void runOpMode(){
//        robot.init(hardwareMap);
//        public static PathChain path(){
//            PathBuilder builder = new PathBuilder();
//            builder.addPath((new BezierLine(new Point(5, 65, Point.CARTESIAN), new Point(31, 65, Point.CARTESIAN))))
//                    .setTangentHeadingInterpolation();
////                .addPath(new BezierLine(new Point(40, 65, Point.CARTESIAN), new Point(5, 65, Point.CARTESIAN)))
////                .setTangentHeadingInterpolation();
////                .addPath(new BezierLine(new Point(46, 30, Point.CARTESIAN), new Point(56, 24, Point.CARTESIAN)));
//            return builder.build();
//        }
//        waitForStart();
//
//        while (robot.armExtension.isBusy()){
//            robot.armExtension.setTargetPosition(-1200);
//            robot.armExtension.setPower(1);
//        }
//    }
//}
