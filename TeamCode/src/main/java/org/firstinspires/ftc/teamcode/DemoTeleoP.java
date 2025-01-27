//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//public class DemoTeleOP extends LinearOpMode {
//
//    Hardware robot = Hardware.getInstance();
//    public void runOpMode(){
//        robot.init(hardwareMap);
//        waitForStart();
//        while (opModeIsActive()){
//           if (gamepad2.left_stick_y < -0.1){
//                robot.armExtension.setPower(0.5);
//           } else if (gamepad2.left_stick_y > .1){
//               robot.armExtension.setPower(-0.5);
//           }
//           else{
//               robot.armExtension.setPower(0);
//           }
//           if (gamepad2.x){
//               robot.leftServo.setPosition(0.5);
//           }
//        }
//    }
//
//}
