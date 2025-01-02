package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp (name ="Motor Test")
public class WheelTest extends LinearOpMode {

    Hardware robot = Hardware.getInstance();
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad2.a){
                robot.lb.setPower(0.5);
            }
            else if (gamepad2.b){
                robot.lf.setPower(0.5);

            }else if (gamepad2.y){
                robot.rb.setPower(0.5);

            }else if (gamepad2.x){
                robot.rf.setPower(0.5);

            } else{
                robot.setPower(0, 0,0,0);
            }
        }

    }
}
