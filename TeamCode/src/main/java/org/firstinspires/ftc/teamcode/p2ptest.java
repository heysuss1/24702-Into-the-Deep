package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "hz test")
public class p2ptest extends LinearOpMode {

    public void runOpMode(){
        ElapsedTime runtime = new ElapsedTime();
        double looptime = 0.0;
       waitForStart();
       while (opModeIsActive()){
           double loop = System.nanoTime();
           telemetry.addData("hz ", 1000000000/(loop - looptime));
           looptime = loop;
           telemetry.update();
       }

    }
}
