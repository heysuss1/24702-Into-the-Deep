package org.firstinspires.ftc.teamcode.runmodes.Autos;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

import dev.frozenmilk.mercurial.Mercurial;

@Mercurial.Attach
@Arm.Attach
@Claw.Attach
public class MercurialSamplesAuto extends OpMode {

    enum PathState {
            GO_TO_BAR,
            GO_BACKWARDS,
            GO_TO_SAMPLE1,
            GO_TO_BASKET,
            GO_TO_SAMPLE2,
            GO_TO_BASKET_FROM_SAMPLE_2,
            GO_TO_SAMPLE3,
            GO_TO_BASKET_FROM_SAMPLE_3,
            GO_TO_PARKING
    }
    Follower follower;
    Hardware robot = Hardware.getInstance();
    public void init(){
        follower = new Follower(hardwareMap);
        robot.init(hardwareMap);
    }

    public void buildPaths(){

    }

    public void loop(){
        follower.update();

    }
}
