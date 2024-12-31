package org.firstinspires.ftc.teamcode;

import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DualServoClaw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class Commands {
    Arm arm = new Arm();
    Wrist wrist = new Wrist();
    DualServoClaw claw = new DualServoClaw();

    public Command straightArm(){
        return new ParallelGroup(
                arm.INSTANCE.parallelArm(),
                wrist.INSTANCE.straight()
        );
    }
}
