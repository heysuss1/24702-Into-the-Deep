package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Wrist extends Subsystem {
    public static final Wrist INSTANCE = new Wrist();
    public Servo wrist;
    private String wristName = "rotateServo";
    public Command straight(){
        return new ParallelGroup(
                new ServoToPosition(wrist, 0.437, this)
        );
    }
    public Command down(){
        return new ServoToPosition(
               wrist, 0.726, this
        );
    }
    public Command up(){
       return new ServoToPosition(
              wrist, 0.174, this
        );
    }

    public void initialize() {
        wrist = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, wristName);
    }
}
