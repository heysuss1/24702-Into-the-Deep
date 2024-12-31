package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToSeperatePositions;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class DualServoClaw extends Subsystem {
    public static final DualServoClaw INSTANCE = new DualServoClaw();
    public Servo leftServo;
    public Servo rightServo;
    public String leftServoName  = "leftServo";
    public String rightServoName = "rightServo";
    public Command open(){
        return new ParallelGroup(
                new ServoToPosition(leftServo, .639, this),
                new ServoToPosition(rightServo, .55, this)
        );
    }
    public Command close(){
        return new ParallelGroup(
                new ServoToPosition(leftServo, .508, this),
                new ServoToPosition(rightServo, .69, this)
        );
    }

    public void initialize() {
        leftServo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, leftServoName);
        rightServo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, rightServoName);

    }
}
