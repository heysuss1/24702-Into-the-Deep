package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Waiter;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class Claw implements Subsystem {

    public static Servo claw, diffy1, diffy2;
    public static final Claw INSTANCE = new Claw();
    public static Waiter waiter;
    int pitch, roll;
    private Claw(){}
    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }
    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }
    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode){
        HardwareMap hardwareMap = opMode.getOpMode().hardwareMap;
        claw = hardwareMap.get(Servo.class, "claw");
        diffy1 = hardwareMap.get(Servo.class, "diffy1");
        diffy2 = hardwareMap.get(Servo.class, "diffy2");
        waiter = new Waiter();
    }
    public static void close(){
        claw.setPosition(0.1);
    }
    public static void open(){
        claw.setPosition(0.4);
    }

//    public static void setWristPosition(int pitch, int roll){
//        rotateServo.setPosition(pos);
//    }

    public static void diddylate(double pitch, double roll){
//        pitch = Range.clip(pitch, 160, 300);
//        roll = Range.clip(roll, -30, 60);
        roll = roll/300;
        pitch = pitch/300;
        diffy1.setPosition(pitch - roll);
        diffy2.setPosition(pitch + roll);
    }
    @NonNull
    public static Lambda closeClaw(){
        return new Lambda("close claw")
                .addRequirements(INSTANCE)
                .setInit(Claw::close);
    }
    @NonNull
    public static Lambda openClaw(){
        return new Lambda("open claw")
                .addRequirements(INSTANCE)
                .setInit(Claw::open);
    }
    @NonNull
    public static Lambda wristDown(){
        return new Lambda("wrist down")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    waiter.start(400);
                    diddylate(175, 0);
                })
                .setFinish(() -> waiter.isDone());
    }

    @NonNull
    public static Lambda wristStraight(){
        return new Lambda("wrist straight")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    waiter.start(400);
                    diddylate(90, 0);
                })
                .setFinish(() -> waiter.isDone());
    }
    @NonNull
    public static Lambda wristUp(){
        return new Lambda("wrist down")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    waiter.start(400);
                    diddylate(15, 0);
                })
                .setFinish(() -> waiter.isDone());
    }
}



/*
public void openClaw(){
        robot.leftServo.setPosition(.639);
        robot.rightServo.setPosition(0.55);
    }
    public void closeClaw(){
        robot.leftServo.setPosition(.508);
        robot.rightServo.setPosition(.69);
    }
 */
/*if (currentGamepad2.y && !previousGamepad2.y) {
                robot.rotateServo.setPosition(0.174);
            }
            if (currentGamepad2.b && !previousGamepad2.y && !currentGamepad2.start) {
                robot.rotateServo.setPosition(0.437);
                // wrist movement commands
            }
            if (currentGamepad2.a && !previousGamepad2.a) {
                robot.rotateServo.setPosition(0.726);
            }

 */

