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

    public static Servo leftServo, rightServo, rotateServo;
    public static final Claw INSTANCE = new Claw();
    public static Waiter waiter;
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
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        waiter = new Waiter();
    }
    public static void close(){
        leftServo.setPosition(.508);
        rightServo.setPosition(.69);
    }
    public static void open(){
        leftServo.setPosition(.639);
        rightServo.setPosition(.55);
    }

    public static void setWristPosition(double pos){
        rotateServo.setPosition(pos);
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
                    setWristPosition(.726);
                })
                .setFinish(() -> waiter.isDone());
    }

    @NonNull
    public static Lambda wristStraight(){
        return new Lambda("wrist straight")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    waiter.start(400);
                    setWristPosition(.437);
                })
                .setFinish(() -> waiter.isDone());
    }
    @NonNull
    public static Lambda wristUp(){
        return new Lambda("wrist down")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    waiter.start(400);
                    setWristPosition(.174);
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

