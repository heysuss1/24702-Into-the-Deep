package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//import static org.firstinspires.ftc.teamcode.MercurialAssistedTeleOP.robot;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
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
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class Arm implements Subsystem {
    public static DcMotorEx extension, vertical;
    public static final Arm INSTANCE = new Arm();
    public static double verticalTarget, extensionTarget, verticalOutput, extensionOutput;
    public static double kP = 0.02, kD = 0.0001, kI = 0, kF = 0;
//    public static boole
    private static double extensionError, verticalError;
//    public static com.arcrobotics.ftclib.controller.PIDFController pid;
    public static int counter = 0;
    public static Waiter waiter;
    public static boolean usePID = false;
    private Arm(){}
    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }
    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Claw.Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }
    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode){
        HardwareMap hardwareMap = opMode.getOpMode().hardwareMap;
        vertical = hardwareMap.get(DcMotorEx.class, "armV");
        extension = hardwareMap.get(DcMotorEx.class, "armE");
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setExtensionTarget(0);
        setVerticalTarget(0);
//        setDefaultCommand(update());
//        pid = new PIDFController(kP, kI, kD, kF);
        extension.setPower(0);
//        waiter = new Waiter();
    }
    public static void setVerticalTarget(int setpoint){ verticalTarget = setpoint;}
    public static void setExtensionTarget(int setpoint) {extensionTarget = setpoint;}
    public static double getVerticalTarget(){return verticalTarget;}
    public static double getExtensionTarget(){return extensionTarget;}

    public static void updatePID(boolean useExtension, PIDFController pidfVertical, PIDFController extend){
        extensionError = extend.calculate((extension.getCurrentPosition()),  extensionTarget);
        verticalError = pidfVertical.calculate(vertical.getCurrentPosition(), verticalTarget);

//        verticalError = pidfVertical.calculate(extension.getCurrentPosition(), -1200);
        vertical.setPower(verticalError);
        extension.setPower(extensionError);

    }
    public static void moveArm(Gamepad gamepad){

        if (gamepad.right_stick_y > 0.1){
            vertical.setPower(-1);
        } else if (gamepad.right_stick_y < -0.1) {
            vertical.setPower(1);
        } else{
            usePID = true;
            
            
            
            setExtensionTarget(extension.getCurrentPosition());
        }
    }
    public static void extendArm(Gamepad gamepad){
        if (gamepad.left_stick_y < -0.1 && extension.getCurrentPosition() > -2600){
            extension.setPower(-1);
            usePID = false;
        } else if (gamepad.left_stick_y > 0.1){
            usePID = false;
            extension.setPower(1);
        } else{
            if (!usePID){
                extension.setPower(0);
                usePID = true;
            }
            setExtensionTarget(extension.getCurrentPosition());
        }
    }
    public static boolean verticalAtTarget(){return Math.abs(verticalTarget - vertical.getCurrentPosition()) <= 1;}
    public static boolean extensionAtTarget(){return Math.abs(extensionTarget - extension.getCurrentPosition()) <=1;}

    public static Lambda update(PIDFController pidfVertical, PIDFController pidfExtension){
        return new Lambda("update")
                .addRequirements(INSTANCE)
                .setExecute(()->{
                    updatePID(true, pidfVertical, pidfExtension);
                });
    }
    @NonNull
    public static Lambda raiseSpecimen(PIDFController pidfVertical, PIDFController pidfExtension){
        return new Lambda("raise specimen")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                   setExtensionTarget(-1220);
                   setVerticalTarget(1500);
                })
                .setExecute(() -> {
                   Arm.updatePID(true, pidfVertical, pidfExtension);
                })
                .setFinish(() -> {
                   return verticalAtTarget() && extensionAtTarget();
                });
    }
    @NonNull
    public static Lambda hangSpecimen(PIDFController pidfVertical, PIDFController pidfExtension){
        return new Lambda("hang specimen")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setExtensionTarget(-350);
                    setVerticalTarget(1150);
                })
                .setExecute(() -> {
                    Arm.updatePID(true, pidfVertical, pidfExtension);
                })
                .setFinish(() -> {
                    return verticalAtTarget() && extensionAtTarget();
                });
    }

    @NonNull
    public static Lambda goToBasket(PIDFController pidfVertical, PIDFController pidfExtension){
        return new Lambda("go to basket")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setExtensionTarget(-2300);
                    setVerticalTarget(2100);
                })
                .setExecute(() -> {
                    Arm.updatePID(true, pidfVertical, pidfExtension);
                })
                .setFinish(() -> {
                    return verticalAtTarget() && extensionAtTarget();
                });
    }
    public static Lambda parallelArm(PIDFController pidfVertical, PIDFController pidfExtension){
        return new Lambda("parallel arm")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    setVerticalTarget(0);
                    setExtensionTarget(extension.getCurrentPosition());
                })
                .setExecute(() -> {
                    Arm.updatePID(false, pidfVertical, pidfExtension);
                })
                .setFinish(Arm::verticalAtTarget);


    }

    public static Lambda moveArm(){
        return new Lambda ("move arm")
                .addRequirements(INSTANCE)
                .setExecute(Arm::moveArm);
    }

}
