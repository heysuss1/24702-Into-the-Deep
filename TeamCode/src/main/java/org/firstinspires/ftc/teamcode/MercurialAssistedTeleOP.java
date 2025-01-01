package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;


@TeleOp (name = "Mercurial teleop")
@Mercurial.Attach
@Arm.Attach
@Claw.Attach
public class MercurialAssistedTeleOP extends OpMode {
    double forward, sideways, turning, max;
    double scaleFactor;
    static boolean isAligned;
    static Follower follower;
    static Hardware robot = Hardware.getInstance();

    Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;
    public void init(){
        follower = new Follower(hardwareMap);
        robot.init(hardwareMap);
        Mercurial.gamepad2().y().onTrue(Claw.wristUp());
        Mercurial.gamepad2().b().onTrue(Claw.wristStraight());
        Mercurial.gamepad2().a().onTrue(Claw.wristDown());
        Mercurial.gamepad2().x().onTrue(Arm.parallelArm());
        Mercurial.gamepad1().y().onTrue(alignHeading());
        Mercurial.gamepad2().dpadLeft().onTrue(Arm.raiseSpecimen());
        Mercurial.gamepad2().dpadRight().onTrue(Arm.hangSpecimen());




    }
    public void loop(){
        forward = -(Math.atan(5 * gamepad1.left_stick_y) / Math.atan(5));
        sideways = (Math.atan(5 * gamepad1.left_stick_x) / Math.atan(5));
        turning = (Math.atan(5 * gamepad1.right_stick_x) / Math.atan(5));
        max = Math.max(Math.abs(forward - sideways - turning), Math.max(Math.abs(forward + sideways - turning), Math.max(Math.abs(forward + sideways + turning), Math.abs(forward + turning - sideways))));
        if (max > robot.maxSpeed) {
            scaleFactor = robot.maxSpeed / max;
        } else {
            scaleFactor = robot.maxSpeed;
        }
        scaleFactor *= Math.max(Math.abs(1 - gamepad1.right_trigger), 0.2);
        robot.setPower((forward - sideways - turning)*scaleFactor, (forward + sideways - turning) * scaleFactor, (forward + sideways + turning) * scaleFactor, (forward + turning - sideways) * scaleFactor);
//        Arm.moveArm(gamepad2);
//        Arm.extendArm(gamepad2);
        if (Arm.usePID && Arm.extensionAtTarget()){
            Arm.usePID = false;
        }
        telemetry.addData("Arm Vertical Pos", Arm.vertical.getCurrentPosition());
        telemetry.addData("Counter", Arm.counter);
        telemetry.addData("Use pid", Arm.usePID);
        telemetry.addData("is the arm vertical at target", Arm.getExtensionTarget());

    }

    public static Lambda alignHeading(){
        return new Lambda("align heading")
                .addRequirements(robot.rf)
                .setExecute(() -> {
                    if (!isAligned){
                        follower.update();
                        isAligned = Math.toDegrees(follower.getPose().getHeading()) < 2 && Math.toDegrees(follower.getPose().getHeading()) > 0;
                        if (Math.toDegrees(follower.getPose().getHeading()) > 180){
                            robot.setPower(0.3 , 0.3, -0.3, -0.3);
                        } else {
                            robot.setPower(-0.3, -0.3, 0.3, 0.3);
                        }
                    }

                });
//                .setFinish(isAligned);

    }
}
