package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;


//@Config
@TeleOp (name = "Mercurial teleop")
@Mercurial.Attach
@Arm.Attach
@Claw.Attach
public class MercurialAssistedTeleOP extends OpMode {
    double forward, sideways, turning, max;
    double scaleFactor;
    static boolean isAligned;
    static Follower follower;
    boolean pressingLT = false; boolean clawIsOpen = false;
    int[] clawPositions = {45, 20, 0, -15, -45};
    int  currentClawPosition = 0;
    double output, extensionError,verticalError, armOutput;
    boolean alignHeading = false;
    static Hardware robot = Hardware.getInstance();
    public static double kP, kI, kD, kF;
//    kP = .02;
    PIDFController pidExtension;
    PIDFController pidVertical;
    double looptime;
    Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;
    public void init(){
        follower = new Follower(hardwareMap);
        kP = .015;
        kD = 0;
        robot.init(hardwareMap);
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
        looptime = 0.0;
        pidExtension= new com.arcrobotics.ftclib.controller.PIDFController(kP, 0, kD, 0);
        pidVertical = new PIDFController(kP, 0, kD, 0);

        Mercurial.gamepad2().y().onTrue(Claw.wristUp());
        Mercurial.gamepad2().b().onTrue(Claw.wristStraight());
        Mercurial.gamepad2().a().onTrue(Claw.wristDown());
        Mercurial.gamepad2().leftBumper().onTrue(Claw.clawRollIncreasing());
        Mercurial.gamepad2().rightBumper().onTrue(Claw.clawRollDecreasing());

        Mercurial.gamepad2().x().onTrue(Arm.parallelArm(pidVertical, pidExtension));
//        Mercurial.gamepad1().y().onTrue();
        Mercurial.gamepad2().dpadLeft().onTrue(Arm.raiseSpecimen(pidVertical, pidExtension));
        Mercurial.gamepad2().dpadRight().onTrue(Arm.hangSpecimen(pidVertical, pidExtension));
        Mercurial.gamepad2().dpadDown().onTrue(Arm.goToBasket(pidVertical, pidExtension));
//



    }
    public void loop(){
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

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
        Arm.moveArm(gamepad2);
        Arm.extendArm(gamepad2);
//        if (Arm.usePID && Arm.extensionAtTarget()){
//            Arm.usePID = false;
//        }
////        output = pid.calculate(robot.armVertical.getCurrentPosition(), 2000);
////        robot.armVertical.setPower(output);
//        if (gamepad2.y && !hi){
//            Arm.raiseSpecimen(pidVertical, pidExtension);
//            hi = true;
//        } else{
//            hi = false;
//        }
//        if (gamepad1.y && !previousGamepad1.y) isAligned = true;


//        if (isAligned){
////           Arm.setVerticalTarget(2000);
////           Arm.setExtensionTarget(-1000);
//           Arm.hangSpecimen(pidVertical, pidExtension);
//        }
        if ((gamepad2.left_trigger > 0.1)&& !pressingLT){
            if(!clawIsOpen){
                //Open claw
                //robot.leftServo.setPosition(0.64);
                //robot.rightServo.setPosition(0.55);//may be wrong position
//                    robot.openClaw(1);
                robot.claw.setPosition(0.1);
                clawIsOpen = true;
            } else {
                //Close claw
                //robot.leftServo.setPosition(0.49);
                //robot.rightServo.setPosition(0.71);
//                    robot.openClaw(0);
                robot.claw.setPosition(0.4);
                clawIsOpen = false;
            }
            pressingLT = true;
        }

        double loop = System.nanoTime();
        Claw.diddylate();
        telemetry.addData("hz ", 1000000000/(loop - looptime));
        looptime = loop;
        telemetry.addData("Arm Vertical Pos", Arm.vertical.getCurrentPosition());
        telemetry.addData("Arm Extension Pos", Arm.extension.getCurrentPosition());
        telemetry.addData("Counter", output);
        telemetry.addData("Use pid", Arm.usePID);
        telemetry.addData("is the arm vertical at target", Arm.getExtensionTarget());
        telemetry.addData("get power", Arm.extension.getPower());
        telemetry.addData("output", pidExtension.calculate(0, extensionError));
    }

//    public static Lambda alignHeading(){
//        return new Lambda("align heading")
//                .addRequirements(robot.rf)
//                .setExecute(() -> {
//                    if (!isAligned){
//                        follower.update();
//                        isAligned = Math.toDegrees(follower.getPose().getHeading()) < 2 && Math.toDegrees(follower.getPose().getHeading()) > 0;
//                        if (Math.toDegrees(follower.getPose().getHeading()) > 180){
//                            robot.setPower(0.3 , 0.3, -0.3, -0.3);
//                        } else {
//                            robot.setPower(-0.3, -0.3, 0.3, 0.3);
//                        }
//                    }
//
//                });
////                .setFinish(isAligned);
//
//    }
}
