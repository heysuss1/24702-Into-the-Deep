package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.control.coefficients.PIDCoefficients;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDController;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class Arm extends Subsystem {
    public static final Arm INSTANCE = new Arm();
    public PIDController extensionPID = new PIDController(new PIDCoefficients(0.02, 0.0, 0.0001));
    public PIDController verticalPID = new PIDController(new PIDCoefficients(0.0219, 0.0, 0.0001));

    MotorEx extension, vertical;
    String extensionName = "armE";
    String verticalName = "armV";

    public Command liftSpecimen(){
        return new ParallelGroup(
          new RunToPosition(
                  extension,
                  -1200.0,
                  extensionPID,
                  this
          ),
          new RunToPosition(
                  vertical,
                  2300.0,
                  verticalPID,
                  this
          )
        );
    }
    public Command hangSpecimen(){
        return new ParallelGroup(
                new RunToPosition(
                        extension,
                        -350,
                        extensionPID,
                        this
                ),
                new RunToPosition(
                        vertical,
                        2100,
                        verticalPID,
                        this
                )
        );
    }
    public Command liftBasket(){
        return new ParallelGroup(
                new RunToPosition(
                        extension,
                        -350,
                        extensionPID,
                        this
                ),
                new RunToPosition(
                        vertical,
                        2100,
                        verticalPID,
                        this
                )
        );
    }
    public Command hangBasket(){
        return new ParallelGroup(
                new RunToPosition(
                        extension,
                        -2300,
                        extensionPID,
                        this
                ),
                new RunToPosition(
                        vertical,
                        4100,
                        verticalPID,
                        this
                )
        );
    }
    public Command parallelArm(){
        return new ParallelGroup(
                new RunToPosition(
                        vertical,
                        -180,
                        verticalPID,
                        this
                )
        );
    }
    public void initialize(){
        vertical = new MotorEx(verticalName);
        extension = new MotorEx(extensionName);
    }

}

