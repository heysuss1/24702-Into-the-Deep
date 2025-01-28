package org.firstinspires.ftc.teamcode.runmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
@TeleOp
public class ColorSensorTest extends OpMode {
    //public RevColorSensorV3 colorSensor;
    private Telemetry TelemetryA;
    public static double distanceThreshold;
    public String colorPrediction;
    public double red;
    public double green;
    public double blue;
    public boolean hasSample;
    public double distance;
    Hardware robot = Hardware.getInstance();

    public void init(){
        TelemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        //colorSensor = hwMap.get(RevColorSensorV3.class, "colorSensor");

    }
    public void loop(){





        TelemetryA.update();
    }
}
