package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class NormalizeColorSensor {
    NormalizedColorSensor colorSensor;

    public enum detectColors {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public static double RPurple = 0.5;
    public static double  GPurple = 0.5;
    public static double  BPurple = 0.1;

    public static double  RGreen = 0.4;
    public static double GGreen = 0.5;
    public static double  BGreen = 0.4;


    public NormalizeColorSensor(HardwareMap hardwareMap, String name) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, name);
        colorSensor.setGain(4);


        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

    }

    public detectColors getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float red = colors.red / colors.alpha;
        float green = colors.green / colors.alpha;
        float blue = colors.blue / colors.alpha;

        boolean isPurple = (red > RPurple) && (blue > BPurple) && (green > GPurple);
        boolean isGreen = (red < RGreen) && (blue < BGreen) && (green > GGreen);

        if (isPurple) {
            return detectColors.PURPLE;
        } else if (isGreen) {
            return detectColors.GREEN;
        }
//
//        telemetry.addData("R","%.3f", colors.red / colors.alpha);
//        telemetry.addData("G", "%.3f", colors.green / colors.alpha);
//        telemetry.addData("B", "%.3f", colors.blue / colors.alpha);
//        telemetry.update();

        return detectColors.UNKNOWN;
    }
}
