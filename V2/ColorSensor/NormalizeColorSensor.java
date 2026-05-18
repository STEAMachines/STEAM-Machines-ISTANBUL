package org.firstinspires.ftc.teamcode.DECODE.V2.ColorSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NormalizeColorSensor {
    NormalizedColorSensor colorSensor;

    public enum detectColors {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public NormalizeColorSensor(HardwareMap hardwareMap, String name) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, name);
        colorSensor.setGain(4);

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

    }

    public detectColors getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float red = colors.red / colors.alpha;
        float green = colors.green / colors.alpha;
        float blue = colors.blue / colors.alpha;

        boolean isPurple = (red > 0.5) && (blue > 0.1) && (green < 0.5);
        boolean isGreen = (red < 0.4) && (blue < 0.4) && (green > 0.5);

        if (isPurple) {
            return detectColors.PURPLE;
        } else if (isGreen) {
            return detectColors.GREEN;
        }

        telemetry.addData("R", colors.red / colors.alpha);
        telemetry.addData("G", colors.green / colors.alpha);
        telemetry.addData("B", colors.blue / colors.alpha);
        telemetry.addData("Purple", isPurple ? "PURPLE" : null);
        telemetry.addData("Green", isGreen ? "GREEN" : null);
        telemetry.update();

        return detectColors.UNKNOWN;
    }
}
