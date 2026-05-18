package org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.DECODE.V2.Configurations.robotConfiguration;

public class ShooterSubs {
    DcMotorEx flyWheel;
    private robotConfiguration rC = new robotConfiguration();

    public enum flyWheelState{
        SPINS,
        STOP
    }

    public flyWheelState currentflyWheelState = flyWheelState.STOP;

    public ShooterSubs(HardwareMap hardwareMap) {
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(200, 0, 0, 13.5);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void setFlyWheel(double dist, String isPressed) {
        if (isPressed.equals("spins")) {
            flyWheel.setVelocity(rC.flywheelSpeed(dist));
            currentflyWheelState = flyWheelState.SPINS;
        } else {
            flyWheel.setVelocity(0);
            currentflyWheelState = flyWheelState.STOP;
        }


    }
}
