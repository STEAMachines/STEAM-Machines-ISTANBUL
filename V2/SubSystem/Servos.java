package org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DECODE.V2.Configurations.robotConfiguration;

public class Servos {
    Servo gate, hoodAngle;
    private robotConfiguration rC = new robotConfiguration();

    private enum gateState {
        OPEN,
        CLOSE
    }
    public gateState currentGate = Servos.gateState.CLOSE;
    private ElapsedTime gateTime = new ElapsedTime();

    boolean isOpened;

    public Servos(HardwareMap hardwareMap) {
        rC = new robotConfiguration();
        gate = hardwareMap.get(Servo.class, "gate");
        hoodAngle = hardwareMap.get(Servo.class, "hoodAngle");
    }

    public void setHoodAngle(double dist) {
        hoodAngle.setPosition(rC.hoodAngle(dist));
    }

    public void setGate(String conditions) {
        if (conditions.equals("open")) {
            gate.setPosition(0.0);
            isOpened = true;
            currentGate = gateState.OPEN;
        } else {
            gate.setPosition(0.5);
            isOpened = false;
            currentGate = gateState.CLOSE;
        }
    }

    public boolean autoGate() {
        switch (currentGate) {
            case CLOSE:
                gate.setPosition(0.0);
                isOpened = true;
                currentGate = gateState.OPEN;
                gateTime.reset();
                ;
                return false;

            case OPEN:
                if (gateTime.milliseconds() > 600) {
                    gate.setPosition(0.5);
                    isOpened = false;
                    currentGate = gateState.CLOSE;
                    return true;
                }
                return false;
        }
        return false;
    }

     public boolean isOpened() {
        return isOpened;
     }
}
