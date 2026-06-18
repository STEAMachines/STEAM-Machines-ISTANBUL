package org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpMode.NormalizeColorSensor;

public class subIntake {
    public DcMotorEx FrontIntake, ReversalIntake;
    private NormalizeColorSensor colorSensor1, colorSensor2, colorSensor3;
    private NormalizeColorSensor.detectColors dColor1, dColor2, dColor3;

    public subIntake(HardwareMap hardwareMap) {
        FrontIntake = hardwareMap.get(DcMotorEx.class, "Intake1");
        ReversalIntake = hardwareMap.get(DcMotorEx.class, "Intake2");

        FrontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        ReversalIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        colorSensor1 = new NormalizeColorSensor(hardwareMap, "color1");
        colorSensor2 = new NormalizeColorSensor(hardwareMap, "color2");
        colorSensor3 = new NormalizeColorSensor(hardwareMap, "color3");
    }

    public int ballCounting(Telemetry telemetry) {
        dColor1 = colorSensor1.getDetectedColor(telemetry);
        dColor2 = colorSensor2.getDetectedColor(telemetry);
        dColor3 = colorSensor3.getDetectedColor(telemetry);

        boolean front = isDetecting(dColor1);
        boolean middle = isDetecting(dColor2);
        boolean atGate = isDetecting(dColor3);

        int count;

        if (front && !(middle && atGate)) {
            count = 1;
            telemetry.addData("FRONT", front);
        } else if(front && middle && !atGate) {
            count = 2;
            telemetry.addData("FRONT", front);
            telemetry.addData("MIDDLE", middle);
        } else if (front && middle && atGate) {
            count = 3;
            telemetry.addData("FRONT", front);
            telemetry.addData("MIDDLE", middle);
            telemetry.addData("BACKERS", atGate);
        } else {
            count = 0;
            telemetry.addLine("NO BALLS");
        }
        telemetry.addData("COUNTED BALL", count);
        telemetry.update();
        return count;
    }

    private boolean isDetecting(NormalizeColorSensor.detectColors color) {
        return color == NormalizeColorSensor.detectColors.PURPLE ||
                color == NormalizeColorSensor.detectColors.GREEN;
    }

    public Action IntakeIn(double power) {return new IntakeAction(power);}

    public Action IntakeOut(double power) {return new IntakeAction(-power);}

    public Action StopIntake() {return new StopIntake();}

    private class IntakeAction implements Action {
        private double power;
        private boolean Initialized = false;

        public IntakeAction(double power) {
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!Initialized) {
                FrontIntake.setPower(power);
                ReversalIntake.setPower(power);
                Initialized = true;
            }
            return true;
        }
    }

    public class StopIntake implements Action {
        private boolean Initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!Initialized) {
                FrontIntake.setPower(0);
                ReversalIntake.setPower(0);
                Initialized = true;
            }
            return false;
        }
    }


    public class controlledIntake {
        public void intake(double trigger, boolean bumper, boolean additional) {
            if (trigger > 0.1) {
                FrontIntake.setPower(1);
                ReversalIntake.setPower(1);
            } else if (bumper) {
                FrontIntake.setPower(-1);
                ReversalIntake.setPower(-1);
            } else {
                FrontIntake.setPower(0);
                ReversalIntake.setPower(0);
            } if (additional) {
                FrontIntake.setPower(1);
            }
        }
    }
}
