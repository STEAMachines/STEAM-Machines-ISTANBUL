package org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class subIntake {
    public DcMotorEx FrontIntake, ReversalIntake;

    public subIntake(HardwareMap hardwareMap) {
        FrontIntake = hardwareMap.get(DcMotorEx.class, "Intake1");
        ReversalIntake = hardwareMap.get(DcMotorEx.class, "Intake2");

        FrontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        ReversalIntake.setDirection(DcMotorSimple.Direction.FORWARD);
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
        public void IntakeIn(double triggered) {
            if (triggered > 0.1) {
                FrontIntake.setPower(1);
                ReversalIntake.setPower(1);
            } else {
                FrontIntake.setPower(0);
                ReversalIntake.setPower(0);
            }
        }
        public void IntakeOut(boolean bumper) {
            if (bumper) {
                FrontIntake.setPower(-1);
                ReversalIntake.setPower(-1);
            }
            else {
                FrontIntake.setPower(0);
                ReversalIntake.setPower(0);
            }
        }
    }
}
