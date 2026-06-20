package org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class FlyWheel {
    // TODO: HOOD ANGLE (1) PALING BAWAH | (0.1) PALING ATAS
    public DcMotorEx flyWheel;
    public Servo hoodAngle, stooper;
    public static double OPEN_angle = 0.35;
    public static double CLOSE_angle = 1;
    public static double dropThreshold = 200;
    public static double flyWheelVel = 1300;

    public FlyWheel(HardwareMap hardwareMap) {
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        PIDFCoefficients flyWheelPIDF = new PIDFCoefficients(200, 0, 0 , 13.1);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flyWheelPIDF);

        hoodAngle = hardwareMap.get(Servo.class, "hoodAngle");
        stooper = hardwareMap.get(Servo.class, "stooper");
        stooper.setDirection(Servo.Direction.REVERSE);
        stooper.setPosition(CLOSE_angle);
    }

    public class Controlled {
        public void flyWheel(double distance, boolean button, boolean Stooper) {
                double currentVelocity = flyWheel.getVelocity();
                double targetVelocity = 1300;
            if (button) {

                if (targetVelocity - currentVelocity > dropThreshold) {
                    targetVelocity += 230;
                }

                flyWheel.setVelocity(targetVelocity);
                hoodAngle(distance);
            } else {
                flyWheel.setVelocity(0);
            }

            if (Stooper ) {
                stooper.setPosition(OPEN_angle);
            } else {
                stooper.setPosition(CLOSE_angle);
            }

        }

        public void hoodAngle(double distance) {
            hoodAngle.setPosition(setHoodAngle(distance));
        }
    }

    public double getDistance(double robotX, double robotY, Pose2d target) {
        double x = target.position.x - robotX;
        double y = target.position.y - robotY;

        return Math.sqrt(Math.pow(x, 2) +  Math.pow(y, 2));
    }

    private double flyWheelSpeed(double distance) {
        double[][] dataPoints = {
                {24.5, 1300},
                {30, 1350},
                {42, 1390},
                {48, 1400},
                {72, 1500},
                {82, 1580},
                {96, 1590},
                {120, 1600},
                {144, 1800}
        };

        if (distance <= dataPoints[0][0]) {
            return dataPoints[0][1];
        }

        if (distance >= dataPoints[dataPoints.length - 1][0]) {
            return dataPoints[dataPoints.length - 1][1];
        }

        for (int i = 0; i < dataPoints.length - 1; i++) {
            double d1 = dataPoints[i][0];
            double p1 = dataPoints[i][1];
            double d2 = dataPoints[i + 1][0];
            double p2 = dataPoints[i + 1][1];

            if (distance >= d1 && distance <= d2) {
                double power = p1 + (p2 - p1) * (distance - d1) / (d2 - d1);

                if (power < 0) power = 0;
                if (power > 3000) power = 3000;

                return power;
            }
        }
        return 1500;
    }
    private double setHoodAngle(double distance) {
        double hood = 1.0;
        if (hood < 0) hood = 0;
        if (hood > 0.95) hood = 0.95;
        return hood;
    }


//    AUTONOMOUS
    public Action flyWheelOn(double distance) {
        return new flyWheelOn(distance);
    }

    public Action flyWheelOff() {
        return new flyWheelOff();
    }

    private class flyWheelOn implements Action {
        private double distance;
        private boolean initialize= false;

        public flyWheelOn(double distance) {
            this.distance = distance;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialize) {
                flyWheel.setVelocity(flyWheelSpeed(distance));
                initialize = true;
            }

            return false;
        }
    }

    private class flyWheelOff implements Action {
        private boolean initialize = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (!initialize) {
                flyWheel.setPower(0);
                initialize = true;
            }
            return false;
        }
    }
}
