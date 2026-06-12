package org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.Pose2d;

public class FlyWheel {
    // TODO: HOOD ANGLE (1) PALING BAWAH | (0.1) PALING ATAS
    public DcMotorEx flyWheel;
    public Servo hoodAngle;

    public FlyWheel(HardwareMap hardwareMap) {
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        PIDFCoefficients flyWheelPIDF = new PIDFCoefficients(200, 0, 0 , 13.1);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flyWheelPIDF);

        hoodAngle = hardwareMap.get(Servo.class, "hoodAngle");
    }

    public class Controlled {
        public void flyWheel(double distance, boolean button) {
            if (button) {
                flyWheel.setVelocity(flyWheelSpeed(distance));
            }
            else {
                flyWheel.setPower(0);
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
                {0, 0},
                {0, 0}
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
        double hood = -4.74322e-8 * Math.pow(distance, 3) + -0.0000382626 * Math.pow(distance, 2)
                + 0.015535 * distance + -0.359574;
        if (hood < 0) hood = 0;
        if (hood > 0.95) hood = 0.95;
        return hood;
    }
}
