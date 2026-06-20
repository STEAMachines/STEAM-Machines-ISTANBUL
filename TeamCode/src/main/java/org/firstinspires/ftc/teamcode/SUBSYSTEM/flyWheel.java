package org.firstinspires.ftc.teamcode.SUBSYSTEM;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


@Configurable
public class flyWheel {
    // TODO: HOOD ANGLE (1) PALING BAWAH | (0.1) PALING ATAS
    public DcMotorEx flyWheel;
    public Servo hoodAngle, stooper;
    public static double OPEN_angle = 0.35;
    public static double CLOSE_angle = 1;
    public static double dropThreshold = 250;
    public static double additional = 250;
    public static double addit_2 = 300;
    public static double flyWheelVel = 1300;
    public static double treshold_DUANIHHH = 1160;

    public flyWheel(HardwareMap hardwareMap) {
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        PIDFCoefficients flyWheelPIDF = new PIDFCoefficients(200, 0, 0, 13.1);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flyWheelPIDF);

        hoodAngle = hardwareMap.get(Servo.class, "hoodAngle");
        stooper = hardwareMap.get(Servo.class, "stooper");
        stooper.setDirection(Servo.Direction.REVERSE);
        stooper.setPosition(CLOSE_angle);
    }

    public void flyWheel(double distance, boolean button, boolean Stooper) {
        double currentVelocity = flyWheel.getVelocity();
        double targetVelocity = flyWheelVel;
        if (button) {

            if (targetVelocity - currentVelocity > dropThreshold) {
                targetVelocity += additional;
            }

            if (targetVelocity - currentVelocity > dropThreshold && currentVelocity < treshold_DUANIHHH) {
                targetVelocity += addit_2;
            }

            flyWheel.setVelocity(targetVelocity);
            hoodAngle(distance);
        } else {
            flyWheel.setVelocity(0);
        }

        if (Stooper) {
            stooper.setPosition(OPEN_angle);
        } else {
            stooper.setPosition(CLOSE_angle);
        }

    }

    public void hoodAngle(double distance) {
        hoodAngle.setPosition(setHoodAngle(distance));
    }

    public double getDistance(double robotX, double robotY, Pose target) {
        double x = target.getX() - robotX;
        double y = target.getY() - robotY;

        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public void ambilData() {
        flyWheel.setVelocity(flyWheelVel);
    }

    private double flyWheelSpeed(double distance) {
        double[][] dataPoints = {
                {59.10188222561065, 1180},
//                {73.53056301784704, 1290},
                {84.36062489834401, 1342},
                {98.40638290806066, 1445},
                {02.05217594968505, 1500},
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
}