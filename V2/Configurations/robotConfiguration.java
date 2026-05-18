package org.firstinspires.ftc.teamcode.DECODE.V2.Configurations;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.Range;

@Config
public class robotConfiguration {
//    POSES ON EACH PERIODE
    public static Pose2d blueAimingTarget = new Pose2d(-62.7, 58.3, 0);
    public static Pose2d redAimingTarget = new Pose2d(-blueAimingTarget.position.x, -blueAimingTarget.position.y, 0);
    public static Pose2d blueBeginPoseTeleop = new Pose2d(-28.0, 28.2, Math.toRadians(90));
    public static Pose2d redBeginPoseTeleop = new Pose2d(-blueBeginPoseTeleop.position.x, -blueBeginPoseTeleop.position.y, Math.toRadians(90));
    public static Pose2d RedLoadingZone = new Pose2d(57.5, 58.5, 0);
    public static Pose2d BlueLoadingZone = new Pose2d(57.5, -58.5, 0);
    public static double[] TRIANGLE_X = {-55.3, -0.4, -55.3};
    public static double[] TRIANGLE_Y = {-55.0, -0.2, 55.0};
    public static double[] TRIANGLE_XS = {68.9, 46.4, 68.9};
    public static double[] TRIANGLE_YS = {-22.7, 0.0, 22.4};

    public static double[] HoodedAngle = {0.0 ,0.9};
    public static double nearDistance = 32;
    public static double farDistance = 144 ;
    public static double OPEN_GATE = 0.5;
    public static double CLOSE_GATE = 0.9;
    public static double INTAKE_SPEED = 1.0;
    public static double TURRET_OFFSET_X = 0.0;
    public static double TURRET_OFFSET_Y = 0.0;
    public static double MIN_ZONE_X = -72;
    public static double MAX_ZONE_X = 72;
    public static double MIN_ZONE_Y = -72;
    public static double MAX_ZONE_Y = 72;

    public static double HeadingKp = 2;
    public static double HeadingKi = 0.0;
    public static double HeadingKd = 0.0008;

    public double PolynomialShooter(double d) {
//        Coefficient. The data got from the desmos.
        double a = 1445.96132;
        double b = 1.0029;

        double power = a * Math.pow(b, d);
        if (power < 1500) power = 1500;
        if (power > 3000) power = 3000;

        return power;
    }

    public double flywheelSpeed(double dist) {
        double power = -0.00000830926 * Math.pow(dist, 4) + 0.00307213 * Math.pow(dist, 3)
                + 0.41343 * Math.pow(dist, 2) + 30.96129 + dist + 1111.89858;
        if (power < 0) power = 0;
        if (power > 2150) power = 2150;
        return power;
    }

    public double InterpolationShooter(double pos) {
        double[][] dataPoints = {
                {27.26, 1220},
                {33.24, 1240},
                {39.34, 1240},
                {48.91, 1240},
                {55.22, 1320},
                {59.69, 1360},
                {62.63, 1360},
                {65.54, 1360},
                {69.57, 1360},
                {72.85, 1400},
                {76.92, 1440},
                {80.53, 1480},
                {113.02, 1620},
                {117.47, 1840},
                {127.86, 1820},
                {132.33, 1840},
        };

        if (pos <= dataPoints[0][0]) {
            return dataPoints[0][1];
        }

        if (pos >= dataPoints[dataPoints.length - 1][0]) {
            return dataPoints[dataPoints.length - 1][1];
        }

        for (int i = 0; i < dataPoints.length - 1; i++) {
            double d1 = dataPoints[i][0];
            double p1 = dataPoints[i][1];
            double d2 = dataPoints[i + 1][0];
            double p2 = dataPoints[i + 1][1];

            if (pos >= d1 && pos <= d2) {
                double power = p1 + (p2 - p1) * (pos - d1) / (d2 - d1);

                if (power < 0) power = 0;
                if (power > 3000) power = 3000;

                return power;
            }
        }
        return 1500;
    }

    public double AngleAdjuster(double pos) {
        double t = (pos - nearDistance) / (farDistance - nearDistance);
        t = Range.clip(t, 0 , 1);
        return HoodedAngle[0] + (HoodedAngle[1] - HoodedAngle[0]) * t;
    }
    public double hoodAngle(double dis) {
        double hood = -4.74322e-8 * Math.pow(dis, 3) + -0.0000382626 * Math.pow(dis, 2)
                + 0.015535 * dis + -0.359574;
        if (hood < 0) hood = 0;
        if (hood > 0.95) hood = 0.95;
        return hood;
    }

    public double pivotX(double robotX, double robotHeading) {
        return robotX + TURRET_OFFSET_X * Math.cos(robotHeading) - TURRET_OFFSET_Y * Math.sin(robotHeading);
    }

    public double pivotY(double robotY, double robotHeading) {
        return robotY + TURRET_OFFSET_X * Math.sin(robotHeading) + TURRET_OFFSET_Y * Math.cos(robotHeading);
    }
}