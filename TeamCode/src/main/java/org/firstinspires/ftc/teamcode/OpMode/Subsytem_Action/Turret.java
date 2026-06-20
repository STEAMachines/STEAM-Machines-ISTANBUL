package org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action;

import android.media.Ringtone;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    public DcMotorEx turret;
    private ElapsedTime spinTimer = new ElapsedTime();

    private final double GearRatio = 537.6898395722 * (100.0 / 20.0);

    private double integralSum = 0;
    private double lastError = 0;
    private final double integralLimit = 15.0;
    private double turretError = 0.0;
//    PARAMETER PIDF
    public static double kP = 0.05;
    public static double kI = 0;
    public static double kD = 0.005;
    public static double kF = 0;

    private final double LEFT_LIMIT = 160.0;
    private final double RIGHT_LIMIT = -220.0;

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "rotateTurret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinTimer.reset();
    }

    public void updateTurret(double robotX, double robotY, double robotHeading,
                             Pose2d target) {
        double x = target.position.x - robotX;
        double y = target.position.y - robotY;

        double angle = Math.atan2(y, x);
        double fieldTurret = normalizeAngle(angle - robotHeading);
        double currentDeg = turret.getCurrentPosition() / GearRatio;


        if (currentDeg < RIGHT_LIMIT - 5) {
            fieldTurret = 360;
        } else if (currentDeg > LEFT_LIMIT + 5) {
            fieldTurret = -360;
        }


        fieldTurret = Range.clip(fieldTurret, RIGHT_LIMIT, LEFT_LIMIT);
        double error = normalizeAngle(fieldTurret - currentDeg);
        error = Range.clip(error, RIGHT_LIMIT, LEFT_LIMIT);
        turretError = error;
        turret.setPower(Range.clip(calculatePID(error), -1, 1));
    }

    private double calculatePID(double error) {
        double dt = spinTimer.seconds();
        spinTimer.reset();

        if (dt > 0.5) dt = 0.5;
        if (dt <= 0) dt = 0.02;

        if (Math.abs(error) < 3) {
            integralSum = 0.0;
            lastError = error;
            return 0;
        }

        if (Math.abs(error) < 15.0) {
            integralSum += error * dt;
        } else {
            integralSum = 0;
        }
        integralSum = Range.clip(integralSum, -integralLimit, integralLimit);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pidTerm = (error * kP) + (integralSum * kI) + (derivative * kD);
        double ff = kF * Math.signum(error);


        double output = pidTerm + ff;

//        if (Math.abs(output) < 0.05 &&  Math.abs(error) > 2) {
//            output = 0.05 * Math.signum(error);
//        }

        return output;
    }

    public double getTurretError() {
        return turretError;
    }

    private double normalizeAngle(double degree) {
        while (degree >  180) degree -= 360;
        while (degree < -180) degree += 360;
        return degree;
    }
}
