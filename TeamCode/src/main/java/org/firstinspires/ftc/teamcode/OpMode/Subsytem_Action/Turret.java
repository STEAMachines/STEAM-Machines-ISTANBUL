package org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action;

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

    private double integralSum = 0;
    private double lastError = 0;
    private final double integralLimit = 15.0;
    private double turretError = 0.0;
//    PARAMETER PIDF
    public static double kP = 0.05;
    public static double kI = 0;
    public static double kD = 0.005;
    public static double kF = 0;

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinTimer.reset();
    }

    public void updateTurret(double robotX, double robotY, double robotHeading,
                             Pose2d target) {
        double x  = target.position.x - robotX;
        double y = target.position.y -  robotY;

        double fieldTargetAngle =  Math.toDegrees(Math.atan2(y, x));
        double turretTargetAngle = normalizeAngle(fieldTargetAngle - robotHeading);

        double error = calculatePID(turretTargetAngle);
        turret.setPower(Range.clip(error, 1, - 1));
    }

    private double calculatePID(double error) {
        double dt = spinTimer.seconds();
        spinTimer.reset();

        if (dt > 0.5) dt = 0.5;
        if (dt <= 0) dt = 0.02;

        if (Math.abs(error) < 2) {
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

        if (Math.abs(output) < 0.05 &&  Math.abs(error) > 2) {
            output = 0.05 * Math.signum(error);
        }

        return output;
    }


    private double normalizeAngle(double degree) {
        while (degree >  180) degree -= 360;
        while (degree < -180) degree += 360;
        return degree;
    }
}
