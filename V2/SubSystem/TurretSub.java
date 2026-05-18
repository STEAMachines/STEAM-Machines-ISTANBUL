package org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DECODE.V2.Configurations.robotConfiguration;

@Config
public class TurretSub {
    final robotConfiguration rC = new robotConfiguration();
    DcMotorEx spinTurret;
    ElapsedTime spinTimer = new ElapsedTime();
    private final double TICKS_PER_REV = (((1 + (46.0/17)) * (1 + (46.0/11))) * 28);
    private final double GearRatio = 100.0 / 20.0;
    private final double OutputSpeed = TICKS_PER_REV * GearRatio;
    final double TicksPerDegree = (OutputSpeed / 360.0);
    private double integralSum = 0;
    private double lastError = 0;
    private final double integralLimit = 15.0;
    public static double kP = 0.05;
    public static double kI = 0;
    public static double kD = 0.005;
    public static double kF = 0;

    private final double turretLimitDeg = 120.0;
    private final double maxLimit = Math.toRadians(120);
    private final double MinLimit = Math.toRadians(-120);
    private double turretError = 0.0;

    public TurretSub(HardwareMap hardwareMap) {
        spinTurret = hardwareMap.get(DcMotorEx.class, "rotateTurret");
        spinTurret.setDirection(DcMotorSimple.Direction.FORWARD);
        spinTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinTimer.reset();
    }

    public void aimingTurret(Pose2d Target, double RobotX, double RobotY, double robotHeading, double VelX, double VelY) {
        double dist = getDistanceTarget(RobotX, RobotY, robotHeading, Target);
        double tof = dist / 200.0;

        Pose2d predicted = new Pose2d(Target.position.x + VelX * tof, Target.position.y + VelY * tof, Target.heading.toDouble() );

        double pivotX = rC.pivotX(RobotX, robotHeading);
        double pivotY = rC.pivotY(RobotY, robotHeading);

        double x = predicted.position.x - pivotX;
        double y = predicted.position.y - pivotY;

        double angle = Math.atan2(y, x); // Vector(Pose) -> Radians
        double turret = angleWrapRadians(angle - robotHeading);
        double currentDeg = turretRad();

//        Limit rotasinya
        if (currentDeg > maxLimit + 5) {
            turret = Math.toRadians(-360);
        } else if (currentDeg < MinLimit - 5) {
            turret = Math.toRadians(360);
        }
        turret = Range.clip(turret, MinLimit, maxLimit);
        double error = angleWrapRadians(turret - currentDeg);
        error = Range.clip(error, -turretLimitDeg, turretLimitDeg);
        turretError = error;

        spinTurret.setPower(Range.clip(calculatePID(error), -1, 1));
    }

    public double turretDeg() {
        return spinTurret.getCurrentPosition() / TicksPerDegree;
    }
    public double turretRad() {
        return Math.toRadians(turretDeg());
    }

    public boolean isAimed() {
        return Math.abs(turretError) < Math.toRadians(3.0);
    }
    public double getDistanceTarget(double rX, double rY, double rH, Pose2d Target) {
        double turretX = rC.pivotX(rX, rH);
        double turretY = rC.pivotY(rY, rH);

        double x = Target.position.x - turretX;
        double y = Target.position.y - turretY;

        return Math.hypot(x, y);
    }

    private double calculatePID(double error) {
        double dt = spinTimer.seconds();
        spinTimer.reset();

        if (dt > 0.5) dt = 0.5;
        if (dt <= 0) dt = 0.02;

        if (Math.abs(error) < 3.0) {
            integralSum = 0.0;
            lastError = error;
            return 0;
        }

        if (Math.abs(error) < 10.0) {
            integralSum += error * dt;
        } else {
            integralSum = 0;
        }
        integralSum = Range.clip(integralSum, -integralLimit, integralLimit);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pidTerm = (error * kP) + (integralSum * kI) + (derivative * kD);
        double ff = kF * Math.signum(error);

        double totalPidf = pidTerm + ff;
//        klo ada target miss kecil bisa dibenerin klo si motor gk kuat
        if (Math.abs(totalPidf) < 0.05 && Math.abs(error) > 0.5) {
            totalPidf = 0.05 * Math.signum(totalPidf);
        }

        return totalPidf;
    }

    //    NgeWrap putaran
    private double angleWrapRadians(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }


}
