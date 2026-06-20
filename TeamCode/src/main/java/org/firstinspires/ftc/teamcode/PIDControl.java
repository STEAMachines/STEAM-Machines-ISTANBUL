package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDControl {

    public double kP, kI, kD;

    private double integralSumInches  = 0;
    private double integralSumRadians = 0;
    private double integralSumDegree  = 0;

    private double lastErrorInch    = 0;
    private double lastErrorRadians = 0;
    private double lastErrorDegree  = 0;

    private double integralLimit = 30;

    private final ElapsedTime inchTimer    = new ElapsedTime();
    private final ElapsedTime radiansTimer = new ElapsedTime();
    private final ElapsedTime degreeTimer  = new ElapsedTime();

    // Flag buat skip derivative di loop pertama
    // (timer bisa sudah berjalan lama sejak constructor dipanggil)
    private boolean firstRunInch    = true;
    private boolean firstRunRadians = true;
    private boolean firstRunDegree  = true;

    private static final double MIN_DT = 0.002; // 2ms floor — cegah infinity

    public PIDControl(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        inchTimer.reset();
        radiansTimer.reset();
        degreeTimer.reset();
    }

    // ─── Calculate Inches ────────────────────────────────────────────────────

    public double calculateInches(double target, double current) {
        double error = target - current;

        double dt = inchTimer.seconds();
        inchTimer.reset();

        // Loop pertama: dt bisa besar (sejak init()), skip derivative
        if (firstRunInch) {
            firstRunInch  = false;
            lastErrorInch = error;
            dt = MIN_DT;
        }
        if (dt < MIN_DT) dt = MIN_DT;

        // Integral — hanya aktif kalau error cukup besar
        if (Math.abs(error) > 3.0) {
            integralSumInches += error * dt;
        } else {
            integralSumInches = 0;
        }
        integralSumInches = Range.clip(integralSumInches, -integralLimit, integralLimit);

        double derivative = (error - lastErrorInch) / dt;
        lastErrorInch = error;

        return (kP * error) + (kI * integralSumInches) + (kD * derivative);
    }

    // ─── Calculate Radians ───────────────────────────────────────────────────

    public double calculateRadians(double targetHeading, double currentHeading) {
        double error = angleWrapRadians(targetHeading - currentHeading);

        double dt = radiansTimer.seconds();
        radiansTimer.reset();

        if (firstRunRadians) {
            firstRunRadians  = false;
            lastErrorRadians = error;
            dt = MIN_DT;
        }
        if (dt < MIN_DT) dt = MIN_DT;

        if (Math.abs(error) > 0.1) {
            integralSumRadians += error * dt;
        } else {
            integralSumRadians = 0;
        }
        integralSumRadians = Range.clip(integralSumRadians, -integralLimit, integralLimit);

        double derivative = (error - lastErrorRadians) / dt;
        lastErrorRadians = error;

        return (kP * error) + (kI * integralSumRadians) + (kD * derivative);
    }

    // ─── Calculate Degree ────────────────────────────────────────────────────

    public double calculateDegree(double targetHeading, double currentHeading) {
        double error = angleWrapDegree(targetHeading - currentHeading);

        double dt = degreeTimer.seconds();
        degreeTimer.reset();

        if (firstRunDegree) {
            firstRunDegree  = false;
            lastErrorDegree = error;
            dt = MIN_DT;
        }
        if (dt < MIN_DT) dt = MIN_DT;

        if (Math.abs(error) > 10.0) {
            integralSumDegree += error * dt;
        } else {
            integralSumDegree = 0;
        }
        integralSumDegree = Range.clip(integralSumDegree, -integralLimit, integralLimit);

        double derivative = (error - lastErrorDegree) / dt;
        lastErrorDegree = error;

        return (kP * error) + (kI * integralSumDegree) + (kD * derivative);
    }

    // ─── Angle Wrap ──────────────────────────────────────────────────────────

    public double angleWrapDegree(double degree) {
        while (degree >  180) degree -= 360;
        while (degree < -180) degree += 360;
        return degree;
    }

    public double angleWrapRadians(double radians) {
        while (radians >  Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    // ─── Reset ───────────────────────────────────────────────────────────────

    public void reset() {
        integralSumInches  = 0;
        integralSumRadians = 0;
        integralSumDegree  = 0;
        lastErrorInch      = 0;
        lastErrorRadians   = 0;
        lastErrorDegree    = 0;

        firstRunInch    = true;
        firstRunRadians = true;
        firstRunDegree  = true;

        inchTimer.reset();
        radiansTimer.reset();
        degreeTimer.reset();
    }
}