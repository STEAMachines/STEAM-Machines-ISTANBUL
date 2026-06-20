package org.firstinspires.ftc.teamcode.SUBSYSTEM;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter Subsystem — Velocity PIDF + Feedforward untuk Flywheel 6000 RPM
 *
 * Arsitektur kontrol:
 *   u(t) = kF * targetRPM                      ← Feedforward statik (dominant term)
 *        + kV * targetRPM                       ← Velocity feedforward (gravitasi/drag)
 *        + kA * (targetRPM - prevTargetRPM)/dt  ← Acceleration feedforward
 *        + kP * error                           ← Proportional
 *        + kI * ∫error dt                       ← Integral (eliminasi steady-state)
 *        + kD * (d/dt)error                     ← Derivative (damping)
 *
 * Cara tuning (urutan):
 *   1. Set semua gain = 0
 *   2. Naikkan kF sampai motor berputar ~85-90% target RPM tanpa overshooting
 *   3. Naikkan kV sedikit-sedikit untuk kompensasi drag/beban
 *   4. Naikkan kP sampai error mengecil cepat
 *   5. Tambahkan kD jika ada osilasi
 *   6. Naikkan kI (kecil saja, misal 0.0001) untuk eliminasi error sisa
 *   7. Tes kA hanya jika ada lag saat ramping kecepatan
 */
@Configurable
public class ShooterV2 {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    public DcMotorEx flywheel;

    // -------------------------------------------------------------------------
    // PARAMETER PIDF — ubah dari OpMode pakai FTC Dashboard atau tuning manual
    // -------------------------------------------------------------------------
    // Feedforward statik: power per RPM target  (mulai dari 1/maxRPM ≈ 0.000167)
    public static double kF = 0.00016;

    // Velocity feedforward: kompensasi drag aerodinamis & gesekan
    public static double kV = 0.000015;

    // Acceleration feedforward: kompensasi inersia saat target RPM berubah
    public static double kA = 0.000005;

    // PID gains
    public static double kP = 0.0002;
    public static double kI = 0.00005;
    public static double kD = 0.00001;

    // -------------------------------------------------------------------------
    // Konstanta Motor
    // -------------------------------------------------------------------------
    /**
     * Ganti dengan ticks-per-revolution motor yang kamu pakai:
     *   - GoBILDA Yellow Jacket 5203 435RPM  = 383.6 ticks/rev
     *   - NeveRest 40               = 1120   ticks/rev
     *   - Tetrix TorqueNADO         = 1440   ticks/rev
     *   - REV Hex Motor             = 288    ticks/rev
     *   - REV Core Hex Motor        = 288    ticks/rev
     *   - Andymark NeveRest Orbital = 537.7  ticks/rev  (paling umum)
     * Kalau shooter pakai motor high-speed tanpa encoder bawaan, sesuaikan.
     */
    private static final double TICKS_PER_REV = 28.0; // contoh: motor brushless 28 ticks/rev

    /**
     * Gear ratio antara motor dan flywheel.
     * Contoh: motor → 15T → 45T flywheel = 15.0/45.0 = 0.333
     * Jika direct-drive: 1.0
     */
    private static final double GEAR_RATIO = 1.0;

    // RPM target default (di-set dari luar via setTargetRPM)
    private double targetRPM = 0.0;

    // -------------------------------------------------------------------------
    // State PIDF
    // -------------------------------------------------------------------------
    private double integralSum    = 0.0;
    private double lastError      = 0.0;
    private double prevTargetRPM  = 0.0;
    private double lastVelocityRPM = 0.0;

    private final double integralLimit = 200.0;  // anti-windup cap (dalam RPM·s)

    // -------------------------------------------------------------------------
    // State internal
    // -------------------------------------------------------------------------
    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean isRunning = false;

    // Toleransi RPM: shooter dianggap "siap tembak" jika dalam range ini
    public static double RPM_TOLERANCE = 150.0;    // ± RPM dari target

    // Window untuk rata-rata velocity (smoothing derau encoder)
    private static final int VELOCITY_WINDOW = 5;
    private final double[] velocityBuffer = new double[VELOCITY_WINDOW];
    private int velocityBufferIndex = 0;

    // -------------------------------------------------------------------------
    // Konstruktor
    // -------------------------------------------------------------------------
    public ShooterV2(HardwareMap hardwareMap, String motorName) {
        flywheel = hardwareMap.get(DcMotorEx.class, motorName);

        // Sesuaikan arah putaran dengan mekanik shooter kamu
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // PENTING: kita kontrol power manual

        // FLOAT supaya tidak ada brake saat idle — lebih aman untuk flywheel
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        loopTimer.reset();
    }

    // Overload dengan nama default
    public ShooterV2(HardwareMap hardwareMap) {
        this(hardwareMap, "shooter");
    }

    // -------------------------------------------------------------------------
    // API Publik
    // -------------------------------------------------------------------------

    /**
     * Set RPM target shooter.
     * Panggil sekali saat mode berubah; update loop dihandle di updateShooter().
     */
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        if (rpm <= 0) {
            stop();
        } else {
            isRunning = true;
        }
    }

    /**
     * Hentikan shooter sepenuhnya.
     */
    public void stop() {
        isRunning = false;
        targetRPM = 0;
        integralSum = 0;
        lastError = 0;
        prevTargetRPM = 0;
        flywheel.setPower(0);
    }

    /**
     * WAJIB dipanggil setiap loop (di opMode.loop() atau dalam while).
     * Menghitung dan mengaplikasikan output PIDF + feedforward ke motor.
     */
    public void updateShooter() {
        if (!isRunning) {
            flywheel.setPower(0);
            return;
        }

        double dt = loopTimer.seconds();
        loopTimer.reset();

        // Clamp dt untuk menghindari spike saat pertama dipanggil
        if (dt > 0.5) dt = 0.5;
        if (dt <= 0)  dt = 0.02;

        // ── Baca kecepatan aktual ──────────────────────────────────────────────
        double currentRPM = getSmoothedRPM();

        // ── Hitung error ──────────────────────────────────────────────────────
        double error = targetRPM - currentRPM;

        // ── Feedforward ───────────────────────────────────────────────────────
        //   kF: power minimum untuk memutar pada targetRPM (statik)
        double ffStatic = kF * targetRPM;

        //   kV: kompensasi drag (velocity feedforward) — lebih besar RPM, lebih besar drag
        double ffVelocity = kV * targetRPM;

        //   kA: kompensasi inersia saat ada perubahan target RPM
        double dTarget = (targetRPM - prevTargetRPM) / dt;
        double ffAccel = kA * dTarget;
        prevTargetRPM = targetRPM;

        double feedforward = ffStatic + ffVelocity + ffAccel;

        // ── PID ───────────────────────────────────────────────────────────────
        //   Integral: hanya aktif saat error kecil (< 30% target) untuk anti-windup
        double integralBand = targetRPM * 0.30;
        if (Math.abs(error) < integralBand) {
            integralSum += error * dt;
        } else {
            integralSum = 0; // reset saat error besar (misal baru nyala)
        }
        integralSum = Range.clip(integralSum, -integralLimit, integralLimit);

        //   Derivative: gunakan velocity aktual supaya tidak ada spike saat error lompat
        double velocityError = (currentRPM - lastVelocityRPM) / dt;
        double derivative = -kD * velocityError; // negatif: damping bukan amplifikasi
        lastVelocityRPM = currentRPM;
        lastError = error;

        double pidOutput = (kP * error) + (kI * integralSum) + derivative;

        // ── Output total ──────────────────────────────────────────────────────
        double output = feedforward + pidOutput;
        output = Range.clip(output, 0.0, 1.0); // flywheel hanya boleh maju

        flywheel.setPower(output);
    }

    /**
     * Cek apakah shooter sudah pada RPM yang stabil dan siap menembak.
     */
    public boolean isAtTargetRPM() {
        if (!isRunning || targetRPM <= 0) return false;
        return Math.abs(targetRPM - getSmoothedRPM()) < RPM_TOLERANCE;
    }

    /**
     * Dapatkan RPM aktual (sudah di-smooth).
     */
    public double getCurrentRPM() {
        return getSmoothedRPM();
    }

    /**
     * Dapatkan RPM target saat ini.
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Dapatkan error RPM saat ini.
     */
    public double getRPMError() {
        return targetRPM - getSmoothedRPM();
    }

    /**
     * Cek apakah shooter sedang aktif.
     */
    public boolean isRunning() {
        return isRunning;
    }

    // -------------------------------------------------------------------------
    // Helper Internal
    // -------------------------------------------------------------------------

    /**
     * Konversi ticks/s dari encoder ke RPM, lalu masukkan ke rolling average buffer.
     * Rolling average dari 5 sample mengurangi derau encoder secara signifikan.
     */
    private double getSmoothedRPM() {
        double rawTicksPerSec = flywheel.getVelocity(); // ticks / detik (dari DcMotorEx)
        double rawRPM = (rawTicksPerSec / TICKS_PER_REV) * 60.0 / GEAR_RATIO;

        // Masukkan ke buffer
        velocityBuffer[velocityBufferIndex] = rawRPM;
        velocityBufferIndex = (velocityBufferIndex + 1) % VELOCITY_WINDOW;

        // Hitung rata-rata
        double sum = 0;
        for (double v : velocityBuffer) sum += v;
        return sum / VELOCITY_WINDOW;
    }

    // -------------------------------------------------------------------------
    // Telemetry Helper — panggil di opMode untuk debug
    // -------------------------------------------------------------------------

    /**
     * Return string telemetry untuk ditampilkan di Driver Station.
     * Contoh penggunaan: telemetry.addData("Shooter", shooter.getTelemetry());
     */
    public String getTelemetry() {
        return String.format(
                "Target: %.0f RPM | Actual: %.0f RPM | Error: %.0f | Ready: %b",
                targetRPM, getSmoothedRPM(), getRPMError(), isAtTargetRPM()
        );
    }
}