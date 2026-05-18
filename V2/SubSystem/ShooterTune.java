package org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
@Config
public class ShooterTune extends OpMode {
    FtcDashboard dashboard;
    DcMotorEx shooter;

    public static double Vel = 1500;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Re-apply PIDF every loop so dashboard tuning takes effect live
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.5000, 0, 0, 13.1000);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        if (gamepad1.y) {
            shooter.setVelocity(Vel);
        } else {
            shooter.setVelocity(0);
        }

        // Create a fresh packet each loop and send it
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("currentVel", shooter.getVelocity());   // fixed: was getCurrentPosition()
        packet.put("targetVel", Vel);
        dashboard.sendTelemetryPacket(packet);
    }
}