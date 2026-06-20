package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SUBSYSTEM.Intake;
import org.firstinspires.ftc.teamcode.SUBSYSTEM.ShooterV2;

@TeleOp
public class ShooterOpModeTest extends OpMode {
    private Intake intake;
    private ShooterV2 shooterV2;

    @Override
    public void init() {
        shooterV2 = new ShooterV2(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            shooterV2.setTargetRPM(1300);
        }else {
            shooterV2.setTargetRPM(0);
        }
        shooterV2.updateShooter();
        intake.intake(gamepad1.left_trigger, gamepad1.left_bumper, gamepad1.a);

        telemetry.addData("Shooter", shooterV2.getTelemetry());
        telemetry.update();
    }
}
