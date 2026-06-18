package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import kotlin.properties.ObservableProperty;

@TeleOp
public class testDrivebase extends OpMode {
    private MecanumDrive drive;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0 ,0));
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(1.0, 0), 0));
        } else {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }
        if (gamepad1.dpad_down) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-1.0, 0), 0));
        } else {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }

        telemetry.addLine("DRIVE BASE");
        telemetry.addData("FRONT LEFT AMP", drive.leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BACK LEFT AMP", drive.leftBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("FRONT RIGHT AMP", drive.rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BACK RIGHT AMP", drive.rightBack.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}
