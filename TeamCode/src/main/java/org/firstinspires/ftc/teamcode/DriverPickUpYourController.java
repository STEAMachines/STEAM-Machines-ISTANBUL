package org.firstinspires.ftc.teamcode;

import android.content.pm.LauncherApps;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SUBSYSTEM.Intake;
import org.firstinspires.ftc.teamcode.SUBSYSTEM.ShooterFeedForwardCSO;
import org.firstinspires.ftc.teamcode.SUBSYSTEM.Turret;
import org.firstinspires.ftc.teamcode.SUBSYSTEM.flyWheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Poses;

@TeleOp
public class DriverPickUpYourController extends OpMode {
    TelemetryManager telemetryManager;
    private Follower follower;
    private Turret turret;
    private flyWheel flyWheel;

    private Intake Intake;
    private PIDControl pidControl;

    private ShooterFeedForwardCSO shooter;

    private Poses poses = new Poses();
    boolean autoheading = false;
    double targetAngle;
    Pose blue = poses.BLUE_TARGET;

    private final Pose STARTING_POSE = new Pose(20.293, 120.820, 140);

    @Override
    public void init() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(STARTING_POSE);
        follower.update();

        turret = new Turret(hardwareMap);
        flyWheel = new flyWheel(hardwareMap);
        Intake = new Intake(hardwareMap);
        shooter = new ShooterFeedForwardCSO(hardwareMap);
        pidControl = new PIDControl(2.0, 0, 0.0008);
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryManager.update();
        Pose pos = follower.getPose();
        double robotX = pos.getX();
        double robotY = pos.getY();
        double robotHeading = pos.getHeading();

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x * 1.1;
        double rotate = -gamepad1.right_stick_x;

        if (gamepad1.right_bumper) {
            targetAngle = Math.atan2(blue.getY() - robotY, blue.getX() - robotX);
            autoheading = true;
        }

        if (autoheading) {
            double error = targetAngle - robotHeading;
            error = pidControl.angleWrapRadians(error);

            if (Math.abs(error) < Math.toRadians(1)) {
                autoheading = false;
            }
            rotate = pidControl.calculateRadians(targetAngle, robotHeading);
        }
        if (Math.abs(-gamepad1.right_stick_x) > 0.5) {
            autoheading = false;
        }

        follower.setTeleOpDrive(forward, strafe, rotate, true);

        double triggered = gamepad1.left_trigger;
        boolean bumper = gamepad1.left_bumper;

        boolean front = gamepad1.x;
        ;
        Intake.intake(triggered, bumper, front);

        if (gamepad1.b) {
            turret.updateTurret(robotX, robotY, robotHeading, poses.BLUE_TARGET);
        } else {
            turret.turret.setPower(0);
        }
        turret.homePos(gamepad1.options);

        boolean pressY = gamepad1.y;
        boolean Stooper = gamepad1.a;
        double distanceTarget = flyWheel.getDistance(robotX, robotY, poses.BLUE_TARGET);
//        flyWheel.flyWheel(distanceTarget, pressY, Stooper);
        shooter.update(distanceTarget, pressY, Stooper);

        telemetryManager.addData("Robot X", robotX);
        telemetryManager.addData("Robot Y", robotY);
        telemetryManager.addData("Heading", robotHeading);
        telemetryManager.addData("DISTANCE", distanceTarget);
        telemetryManager.addData("CURRENT VEL", flyWheel.flyWheel.getVelocity());
        telemetryManager.addData("TARGET VEL", org.firstinspires.ftc.teamcode.SUBSYSTEM.flyWheel.flyWheelVel);

        telemetryManager.update();
    }
}
