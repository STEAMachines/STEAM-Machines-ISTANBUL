package org.firstinspires.ftc.teamcode.DECODE.V2.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DECODE.V2.Configurations.PosesStorage;
import org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem.Servos;
import org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem.ShooterSubs;
import org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem.TurretSub;
import org.firstinspires.ftc.teamcode.DECODE.V2.Configurations.robotConfiguration;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class BlueNear extends LinearOpMode {
    private MecanumDrive drive;
    private TurretSub turret;
    private ShooterSubs shooter;
    private Intake intake;
    private Servos servos;
    private final Pose2d StartPose = (new Pose2d(-55.3, -55.2, -90));
    Pose2d snapTarget = robotConfiguration.blueAimingTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, StartPose);
        turret = new TurretSub(hardwareMap);
        shooter = new ShooterSubs(hardwareMap);
        intake =  new Intake(hardwareMap);
        servos = new Servos(hardwareMap);

        Action turretTrack = TelemetryPacket -> {
            drive.localizer.update();
            Pose2d pos = drive.localizer.getPose();
            PoseVelocity2d vel = drive.updatePoseEstimate();
            double xPose = pos.position.x;
            double yPose = pos.position.y;
            double hPose = pos.heading.toDouble();
            double xVel = vel.linearVel.x;
            double yVel = vel.linearVel.y;

            turret.aimingTurret(snapTarget, xPose, yPose, hPose, xVel, yVel);
            return true;
        };

        Action shoot = TelemetryPacket -> {
            drive.localizer.update();
            Pose2d pos = drive.localizer.getPose();
            double xPose = pos.position.x;
            double yPose = pos.position.y;
            double hPose = pos.heading.toDouble();
            double disTarget = turret.getDistanceTarget(xPose, yPose, hPose, snapTarget);

            shooter.setFlyWheel(disTarget, "spins");
            servos.setHoodAngle(disTarget);
            return true;
        };

        Action intakeAc = TelemetryPacket -> {
            intake.normalIntake();
            return false;
        };

        Action gate = TelemetryPacket -> {
            if (!turret.isAimed()) {
                return false;
            }

            return servos.autoGate();
        };

        Action motion = drive.actionBuilder(StartPose)
                .strafeToLinearHeading(new Vector2d(-41.4, -40.7), Math.toRadians(-20))
                .stopAndAdd(gate)

                .strafeToConstantHeading(new Vector2d(-11.5, -47.2))
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-41.4, -40.7))
                .stopAndAdd(gate)

                .strafeToLinearHeading(new Vector2d(17.1, -46.0), Math.toRadians(5))
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-23.1, -23.1))
                .stopAndAdd(gate)

                .strafeToConstantHeading(new Vector2d(-0.2, -34.5))
                .splineToLinearHeading(new Pose2d(new Vector2d(5.8, -59.6), 10), 10)
                .waitSeconds(2)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-16.0, -16.4), 11)
                .stopAndAdd(gate)

                .strafeToConstantHeading(new Vector2d(-0.2, -34.5))
                .splineToLinearHeading(new Pose2d(new Vector2d(5.8, -59.6), 10), 10)
                .waitSeconds(2)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-16.0, -16.4), 11)
                .stopAndAdd(gate)

                .strafeToLinearHeading(new Vector2d(37.5, -47.4), Math.toRadians(-15))
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-24.5, -9.2))
                .afterTime(0.6, gate)
                .build();

        waitForStart();
        if (isStopRequested()) return;
        PosesStorage.currentPose = drive.localizer.getPose();
        Actions.runBlocking(new ParallelAction(turretTrack ,shoot, intakeAc, new SequentialAction(motion)));
    }
}
