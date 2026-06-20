package org.firstinspires.ftc.teamcode.OpMode.AutonomousRoadrunner;

import android.graphics.HardwareRenderer;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpMode.Configuration;
import org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action.FlyWheel;
import org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action.Turret;
import org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action.subIntake;

@Autonomous(name = "NearBlue", group = "~")
public class AutoNearBlue extends LinearOpMode {

    private Configuration configuration = new Configuration();
    private MecanumDrive drive;
    private subIntake intake;
    private FlyWheel flyWheel;
    private Turret turret;

    private Pose2d BEGIN_POSE = new Pose2d(-55.3, -55.2, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, BEGIN_POSE);
        intake = new subIntake(hardwareMap);
        flyWheel = new FlyWheel(hardwareMap);
        turret = new Turret(hardwareMap);

        Action turretGoal = TelemetryPacket -> {
            drive.localizer.update();
            Pose2d position = drive.localizer.getPose();
            double x = position.position.x;
            double y = position.position.y;
            double h = position.heading.toDouble();
            turret.updateTurret(x, y, h, configuration.blueTarget);

            return true;
        };


        Action motions = drive.actionBuilder(BEGIN_POSE)
                .afterTime(0, () -> intake.IntakeIn(1))
                .strafeToLinearHeading(new Vector2d(-13.2, -13.2), Math.toRadians(-60))
                .afterTime(0, () -> flyWheel.flyWheelOn(96))
//                .waitSeconds(1)
                .stopAndAdd(flyWheel.flyWheelOff())
                .afterTime(0, () -> intake.StopIntake())

//                First Intake ball take
                .afterTime(1.5, () -> intake.IntakeIn(1))
                .splineToLinearHeading(new Pose2d(new Vector2d(11.8, -54.1), Math.toRadians(-90)), 5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(-13.2, -13.2), Math.toRadians(0)), 5)
                .waitSeconds(1)

//                Second Intake ball take
                .splineToLinearHeading(new Pose2d(new Vector2d(0.0, -62.8), Math.toRadians(-110)), 16)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(-13.2, -13.2), Math.toRadians(0)), 5)
                .waitSeconds(1)

//                Third Intake ball take
                .splineToLinearHeading(new Pose2d(new Vector2d(0.0, -62.8), Math.toRadians(-110)), 16)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(-13.2, -13.2), Math.toRadians(0)), 5)
                .waitSeconds(1)

                //                Fourth Intake ball take
                .splineToLinearHeading(new Pose2d(new Vector2d(0.0, -62.8), Math.toRadians(-110)), 16)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(-13.2, -13.2), Math.toRadians(-90)), 5)
                .waitSeconds(1)

//                //                Fifth Intake ball take
//                .splineToLinearHeading(new Pose2d(new Vector2d(0.0, -62.8), Math.toRadians(-110)), 16)
//                .waitSeconds(1)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(-13.2, -13.2), Math.toRadians(-90)), 5)
//                .waitSeconds(1)

                .strafeToConstantHeading(new Vector2d(-12.5, -53.4))
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-13.2, -13.2))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(
                turretGoal,
                new SequentialAction(motions)));

        while (opModeIsActive()) idle();
    }

}
