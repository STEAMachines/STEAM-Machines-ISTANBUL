package org.firstinspires.ftc.teamcode.OpMode.AutonomousRoadrunner;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action.subIntake;

@Autonomous(name = "NearBlue", group = "~")
public class AutoNearBlue extends LinearOpMode {
    private MecanumDrive drive;
    private subIntake intake;

    private Pose2d BEGIN_POSE = new Pose2d(-55.3, -55.2, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, BEGIN_POSE);
        intake = new subIntake(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(motions()));

        while (opModeIsActive()) idle();
    }

    private Action motions() {
        return drive.actionBuilder(BEGIN_POSE)
                .strafeToLinearHeading(new Vector2d(-13.2, -13.2), Math.toRadians(-60))
                .waitSeconds(1)

//                First Intake ball take
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
    }
}
