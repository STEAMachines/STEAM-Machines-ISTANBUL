package org.firstinspires.ftc.teamcode.OpMode.DriverControl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpMode.Configuration;
import org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action.FlyWheel;
import org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action.Turret;
import org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action.subIntake;


@TeleOp(name = "DriveControl", group = "~")
public class MainDriverControlled extends OpMode {
    private  MecanumDrive drive;
    private FlyWheel flyWheel;
    private FlyWheel.Controlled flyWheelControlled;
    private Configuration configuration;
    private Turret turret;
    private subIntake Intake;
    private subIntake.controlledIntake controlledIntake;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0 ,0));
        flyWheel = new FlyWheel(hardwareMap);
        turret = new Turret(hardwareMap);
        Intake = new subIntake(hardwareMap);
    }

    @Override
    public void loop() {
//        ========= PARAMETERS =========
        drive.localizer.update();
        Pose2d position = drive.localizer.getPose();
        double xPosition = position.position.x;
        double yPosition = position.position.y;
        double robotHeading = position.heading.toDouble();
        double distanceTarget = flyWheel.getDistance(xPosition, yPosition, configuration.blueTarget);

//        ====== DRIVE BASE ======
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x * 0.6;
        double rotate = gamepad1.right_stick_x;
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, strafe), rotate));


//        ======= SHOOTER =======
        boolean pressY = gamepad1.y;
        flyWheelControlled.flyWheel(distanceTarget, pressY);
        flyWheelControlled.hoodAngle(distanceTarget);

//        ======= TURRET =======
        boolean pressButton = gamepad1.b;
        if (pressButton) {
            turret.updateTurret(xPosition, yPosition, robotHeading, configuration.blueTarget);
        }

//        ====== INTAKE ======
        double triggered = gamepad1.left_trigger;
        boolean bumper = gamepad1.left_bumper;
        controlledIntake.IntakeIn(triggered);
        controlledIntake.IntakeOut(bumper);

        double totalAMP = drive.leftFront.getCurrent(CurrentUnit.AMPS) +
                drive.leftBack.getCurrent(CurrentUnit.AMPS) +
                drive.rightFront.getCurrent(CurrentUnit.AMPS) +
                drive.rightFront.getCurrent(CurrentUnit.AMPS) +
                drive.rightBack.getCurrent(CurrentUnit.AMPS) +
                flyWheel.flyWheel.getCurrent(CurrentUnit.AMPS) +
                Intake.FrontIntake.getCurrent(CurrentUnit.AMPS) +
                Intake.ReversalIntake.getCurrent(CurrentUnit.AMPS) +
                turret.turret.getCurrent(CurrentUnit.AMPS);

//        ====== TELEMETRY ======
        telemetry.addData("TOTAL AMP", totalAMP);
        telemetry.addLine("DRIVE BASE");
        telemetry.addData("FRONT LEFT AMP", drive.leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BACK LEFT AMP", drive.leftBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("FRONT RIGHT AMP", drive.rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BACK RIGHT AMP", drive.rightBack.getCurrent(CurrentUnit.AMPS));

        telemetry.addLine("SHOOTER");
        telemetry.addData("FLYWHEEL AMP", flyWheel.flyWheel.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("FLYWHEEL VELOCITY", flyWheel.flyWheel.getVelocity());
        telemetry.addData("HOOD ANGLE", flyWheel.hoodAngle.getPosition());

        telemetry.addLine("INTAKE");
        telemetry.addData("FRONT INTAKE AMP", Intake.FrontIntake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("REVERSAL INTAKE AMP", Intake.ReversalIntake.getCurrent(CurrentUnit.AMPS));

        telemetry.addLine("TURRET");
        telemetry.addData("TURRET AMP", turret.turret.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}
