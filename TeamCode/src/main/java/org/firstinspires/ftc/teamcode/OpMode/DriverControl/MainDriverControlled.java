//package org.firstinspires.ftc.teamcode.OpMode.DriverControl;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.OpMode.Configuration;
//import org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action.FlyWheel;
//import org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action.Turret;
//import org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action.subIntake;
//import org.firstinspires.ftc.teamcode.PinpointLocalizer;
//
//
//@TeleOp(name = "DriveControl", group = "~")
//public class MainDriverControlled extends OpMode {
//    private  MecanumDrive drive;
//    private FlyWheel flyWheel;
//    private GoBildaPinpointDriver driver;
//    private FlyWheel.Controlled flyWheelControlled ;
//    private Configuration configuration = new Configuration();
//    private Turret turret;
//    private subIntake Intake;
//    private subIntake.controlledIntake controlledIntake;
//
////    PinpointLocalizer pp = (PinpointLocalizer) drive.localizer;
//    private final Pose2d BEGIN_POSE = new Pose2d(59.7, -9 ,90);
//
//
//    @Override
//    public void init() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        drive = new MecanumDrive(hardwareMap, BEGIN_POSE);
//        flyWheel = new FlyWheel(hardwareMap);
//        turret = new Turret(hardwareMap);
//        Intake = new subIntake(hardwareMap);
//
//        flyWheelControlled = flyWheel.new Controlled();
//        controlledIntake = Intake.new controlledIntake();
//    }
//
//    @Override
//    public void loop() {
////        ========= PARAMETERS =========
//        drive.localizer.update();
//        Pose2d position = drive.localizer.getPose();
//        double xPosition = position.position.x;
//        double yPosition = position.position.y;
//        double robotHeading = position.heading.toDouble();
//        double distanceTarget = flyWheel.getDistance(xPosition, yPosition, configuration.blueTarget);
//
////        ====== DRIVE BASE ======
//        double slowMultiply = gamepad1.right_bumper ? 0.6 : 1.0;
//        double forward = -gamepad1.left_stick_y * slowMultiply;
//        double strafe = -gamepad1.left_stick_x * slowMultiply;
//        double rotate = -gamepad1.right_stick_x * slowMultiply;
//
//        double cos = Math.cos(robotHeading);
//        double sin = Math.sin(robotHeading);
//        double rotX = forward * cos - strafe * sin;
//        double rotY = forward * sin + strafe * cos;
//        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotX, rotY), rotate));
//
//
////        ======= SHOOTER =======
//        boolean pressY = gamepad1.y;
//        boolean Stooper = gamepad1.a;
//        flyWheelControlled.flyWheel(distanceTarget, pressY, Stooper);
////        flyWheelControlled.Stooper(gamepad1.a);
////        flyWheelControlled.hoodAngle(distanceTarget);
//
////        ======= TURRET =======
//        boolean pressButton = gamepad1.b;
//        if (pressButton) {
//            turret.updateTurret(xPosition, yPosition
//                    ,Math.toDegrees(robotHeading), configuration.blueTarget);
//        } else {
//            turret.turret.setPower(0);
//        }
//
////        ====== INTAKE ======
//        double triggered = gamepad1.left_trigger;
//        boolean bumper = gamepad1.left_bumper;
//        boolean x = gamepad1.x;;
//        controlledIntake.intake(triggered, bumper, x);
//
////        double totalAMP =
////                drive.leftFront.getCurrent(CurrentUnit.AMPS) +
////                drive.leftBack.getCurrent(CurrentUnit.AMPS) +
////                drive.rightFront.getCurrent(CurrentUnit.AMPS) +
////                drive.rightBack.getCurrent(CurrentUnit.AMPS) +
////                flyWheel.flyWheel.getCurrent(CurrentUnit.AMPS) +
////                Intake.FrontIntake.getCurrent(CurrentUnit.AMPS) +
////                Intake.ReversalIntake.getCurrent(CurrentUnit.AMPS) +
////                turret.turret.getCurrent(CurrentUnit.AMPS);
//
////        ====== TELEMETRY ======
////        telemetry.addData("TOTAL AMP", totalAMP);
//        telemetry.addData("Robot X", xPosition);
//        telemetry.addData("Robot Y", yPosition);
////        telemetry.addData("RAW X", pp.driver.getPosX(DistanceUnit.INCH));
////        telemetry.addData("RAW Y", pp.driver.getPosY(DistanceUnit.INCH));
//        telemetry.addData("Robot Heading", robotHeading);
//        telemetry.addLine("DRIVE BASE");
//        telemetry.addData("FRONT LEFT AMP", drive.leftFront.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("BACK LEFT AMP", drive.leftBack.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("FRONT RIGHT AMP", drive.rightFront.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("BACK RIGHT AMP", drive.rightBack.getCurrent(CurrentUnit.AMPS));
////
////        telemetry.addLine("SHOOTER");
////        telemetry.addData("FLYWHEEL AMP", flyWheel.flyWheel.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("FLYWHEEL VELOCITY", flyWheel.flyWheel.getVelocity());
//        telemetry.addData("HOOD ANGLE", flyWheel.hoodAngle.getPosition());
//        telemetry.addData("TURRET TICK", turret.turret.getCurrentPosition());
//        telemetry.addData("TURRET ERROR", turret.getTurretError());
////
////        telemetry.addLine("INTAKE");
////        telemetry.addData("FRONT INTAKE AMP", Intake.FrontIntake.getCurrent(CurrentUnit.AMPS));
////        telemetry.addData("REVERSAL INTAKE AMP", Intake.ReversalIntake.getCurrent(CurrentUnit.AMPS));
////
////        telemetry.addLine("TURRET");
////        telemetry.addData("TURRET AMP", turret.turret.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Trigger", triggered);
//        telemetry.update();
//    }
//}
