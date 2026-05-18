    package org.firstinspires.ftc.teamcode.DECODE.V2.DriverControl;

    import com.acmerobotics.dashboard.config.Config;
    import com.acmerobotics.roadrunner.Pose2d;
    import com.acmerobotics.roadrunner.PoseVelocity2d;
    import com.acmerobotics.roadrunner.Vector2d;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.firstinspires.ftc.teamcode.DECODE.V2.Configurations.PIDControl;
    import org.firstinspires.ftc.teamcode.DECODE.V2.Configurations.PosesStorage;
    import org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem.Intake;
    import org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem.ShooterSubs;
    import org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem.Servos;
    import org.firstinspires.ftc.teamcode.DECODE.V2.SubSystem.TurretSub;
    import org.firstinspires.ftc.teamcode.DECODE.V2.Configurations.robotConfiguration;
    import org.firstinspires.ftc.teamcode.MecanumDrive;

    @Config
    @TeleOp(name = "V2 Mecanum Istanbul")
    public class robotTeleopIstanbul extends OpMode {
//        Imports
        private  MecanumDrive drive;
        private PIDControl pidControl;
        private Servos servos;
        private TurretSub turret;
        private ShooterSubs shooter;
        private Intake intake;

//        Configuration
        private final double SNAP_90 = Math.toRadians(90), SNAP_0 = Math.toRadians(0);
        private Pose2d snapTarget = robotConfiguration.blueAimingTarget;
        private final Pose2d StartPose = PosesStorage.currentPose;
        private boolean isBlueAiming = true, isRedAiming = false;
        private boolean autoheading = false;
        private double targetAngle;

        private boolean lastY = false;
        private boolean isflyWheelOn = false;

        @Override
        public void init() {
            drive = new MecanumDrive(hardwareMap, StartPose);
            servos = new Servos(hardwareMap);
            turret = new TurretSub(hardwareMap);
            shooter = new ShooterSubs(hardwareMap);
            intake = new Intake(hardwareMap);
            pidControl = new PIDControl(robotConfiguration.HeadingKp, robotConfiguration.HeadingKi, robotConfiguration.HeadingKd);
        }

        @Override
        public void init_loop() {
            telemetry.addLine("BLUE ALLIANCE (DPAD_LEFT) | RED ALLIANCE (DPAD_RIGHT)");
            if (gamepad1.dpad_left) {
                snapTarget = robotConfiguration.blueAimingTarget;
                isBlueAiming = true;
                isRedAiming = false;
                telemetry.addData("Alliance", "BLUE");
            }
            if (gamepad1.dpad_right) {
                snapTarget = robotConfiguration.redAimingTarget;
                isRedAiming = true;
                isBlueAiming = false;
                telemetry.addData("Alliance", "RED");
            }
            telemetry.addData("Selected", isBlueAiming ?"BLUE" : "RED");
            telemetry.update();
        }

        @Override
        public void loop() {
//            Odometry
            drive.localizer.update();
            Pose2d pos = drive.localizer.getPose();
            PoseVelocity2d vel = drive.updatePoseEstimate();
            double xPose = pos.position.x;
            double yPose = pos.position.y;
            double hPose = pos.heading.toDouble();
            double xVel = vel.linearVel.x;
            double yVel = vel.linearVel.y;
            double disTarget = turret.getDistanceTarget(xPose, yPose, hPose, snapTarget);

//            Drivebase

            double slowMultiply = gamepad1.left_bumper ? 0.6 : 1.0;
            double forward = -gamepad1.left_stick_y * slowMultiply;
            double strafe = -gamepad1.left_stick_x * slowMultiply;
            double rotate = -gamepad1.right_stick_x * slowMultiply;

            double cos = Math.cos(hPose);
            double sin = Math.sin(hPose);
            double rotX = forward * cos - strafe * sin;
            double rotY = forward * sin + strafe * cos;

            if (gamepad1.dpad_up) {targetAngle = SNAP_0; autoheading = true;}
            else if (gamepad1.dpad_down) { targetAngle = SNAP_90; autoheading = true;}
            if (gamepad1.right_bumper) {targetAngle = Math.atan2(snapTarget.position.y - yPose, snapTarget.position.x - xPose);autoheading = true;}

            if (autoheading) {
                double error = targetAngle - hPose;
                error = pidControl.angleWrapRadians(error);

                if (Math.abs(error) < Math.toRadians(1)) {
                    autoheading = false;
                }
                rotate = pidControl.calculateRadians(targetAngle, hPose);
            }
            if (Math.abs(-gamepad1.right_stick_x) > 0.5) {autoheading = false;}
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotX, rotY), rotate));

//            SubSystem (Servos)
            if (gamepad1.a) { servos.setGate("open"); } else { servos.setGate("close"); }
            servos.setHoodAngle(disTarget);

//            SubSystem (Turret && Shooter)
            turret.aimingTurret(snapTarget, xPose, yPose, hPose, xVel, yVel);
            if (gamepad1.y && !lastY && turret.isAimed()) { isflyWheelOn = !isflyWheelOn; }

            if (isflyWheelOn) { shooter.setFlyWheel(disTarget, "spins"); }
            else { shooter.setFlyWheel(0, "stop"); }
            lastY = gamepad1.y;

//            SubSytem (INTAKE)
            double glt = gamepad1.left_trigger;
            double grt = gamepad1.right_trigger;
            boolean isReadyToShoot = servos.isOpened();
            intake.intake(glt, grt, isReadyToShoot, telemetry);

//           ================= TELEMETRY =================
            telemetry.addLine("ROBOT POSITION");
            telemetry.addData("START POSE", StartPose);
            telemetry.addData("xPose", xPose);
            telemetry.addData("yPose", yPose);
            telemetry.addData("hPose", Math.toDegrees(hPose));

            telemetry.addLine("SUBSYSTEM");
            telemetry.addData("SNAPPED TARGET", isBlueAiming ? "BLUE ALLIANCE" : "RED ALLIANCE");
            telemetry.addData("IS AIMED", turret.isAimed() ? "AIMED" : "ISN'T");
            telemetry.addData("DISTANCE TARGET", disTarget);
            telemetry.addData("GATE STATUS", servos.isOpened() ? "OPEN" : "CLOSE");
            telemetry.addData("INTAKE STATUS", intake.currentIntake);
            telemetry.addData("FLYWHEEL STATUS", shooter.currentflyWheelState);
            telemetry.addData("GATE STATUS", servos.currentGate);
            telemetry.update();
        }
    }
