package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action.FlyWheel;
import org.firstinspires.ftc.teamcode.OpMode.Subsytem_Action.subIntake;

@TeleOp
public class LatihanDriver extends OpMode {
    IMU imu;
    private subIntake Intake;
    private subIntake.controlledIntake controlledIntake;
    private FlyWheel flyWheel;
    private FlyWheel.Controlled flyWheelControlled ;

    DcMotor leftFront, leftBack, rightBack, rightFront;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "rearRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        flyWheel = new FlyWheel(hardwareMap);
        Intake = new subIntake(hardwareMap);

        controlledIntake = Intake.new controlledIntake();
        flyWheelControlled = flyWheel.new Controlled();

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(imuParams);
        imu.resetYaw();
    }

    @Override
    public void loop() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * 1.1;

        double clamp = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        leftFront.setPower((y + x + rx) / clamp);
        leftBack.setPower((y - x + rx) / clamp);
        rightFront.setPower((y - x - rx) / clamp);
        rightBack.setPower((y + x - rx) / clamp);

        boolean pressY = gamepad1.y;
        boolean Stooper = gamepad1.a;
        flyWheelControlled.flyWheel(90, pressY, Stooper);
//        flyWheelControlled.Stooper(gamepad1.a);
//        flyWheelControlled.hoodAngle(distanceTarget);

//        ======= TURRET =======
        boolean pressButton = gamepad1.b;
//        ====== INTAKE ======
        double triggered = gamepad1.left_trigger;
        boolean bumper = gamepad1.left_bumper;

        boolean front = gamepad1.x;;
        controlledIntake.intake(triggered, bumper, front);

        telemetry.addData("AMPER", flyWheel.flyWheel.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("TARGET", flyWheelControlled.targetVelocity);
        telemetry.addData("CURRENT VEL", flyWheel.flyWheel.getVelocity());
        telemetry.update();
    }
}
