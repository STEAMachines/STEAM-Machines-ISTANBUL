package org.firstinspires.ftc.teamcode.SUBSYSTEM;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;


@Configurable
public class Intake {
    public DcMotorEx FrontIntake, ReversalIntake;
    public Servo IntakeRight, IntakeLeft;
    public static double UP_ANGLE;
    public static double DOWN_ANGLE;

    private enum IntakeState {
        INTAKING, REVERSING, STOOP
    }

    public IntakeState state;

    private StateMachine intakeState;

    public Intake(HardwareMap hardwareMap) {
        FrontIntake = hardwareMap.get(DcMotorEx.class, "Intake1");
        ReversalIntake = hardwareMap.get(DcMotorEx.class, "Intake2");

        FrontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        ReversalIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        IntakeRight = hardwareMap.get(Servo.class, "IntakeRight");
        IntakeLeft = hardwareMap.get(Servo.class, "IntakeLeft");

        IntakeLeft.setDirection(Servo.Direction.REVERSE);

        IntakeLeft.setPosition(UP_ANGLE);
        IntakeRight.setPosition(UP_ANGLE);

        state = IntakeState.STOOP;
    }

    private StateMachine intakeState(double IntakeTrigger, boolean Reversing) {
        intakeState = new StateMachineBuilder()
                .state(IntakeState.STOOP)
                .onEnter(() -> {
                    FrontIntake.setPower(0);
                    ReversalIntake.setPower(0);
                    IntakeLeft.setPosition(UP_ANGLE);
                    IntakeRight.setPosition(UP_ANGLE);
                })
                .transition(() -> IntakeTrigger > 0.1, IntakeState.INTAKING)
                .transition(() -> Reversing, IntakeState.REVERSING)


                .state(IntakeState.INTAKING)
                .onEnter(() -> {
                    FrontIntake.setPower(1);
                    ReversalIntake.setPower(1);
                    IntakeLeft.setPosition(DOWN_ANGLE);
                    IntakeRight.setPosition(DOWN_ANGLE);
                })

                .transition(() -> IntakeTrigger <= 0.1, IntakeState.STOOP)

                .state(IntakeState.REVERSING)
                .onEnter(() -> {
                    FrontIntake.setPower(-1);
                    ReversalIntake.setPower(-1);
                    IntakeLeft.setPosition(UP_ANGLE);
                    IntakeRight.setPosition(UP_ANGLE);
                })

                .transition(() -> !Reversing, IntakeState.STOOP)

                .build();
        intakeState.start();
        return intakeState;
    }

    public void intake(double IntakeTrigger, boolean bumper) {
        intakeState.update();
    }

//    public void intake(double trigger, boolean bumper, boolean additional) {
//        if (trigger > 0.1) {
//            FrontIntake.setPower(1);
//            ReversalIntake.setPower(1);
//
//            IntakeRight.setPosition(DOWN_ANGLE);
//            IntakeLeft.setPosition(DOWN_ANGLE);
//        } else if (bumper) {
//            FrontIntake.setPower(-1);
//            ReversalIntake.setPower(-1);
//
//            IntakeRight.setPosition(UP_ANGLE);
//            IntakeLeft.setPosition(UP_ANGLE);
//        } else {
//            FrontIntake.setPower(0);
//            ReversalIntake.setPower(0);
//
//            IntakeRight.setPosition(UP_ANGLE);
//            IntakeLeft.setPosition(UP_ANGLE);
//        }
//        if (additional) {
//            FrontIntake.setPower(1);
//        }
//    }

}
