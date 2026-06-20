package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Auto.SUB_AUTO.subState.FINISH;
import static org.firstinspires.ftc.teamcode.Auto.SUB_AUTO.subState.IDLE;
import static org.firstinspires.ftc.teamcode.Auto.SUB_AUTO.subState.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.Auto.SUB_AUTO.subState.LAUNCH_OPEN_STOOPER;
import static org.firstinspires.ftc.teamcode.Auto.SUB_AUTO.subState.SPIN;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.teamcode.SUBSYSTEM.ShooterFeedForwardCSO;

public class SUB_AUTO {
    public DcMotorEx FrontIntake, ReversalIntake;
    private ShooterFeedForwardCSO shooter;
    private StateMachine shotsState;

    public enum subState {
        IDLE, INTAKE_IN, SPIN, LAUNCH_OPEN_STOOPER, FINISH
    }

    public subState State;

    public SUB_AUTO(HardwareMap hardwareMap) {
        FrontIntake = hardwareMap.get(DcMotorEx.class, "Intake1");
        ReversalIntake = hardwareMap.get(DcMotorEx.class, "Intake2");

        FrontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        ReversalIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter = new ShooterFeedForwardCSO(hardwareMap);
        shooter.stooper.setPosition(ShooterFeedForwardCSO.CLOSE_angle);

        State = IDLE;

        shotsState = new StateMachineBuilder().state(INTAKE_IN).onEnter(() -> {
                    FrontIntake.setPower(1);
                    ReversalIntake.setPower(1);
                }).transitionTimed(0.2)

                .state(SPIN).onEnter(() -> shooter.update(74, true, false)).transitionTimed(1, LAUNCH_OPEN_STOOPER)

                .onEnter(() -> shooter.stooper.setPosition(ShooterFeedForwardCSO.OPEN_angle)).transitionTimed(0.2, FINISH)

                .onExit(() -> {
                    shooter.flyWheel.setVelocity(0);
                    FrontIntake.setPower(0);
                    ReversalIntake.setPower(0);
                    shooter.stooper.setPosition(ShooterFeedForwardCSO.CLOSE_angle);
                })

                .state(FINISH).build();
    }


    public void updateShots() {
        shotsState.update();
    }

    public boolean isFinished() {
        return State == FINISH;
    }

    public void startShots() {
        shotsState.start();
    }

    public void getState() {
        shotsState.getState();
    }

}
