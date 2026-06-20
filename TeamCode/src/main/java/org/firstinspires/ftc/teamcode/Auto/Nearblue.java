package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.CheckResult;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Nearblue extends OpMode {
    private TelemetryManager telemetryManager;
    private Follower follower;
    private SUB_AUTO helper;
    private Timer pathTimer, opModeTimer;

    private enum PathState {
        DRIVE_START_SHOOT_POS,
        PRELOAD_1,
        FIRST_BALL_INTAKE,
        PRELOAD_2,
        FIRST_BALL_RAMP,
        PRELOAD_3,
        FINISH
    }
    PathState pathState;

    private final Pose START_POSE = new Pose(20.292617449664437, 120.82013422818791, Math.toRadians(140));
    private final Pose SHOOT_POSE = new Pose(44.860742875464936, 95.93367224021351, Math.toRadians(180));
    private final Pose PICKUP_SPIKE_1 = new Pose(13.642463087248322, 58.064, Math.toRadians(180));
    private final Pose PICKUP_RAMP_1 = new Pose(44.860742875464936, 95.93367224021351, Math.toRadians(140));
    private final Pose END_OF_POSE = new Pose(68.23377187301062, 104.23360206926517, Math.toRadians(230));

    private PathChain driveStartPosShootPos, PICKUP_FIRST_SIX_BALL, SHOOT_SIX_BALL,  PICKUP_NINE_BALL, SHOOT_NINE_BALL, END;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POSE.getHeading())
                .build();
        PICKUP_FIRST_SIX_BALL = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, PICKUP_SPIKE_1))
                .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), PICKUP_SPIKE_1.getHeading())
                .build();
        SHOOT_SIX_BALL = follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_SPIKE_1, SHOOT_POSE))
                .setLinearHeadingInterpolation(PICKUP_SPIKE_1.getHeading(), SHOOT_POSE.getHeading())
                .build();

        PICKUP_NINE_BALL = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, PICKUP_RAMP_1))
                .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), PICKUP_RAMP_1.getHeading())
                .build();
        SHOOT_NINE_BALL = follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_RAMP_1, SHOOT_POSE))
                .setLinearHeadingInterpolation(PICKUP_RAMP_1.getHeading(), SHOOT_POSE.getHeading())
                .build();
    }

    public void pathStateUpdate() {
        switch (pathState) {
            case DRIVE_START_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.DRIVE_START_SHOOT_POS);
                break;

            case PRELOAD_1:
                if (!follower.isBusy()) {
                    helper.startShots();
                    if (helper.isFinished()) {
                        setPathState(PathState.FIRST_BALL_INTAKE);
                    }
                }
                break;

            case FIRST_BALL_INTAKE:
                follower.followPath(PICKUP_FIRST_SIX_BALL);
                helper.FrontIntake.setPower(1);
                helper.ReversalIntake.setPower(1);
                if(!follower.isBusy()) {
                    helper.FrontIntake.setPower(0);
                    helper.ReversalIntake.setPower(0);
                    setPathState(PathState.PRELOAD_2);
                }
                break;

            case PRELOAD_2:
                if (!follower.isBusy()) {
                    helper.startShots();
                    if (helper.isFinished()) {
                        setPathState(PathState.FIRST_BALL_RAMP);
                    }
                }
                break;

            case FIRST_BALL_RAMP:
                follower.followPath(PICKUP_NINE_BALL);
                helper.FrontIntake.setPower(1);
                helper.ReversalIntake.setPower(1);
                if(!follower.isBusy()) {
                    helper.FrontIntake.setPower(0);
                    helper.ReversalIntake.setPower(0);
                    setPathState(PathState.PRELOAD_3);
                }
                break;

            case PRELOAD_3:
                if (!follower.isBusy()) {
                    helper.startShots();
                    if (helper.isFinished()) {
                        setPathState(PathState.FINISH);
                    }
                }
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        helper = new SUB_AUTO(hardwareMap);

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower.setStartingPose(START_POSE);
        buildPaths();

        pathState = PathState.DRIVE_START_SHOOT_POS;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.DRIVE_START_SHOOT_POS);
    }

    @Override
    public void loop() {
        follower.update();
        helper.updateShots();
        pathStateUpdate();

        telemetryManager.debug("path state: " + pathState);
        telemetryManager.debug("x: " + follower.getPose().getX());
        telemetryManager.debug("y: " + follower.getPose().getY());
        telemetryManager.debug("heading: " + follower.getPose().getHeading());
        telemetryManager.update();
    }
}
