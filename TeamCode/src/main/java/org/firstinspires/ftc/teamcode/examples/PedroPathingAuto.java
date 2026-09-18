package org.firstinspires.ftc.teamcode.examples;

import com.pedropathing.follower.Follower;
// Wildcards match pedroPathing/Tuning.java -- BezierLine, BezierCurve, Pose and PathChain
// are split across these two packages.
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Three leg Pedro Pathing autonomous, driven by a state machine.
 *
 * READ THIS BEFORE RUNNING. Constants.java in this repo is still the bare quickstart --
 * no drivetrain, no localizer, no tuned gains. This OpMode compiles against it but the
 * robot will not track a path until you run the Tuning OpMode and fill Constants.java in.
 * Do that first, confirm LocalizationTest reports a sane pose while you push the robot
 * around, then come back here.
 *
 * The poses below are made up. Replace them with real field coordinates for our alliance
 * and start position. Pedro's field frame is 144x144 inches with the origin at a corner,
 * and headings are RADIANS.
 *
 * The shape to copy is the state machine, not the numbers:
 *   - followPath() returns immediately, it does not wait
 *   - every loop calls follower.update() exactly once
 *   - the OpMode never blocks, so you can run an intake or shooter alongside the path
 */
@Autonomous(name = "Example: Pedro Pathing Auto", group = "Examples")
public class PedroPathingAuto extends OpMode {

    private static final Pose START_POSE = new Pose(9, 60, Math.toRadians(0));
    private static final Pose SCORE_POSE = new Pose(37, 68, Math.toRadians(0));
    private static final Pose PICKUP_CONTROL = new Pose(30, 30);
    private static final Pose PICKUP_POSE = new Pose(60, 24, Math.toRadians(-90));
    private static final Pose PARK_POSE = new Pose(60, 96, Math.toRadians(90));

    /** How long we sit still at the scoring position pretending to score. */
    private static final double SCORE_SECONDS = 1.0;

    private Follower follower;
    private PathChain toScore, toPickup, toPark;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private int state;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        buildPaths();
    }

    @Override
    public void init_loop() {
        follower.update();
        telemetry.addData("Pose", follower.getPose());
    }

    @Override
    public void start() {
        state = 0;
        stateTimer.reset();
        follower.followPath(toScore);
    }

    @Override
    public void loop() {
        follower.update();
        runStateMachine();

        telemetry.addData("State", state);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("x", "%.1f", follower.getPose().getX());
        telemetry.addData("y", "%.1f", follower.getPose().getY());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
    }

    /**
     * Advance on "path finished", never on "enough time has passed". Each case sets up
     * the next action and falls through to the next loop iteration.
     */
    private void runStateMachine() {
        switch (state) {
            case 0: // driving to the scoring position
                if (!follower.isBusy()) {
                    // Start your scoring mechanism here.
                    stateTimer.reset();
                    state = 1;
                }
                break;
            case 1: // parked at the goal, waiting on the mechanism
                if (stateTimer.seconds() > SCORE_SECONDS) {
                    follower.followPath(toPickup);
                    state = 2;
                }
                break;
            case 2: // curving out to the pickup
                if (!follower.isBusy()) {
                    follower.followPath(toPark);
                    state = 3;
                }
                break;
            case 3: // driving to park
                if (!follower.isBusy()) {
                    state = 4;
                }
                break;
            default: // done, follower holds the park position
                break;
        }
    }

    private void buildPaths() {
        // A straight line with the heading rotating smoothly from start to end.
        toScore = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SCORE_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), SCORE_POSE.getHeading())
                .build();

        // A curve. The middle pose is a control point -- the robot is pulled toward it but
        // never drives through it. Move it to reshape the arc around field obstacles.
        toPickup = follower.pathBuilder()
                .addPath(new BezierCurve(SCORE_POSE, PICKUP_CONTROL, PICKUP_POSE))
                .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), PICKUP_POSE.getHeading())
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(PICKUP_POSE, PARK_POSE))
                .setLinearHeadingInterpolation(PICKUP_POSE.getHeading(), PARK_POSE.getHeading())
                .build();
    }
}
