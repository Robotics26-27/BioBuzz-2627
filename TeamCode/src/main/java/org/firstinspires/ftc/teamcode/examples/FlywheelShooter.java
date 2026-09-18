package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Closed loop flywheel shooter with a feeder servo.
 *
 * The flywheel runs on velocity control, not raw power. Raw power means the wheel slows
 * down on every shot and the second artifact of a burst flies short. Velocity control
 * makes the motor fight back, and the "at speed" gate stops the feeder from firing into
 * a wheel that has not recovered yet.
 *
 * Controls:
 *   right bumper    hold to spin the flywheel up to TARGET_RPM
 *   a               fire one artifact (only works once the wheel is at speed)
 *
 * Hardware config names used here -- change them to match the Driver Station config:
 *   flywheel (DcMotorEx), feeder (Servo)
 *
 * TUNE THESE ON THE ROBOT. The PIDF gains and RPM below are placeholders, not values
 * measured off our shooter. Start by tuning F alone: set P=I=D=0, then raise F until the
 * measured RPM sits near target on its own. Only then add P.
 */
@TeleOp(name = "Example: Flywheel Shooter", group = "Examples")
public class FlywheelShooter extends OpMode {

    /** Encoder counts per output revolution. 28 is a bare goBILDA/REV motor (no gearbox). */
    private static final double TICKS_PER_REV = 28.0;

    private static final double TARGET_RPM = 3000.0;

    /** How close to target counts as ready to fire. Tighter = more consistent, slower. */
    private static final double RPM_TOLERANCE = 60.0;

    /** Velocity PIDF, in ticks/sec units. F carries the load; P only cleans up the rest. */
    private static final double VELOCITY_P = 1.5;
    private static final double VELOCITY_I = 0.1;
    private static final double VELOCITY_D = 0.0;
    private static final double VELOCITY_F = 13.0;

    private static final double FEEDER_REST = 0.0;
    private static final double FEEDER_PUSH = 0.35;
    private static final double FEEDER_PUSH_SECONDS = 0.2;
    private static final double FEEDER_RETURN_SECONDS = 0.2;

    private enum FeederState { REST, PUSHING, RETURNING }

    private DcMotorEx flywheel;
    private Servo feeder;

    private final ElapsedTime feederTimer = new ElapsedTime();
    private FeederState feederState = FeederState.REST;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        // A flywheel must coast. Braking a spun-up wheel wrecks the gearbox.
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);

        feeder = hardwareMap.get(Servo.class, "feeder");
        feeder.setPosition(FEEDER_REST);
    }

    @Override
    public void loop() {
        boolean spinUp = gamepad1.right_bumper;
        flywheel.setVelocity(spinUp ? rpmToTicksPerSecond(TARGET_RPM) : 0.0);

        double measuredRpm = ticksPerSecondToRpm(flywheel.getVelocity());
        boolean atSpeed = spinUp && Math.abs(measuredRpm - TARGET_RPM) < RPM_TOLERANCE;

        updateFeeder(atSpeed && gamepad1.a);

        telemetry.addData("Target RPM", "%.0f", TARGET_RPM);
        telemetry.addData("Measured RPM", "%.0f", measuredRpm);
        telemetry.addData("Status", !spinUp ? "idle" : atSpeed ? "READY" : "spinning up");
        telemetry.addData("Feeder", feederState);
    }

    @Override
    public void stop() {
        flywheel.setVelocity(0.0);
    }

    /**
     * One shot per press. Runs as a timed state machine so loop() never blocks -- a
     * sleep() here would freeze the drivetrain for the length of the shot.
     */
    private void updateFeeder(boolean fireRequested) {
        switch (feederState) {
            case REST:
                if (fireRequested) {
                    feeder.setPosition(FEEDER_PUSH);
                    feederTimer.reset();
                    feederState = FeederState.PUSHING;
                }
                break;
            case PUSHING:
                if (feederTimer.seconds() > FEEDER_PUSH_SECONDS) {
                    feeder.setPosition(FEEDER_REST);
                    feederTimer.reset();
                    feederState = FeederState.RETURNING;
                }
                break;
            case RETURNING:
                if (feederTimer.seconds() > FEEDER_RETURN_SECONDS) {
                    feederState = FeederState.REST;
                }
                break;
        }
    }

    private static double rpmToTicksPerSecond(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    private static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / TICKS_PER_REV;
    }
}
