package org.firstinspires.ftc.teamcode.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Field centric mecanum drive, using the Pedro follower.
 *
 * Paired with FieldCentricTeleOpImu, which does the same job on raw motors and the IMU.
 * Use that one if the follower is not tuned yet; use this one otherwise.
 *
 * Push the left stick away from you and the robot drives away from you, no matter which
 * way it is pointing. Right stick turns.
 *
 * The follower already knows the robot's heading from the localizer and already owns the
 * mecanum math, so field centric is one boolean: the last argument to setTeleOpDrive is
 * robotCentric, and passing false means field centric. There is no IMU read and no wheel
 * mixing in this file on purpose -- doing it by hand is how sign bugs get in.
 *
 * REQUIRES A TUNED Constants.java. Constants.java in this repo is still the bare
 * quickstart, so this OpMode will not drive until you run the Tuning OpMode and fill it
 * in. Confirm LocalizationTest reports a sane pose while you push the robot around first.
 *
 * Controls:
 *   left stick      translate (field relative)
 *   right stick x   turn
 *   left bumper     hold for robot centric driving
 *   right trigger   hold for slow mode
 *   options         re-zero the heading (point robot away from driver first)
 */
@TeleOp(name = "Example: Field Centric (Pedro)", group = "Examples")
public class FieldCentricTeleOpPedro extends OpMode {

    /** Multiplier applied to every axis while the slow mode trigger is held. */
    private static final double SLOW_MODE_SCALE = 0.35;

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        // Field forward is measured from this pose. In a real match, set it to where the
        // robot actually starts so teleop and auto share one coordinate frame.
        follower.setStartingPose(new Pose());
    }

    @Override
    public void init_loop() {
        follower.update();
        telemetry.addData("Pose", follower.getPose());
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        if (gamepad1.optionsWasPressed()) {
            resetHeading();
        }

        double scale = gamepad1.right_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;

        // Stick signs match pedroPathing/Tuning.java:183, which is the convention this
        // follower is tuned against. Last arg is robotCentric, so held bumper = robot centric.
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * scale,
                -gamepad1.left_stick_x * scale,
                -gamepad1.right_stick_x * scale,
                gamepad1.left_bumper);
        follower.update();

        telemetry.addData("Mode", gamepad1.left_bumper ? "ROBOT centric" : "FIELD centric");
        telemetry.addData("x", "%.1f", follower.getPose().getX());
        telemetry.addData("y", "%.1f", follower.getPose().getY());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
    }

    /**
     * Re-zero the heading the field centric frame is measured from, keeping x and y where
     * they are. Use it if the localizer has drifted and "away from the driver" has stopped
     * meaning away from the driver.
     *
     * VERIFY THIS ON THE ROBOT. setStartingPose is the only pose setter this repo proves
     * exists (pedroPathing/Tuning.java uses it), but every use there is at init, not
     * mid-match. Drive, press options, and check the heading telemetry snaps to 0 while x
     * and y hold. If it does not, Pedro 2.x also has a setTeleOpDrive overload taking an
     * offsetHeading, which is the cleaner fix -- I could not confirm it exists in 2.1.2.
     */
    private void resetHeading() {
        Pose current = follower.getPose();
        follower.setStartingPose(new Pose(current.getX(), current.getY(), 0));
    }
}
