package org.firstinspires.ftc.teamcode.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AprilTag tracking: drive manually, then hold a button to let the robot line itself up
 * on a tag.
 *
 * Driving goes through the Pedro follower rather than a hand written mecanum mix, so this
 * file only has to decide how fast to go on three axes. Alignment is robot centric -- the
 * camera measures the tag relative to itself, not to the field -- so setTeleOpDrive gets
 * true for robotCentric in both modes.
 *
 * REQUIRES A TUNED Constants.java, same as FieldCentricTeleOp.
 *
 * DECODE tag IDs:
 *   20  blue goal
 *   24  red goal
 *   21, 22, 23  obelisk motif tags -- these tell you the randomised pattern, they are
 *               not alignment targets, so do not aim at them
 *
 * Controls:
 *   left stick / right stick x   manual drive
 *   left bumper                  hold to auto align on TARGET_TAG_ID
 *
 * The three gains are a P controller per axis. Tune one at a time, and tune them low:
 * too high and the robot oscillates around the tag instead of settling on it.
 */
@TeleOp(name = "Example: AprilTag Tracking", group = "Examples")
public class AprilTagTracking extends OpMode {

    /** Which tag to chase. Set to -1 to lock onto whichever tag is visible. */
    private static final int TARGET_TAG_ID = 20;

    /** How far from the tag we want to end up, in inches. */
    private static final double DESIRED_RANGE = 24.0;

    private static final double RANGE_GAIN = 0.02;    // power per inch of range error
    private static final double STRAFE_GAIN = 0.015;  // power per degree of yaw error
    private static final double TURN_GAIN = 0.01;     // power per degree of bearing error

    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double MAX_AUTO_STRAFE = 0.5;
    private static final double MAX_AUTO_TURN = 0.3;

    private Follower follower;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());

        aprilTag = new AprilTagProcessor.Builder().build();
        // Decimation trades detection range against frame rate. 2 is a reasonable middle
        // for tracking a goal from across the field; drop to 1 if it loses the tag far out.
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Camera starting. Wait for a detection before pressing start.");
    }

    @Override
    public void init_loop() {
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        AprilTagDetection target = findTarget();
        boolean aligning = target != null && gamepad1.left_bumper;

        if (aligning) {
            // ftcPose is the tag relative to the camera:
            //   range   straight line distance, inches
            //   bearing left/right angle off the camera centre, degrees -- turn to zero it
            //   yaw     how far off square we are to the tag face, degrees -- strafe to zero it
            // These signs are the SDK sample RobotAutoDriveToAprilTagOmni's verbatim. Its
            // axes match Pedro's: both drive off -left_stick_y, -left_stick_x, -right_stick_x,
            // so lateral is positive-left and heading is positive-counter-clockwise in both.
            follower.setTeleOpDrive(
                    clip((target.ftcPose.range - DESIRED_RANGE) * RANGE_GAIN, MAX_AUTO_SPEED),
                    clip(-target.ftcPose.yaw * STRAFE_GAIN, MAX_AUTO_STRAFE),
                    clip(target.ftcPose.bearing * TURN_GAIN, MAX_AUTO_TURN),
                    true);
        } else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true);
        }
        follower.update();

        telemetry.addLine(aligning ? "AUTO ALIGN"
                : target == null ? "No target -- drive manually until the tag is in frame"
                : "Target found -- hold left bumper to align");
        if (target != null) {
            telemetry.addData("Tag", target.id);
            telemetry.addData("range / bearing / yaw", "%.1f in  %.1f deg  %.1f deg",
                    target.ftcPose.range, target.ftcPose.bearing, target.ftcPose.yaw);
        }
    }

    @Override
    public void stop() {
        visionPortal.close();
    }

    /**
     * First detection matching TARGET_TAG_ID that also has pose data. Detections without
     * metadata have a null ftcPose -- reading it would throw, so they are skipped.
     */
    private AprilTagDetection findTarget() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata == null) {
                continue;
            }
            if (TARGET_TAG_ID < 0 || detection.id == TARGET_TAG_ID) {
                return detection;
            }
        }
        return null;
    }

    private static double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
