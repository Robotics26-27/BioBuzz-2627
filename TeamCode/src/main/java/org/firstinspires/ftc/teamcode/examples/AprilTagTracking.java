package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AprilTag tracking: drive manually, then hold a button to let the robot line itself up
 * on a tag.
 *
 * DECODE tag IDs:
 *   20  blue goal
 *   24  red goal
 *   21, 22, 23  obelisk motif tags -- these tell you the randomised pattern, they are
 *               not alignment targets, so do not aim at them
 *
 * Controls:
 *   left stick / right stick x   manual mecanum drive
 *   left bumper                  hold to auto align on TARGET_TAG_ID
 *
 * Hardware config names used here -- change them to match the Driver Station config:
 *   front_left_drive, front_right_drive, back_left_drive, back_right_drive, "Webcam 1"
 *
 * The three gains below are a P controller per axis. Tune one at a time, and tune them
 * low: too high and the robot oscillates around the tag instead of settling on it.
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

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

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
    public void loop() {
        AprilTagDetection target = findTarget();

        if (target != null && gamepad1.left_bumper) {
            // ftcPose is the tag relative to the camera:
            //   range   straight line distance, inches
            //   bearing left/right angle off the camera centre, degrees -- turn to zero it
            //   yaw     how far off square we are to the tag face, degrees -- strafe to zero it
            // Signs are for the drive() convention below: +right and +clockwise. The SDK
            // sample RobotAutoDriveToAprilTagOmni uses +left and +counter-clockwise, so
            // its strafe and turn signs are the opposite of these.
            double rangeError = target.ftcPose.range - DESIRED_RANGE;
            double drive = clip(rangeError * RANGE_GAIN, MAX_AUTO_SPEED);
            double strafe = clip(target.ftcPose.yaw * STRAFE_GAIN, MAX_AUTO_STRAFE);
            double turn = clip(-target.ftcPose.bearing * TURN_GAIN, MAX_AUTO_TURN);

            drive(drive, strafe, turn);
            telemetry.addLine("AUTO ALIGN");
            telemetry.addData("Range error (in)", "%.1f", rangeError);
        } else {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addLine(target == null
                    ? "No target -- drive manually until the tag is in frame"
                    : "Target found -- hold left bumper to align");
        }

        if (target != null) {
            telemetry.addData("Tag", target.id);
            telemetry.addData("range / bearing / yaw", "%.1f in  %.1f deg  %.1f deg",
                    target.ftcPose.range, target.ftcPose.bearing, target.ftcPose.yaw);
        }
    }

    @Override
    public void stop() {
        drive(0, 0, 0);
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

    private void drive(double forward, double right, double turn) {
        double frontLeftPower = forward + right + turn;
        double frontRightPower = forward - right - turn;
        double backLeftPower = forward - right + turn;
        double backRightPower = forward + right - turn;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        frontLeft.setPower(frontLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        backLeft.setPower(backLeftPower / max);
        backRight.setPower(backRightPower / max);
    }
}
