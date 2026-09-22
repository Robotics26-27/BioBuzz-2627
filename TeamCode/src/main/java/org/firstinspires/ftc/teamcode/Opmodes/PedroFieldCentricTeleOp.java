package org.firstinspires.ftc.teamcode.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Field centric mecanum drive through the Pedro Pathing follower.
 *
 * Heading comes from the follower's localizer, not the IMU directly. The robot's heading at
 * init is field "forward", so point it away from the driver before pressing init.
 *
 * Controls (gamepad1):
 *   left stick      translate (field relative)
 *   right stick x   turn
 */
@TeleOp(name = "Pedro Field Centric TeleOp", group = "TeleOp")
public class PedroFieldCentricTeleOp extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        // Heading 0 rad = the direction the robot faces right now.
        follower.setStartingPose(new Pose());
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {
        // Pedro: +forward, +strafe = left, +turn = counter-clockwise. Stick y and x are
        // negated to match, same as LocalizationTest in Tuning.java.
        // Last arg false = field centric (robotCentric = false).
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        telemetry.addData("x (in)", "%.1f", follower.getPose().getX());
        telemetry.addData("y (in)", "%.1f", follower.getPose().getY());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
    }
}
