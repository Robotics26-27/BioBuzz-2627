package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field centric mecanum drive using the control hub IMU.
 *
 * Push the left stick away from you and the robot drives away from you, no matter which
 * way it is pointing. Right stick turns.
 *
 * Controls:
 *   left stick      translate (field relative)
 *   right stick x   turn
 *   left bumper     hold for robot centric driving
 *   right trigger   hold for slow mode
 *   options         re-zero the heading (point robot away from driver first)
 *
 * Hardware config names used here -- change them to match the Driver Station config:
 *   front_left_drive, front_right_drive, back_left_drive, back_right_drive, imu
 *
 * Note: if you are already running Pedro Pathing, you get field centric for free with
 * follower.setTeleOpDrive(forward, lateral, turn, false) -- the last arg is "robotCentric".
 * This OpMode exists so you can drive without a tuned follower.
 */
@TeleOp(name = "Example: Field Centric TeleOp", group = "Examples")
public class FieldCentricTeleOp extends OpMode {

    /** Multiplier applied to every wheel while the slow mode trigger is held. */
    private static final double SLOW_MODE_SCALE = 0.35;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Flip whichever side runs backwards on your drivetrain. Test with the robot on
        // blocks: all four wheels should spin forward when you push the stick forward.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        // Both of these must match how the hub is physically mounted, or the heading is wrong.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addLine("Point the robot away from the driver, then press start.");
    }

    @Override
    public void start() {
        // Whatever direction the robot faces when the match starts becomes "field forward".
        imu.resetYaw();
    }

    @Override
    public void loop() {
        if (gamepad1.optionsWasPressed()) {
            imu.resetYaw();
        }

        double forward = -gamepad1.left_stick_y;   // stick y is negative when pushed away
        double right = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double headingRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (!gamepad1.left_bumper) {
            // Rotate the requested field-relative vector into the robot's frame.
            double cos = Math.cos(-headingRadians);
            double sin = Math.sin(-headingRadians);
            double robotForward = forward * cos - right * sin;
            double robotRight = forward * sin + right * cos;
            forward = robotForward;
            right = robotRight;
        }

        double scale = gamepad1.right_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;
        drive(forward * scale, right * scale, turn * scale);

        telemetry.addData("Mode", gamepad1.left_bumper ? "ROBOT centric" : "FIELD centric");
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(headingRadians));
    }

    @Override
    public void stop() {
        drive(0, 0, 0);
    }

    /** Standard mecanum mix, normalised so no wheel is ever commanded past 1.0. */
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
