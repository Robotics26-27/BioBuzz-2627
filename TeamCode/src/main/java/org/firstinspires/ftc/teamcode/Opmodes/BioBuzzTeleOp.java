package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field centric mecanum drive + intake + two launchers. No Pedro Pathing.
 *
 * CONTROLLER MAP (Logitech F310 in XInput mode, switch on the back set to "X"; gamepad1)
 *   left stick          drive, field relative
 *   right stick X       turn
 *   right bumper        intake IN
 *   left bumper         intake OUT (reverse)
 *   right trigger       tap: toggle both launchers on/off (spin to target RPM, then hold it)
 *   BACK                reset IMU heading (point robot away from driver first)
 *
 * Hardware config names -- must match the Driver Station config exactly:
 *   Control Hub:   RightBack (0), RightFront (1), LeftFront (2), LeftBack (3)
 *   Expansion Hub: Intake1 (0), Intake2 (1), NectarLauncher (2), PollenLauncher (3)
 *   IMU: "imu" (default name in the Control Hub config)
 */
@TeleOp(name = "BioBuzz TeleOp", group = "TeleOp")
public class BioBuzzTeleOp extends OpMode {

    // Placeholders, not measured on the robot. Tune.
    private static final double INTAKE_POWER = 1.0;
    private static final double LAUNCHER_TARGET_RPM = 3000;
    private static final double LAUNCHER_RPM_TOLERANCE = 50;
    private static final double LAUNCHER_TICKS_PER_REV = 28; // bare motor shaft, per team

    private enum LauncherState { OFF, SPINNING_UP, READY }
    private LauncherState launcherState = LauncherState.OFF;

    private DcMotor rightBack, rightFront, leftFront, leftBack;
    private DcMotor intake1, intake2;
    private DcMotorEx nectarLauncher, pollenLauncher;
    private IMU imu;

    @Override
    public void init() {
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        intake1 = hardwareMap.get(DcMotor.class, "Intake1");
        intake2 = hardwareMap.get(DcMotor.class, "Intake2");
        nectarLauncher = hardwareMap.get(DcMotorEx.class, "NectarLauncher");
        pollenLauncher = hardwareMap.get(DcMotorEx.class, "PollenLauncher");

        // ===== MOTOR DIRECTIONS: CHANGE HERE =====
        // Drive: push stick forward on blocks, all 4 wheels must spin forward. Flip any that don't.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        // Intake: right bumper must pull game elements in. Intake2 REVERSE assumes it is mounted mirrored.
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.REVERSE);
        // Launchers: trigger must throw game elements out.
        nectarLauncher.setDirection(DcMotor.Direction.FORWARD);
        pollenLauncher.setDirection(DcMotor.Direction.FORWARD);
        // =========================================

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        nectarLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pollenLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Velocity control needs the encoder.
        nectarLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // Control Hub: logo faces forward, USB port faces up.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        telemetry.addLine("Point robot away from driver, then press start.");
    }

    @Override
    public void start() {
        imu.resetYaw();
    }

    @Override
    public void loop() {
        if (gamepad1.back) {
            imu.resetYaw();
        }

        double forward = -gamepad1.left_stick_y; // stick y is negative when pushed away
        double right = gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x; // negated: stick left was turning the robot right

        double headingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // CCW positive

        // Rotate the field-relative stick vector by -heading into the robot frame.
        double cos = Math.cos(-headingRad);
        double sin = Math.sin(-headingRad);
        double robotForward = forward * cos + right * sin;
        double robotRight = right * cos - forward * sin;

        // Standard mecanum mix, normalised so no wheel exceeds 1.0.
        double lf = robotForward + robotRight + turn;
        double lb = robotForward - robotRight + turn;
        double rf = robotForward - robotRight - turn;
        double rb = robotForward + robotRight - turn;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb))));
        leftFront.setPower(lf / max);
        leftBack.setPower(lb / max);
        rightFront.setPower(rf / max);
        rightBack.setPower(rb / max);

        double intakePower = gamepad1.right_bumper ? INTAKE_POWER
                : gamepad1.left_bumper ? -INTAKE_POWER : 0;
        intake1.setPower(intakePower);
        intake2.setPower(intakePower);

        // Tap right trigger: OFF <-> spin up. Velocity control holds the RPM once reached.
        if (gamepad1.rightTriggerWasPressed()) {
            launcherState = launcherState == LauncherState.OFF ? LauncherState.SPINNING_UP : LauncherState.OFF;
        }
        double nectarRpm = nectarLauncher.getVelocity() * 60 / LAUNCHER_TICKS_PER_REV;
        double pollenRpm = pollenLauncher.getVelocity() * 60 / LAUNCHER_TICKS_PER_REV;
        if (launcherState == LauncherState.OFF) {
            nectarLauncher.setVelocity(0);
            pollenLauncher.setVelocity(0);
        } else {
            double targetTicksPerSec = LAUNCHER_TARGET_RPM * LAUNCHER_TICKS_PER_REV / 60;
            nectarLauncher.setVelocity(targetTicksPerSec);
            pollenLauncher.setVelocity(targetTicksPerSec);
            boolean atSpeed = Math.abs(nectarRpm - LAUNCHER_TARGET_RPM) < LAUNCHER_RPM_TOLERANCE
                    && Math.abs(pollenRpm - LAUNCHER_TARGET_RPM) < LAUNCHER_RPM_TOLERANCE;
            launcherState = atSpeed ? LauncherState.READY : LauncherState.SPINNING_UP;
        }

        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(headingRad));
        telemetry.addData("Intake", intakePower);
        telemetry.addData("Launcher", launcherState);
        telemetry.addData("Nectar RPM", "%.0f", nectarRpm);
        telemetry.addData("Pollen RPM", "%.0f", pollenRpm);
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        intake1.setPower(0);
        intake2.setPower(0);
        nectarLauncher.setPower(0);
        pollenLauncher.setPower(0);
    }
}
