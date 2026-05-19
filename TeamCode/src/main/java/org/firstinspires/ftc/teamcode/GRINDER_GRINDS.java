package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "GRINDER_GRINDS", group = "")
public class GRINDER_GRINDS extends LinearOpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake;
    private DcMotor intakeStart;
    private DcMotor leftLauncher;
    private DcMotor rightLauncher;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ── Initialization: drive wheels ──────────────────────────────────────
        frontLeft    = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight   = hardwareMap.get(DcMotor.class, "Front Right");
        backLeft     = hardwareMap.get(DcMotor.class, "Back Left");
        backRight    = hardwareMap.get(DcMotor.class, "Back Right");
        intake       = hardwareMap.get(DcMotor.class, "Intake");
        intakeStart  = hardwareMap.get(DcMotor.class, "IntakeStart");
        leftLauncher  = hardwareMap.get(DcMotor.class, "Left Launcher");
        rightLauncher = hardwareMap.get(DcMotor.class, "Right Launcher");

        // Zero-power behavior — all four drive wheels brake on release
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor directions — copied exactly from blocks (Image 3)
        intake.setDirection(DcMotor.Direction.REVERSE);
        intakeStart.setDirection(DcMotor.Direction.REVERSE);
        leftLauncher.setDirection(DcMotor.Direction.FORWARD);
        rightLauncher.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        // ── Initialization: other ─────────────────────────────────────────────
        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // ── Main loop ─────────────────────────────────────────────────────────
        while (opModeIsActive()) {

            // ── Run drive (Gamepad 1) — copied exactly from blocks (Image 2) ──
            // axial   = -LeftStickY  (push up = positive = forward)
            // lateral =  LeftStickX  (push right = positive = strafe right)
            // yaw     =  RightStickX (push right = positive = rotate right)
            double axial   = gamepad1.left_stick_y;
            double lateral =  -gamepad1.left_stick_x;
            double yaw     =  -gamepad1.right_stick_x;

            // Mecanum wheel power mix
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize — no wheel exceeds 100%
            double max = Math.max(Math.abs(leftFrontPower),
                    Math.max(Math.abs(rightFrontPower),
                            Math.max(Math.abs(leftBackPower),
                                    Math.abs(rightBackPower))));
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            frontRight.setPower(rightFrontPower);
            frontLeft.setPower(leftFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);

            // ── Run other (Gamepad 2) — copied exactly from blocks (Image 1) ──

            // Share — play "Secret" sound at low volume for 2 seconds then stop
            if (gamepad2.share) {
                // AndroidSoundPool Volume 0.3, play "Secret"
                sleep(2000);
                // AndroidSoundPool stop
            }

            // Right Bumper — intake + launchers forward (collect / feed mode)
            if (gamepad2.right_bumper) {
                intake.setDirection(DcMotor.Direction.FORWARD);
                leftLauncher.setPower(0.7);
                rightLauncher.setPower(0.7);
                intake.setPower(0.7);
            } else {
                intake.setDirection(DcMotor.Direction.REVERSE);
                intake.setPower(0);
                leftLauncher.setPower(0);
                rightLauncher.setPower(0);
            }

            // Triangle — reverse launcher direction + intake (shoot/launch mode)
            if (gamepad2.triangle) {
                intake.setDirection(DcMotor.Direction.REVERSE);
                leftLauncher.setDirection(DcMotor.Direction.REVERSE);
                rightLauncher.setDirection(DcMotor.Direction.FORWARD);
                leftLauncher.setPower(0.7);
                rightLauncher.setPower(0.7);
                intake.setPower(0.7);
            } else {
                intake.setDirection(DcMotor.Direction.FORWARD);
                leftLauncher.setDirection(DcMotor.Direction.FORWARD);
                rightLauncher.setDirection(DcMotor.Direction.REVERSE);
                leftLauncher.setPower(0);
                rightLauncher.setPower(0);
                intake.setPower(0);
            }

            // Left Bumper — IntakeStart reverse at full power
            if (gamepad2.left_bumper) {
                intakeStart.setDirection(DcMotor.Direction.REVERSE);
                intakeStart.setPower(1);
            } else {
                intakeStart.setDirection(DcMotor.Direction.FORWARD);
                intakeStart.setPower(0);
            }

            // Circle — IntakeStart forward at 0.4
            if (gamepad2.circle) {
                intakeStart.setDirection(DcMotor.Direction.FORWARD);
                intakeStart.setPower(0.4);
            } else {
                intakeStart.setDirection(DcMotor.Direction.REVERSE);
                intakeStart.setPower(0);
            }

            // ── Telemetry ─────────────────────────────────────────────────────
            telemetry.update();
        }

        // OpMode ending — play "GameOver" sound at full volume
        // AndroidSoundPool Volume 1, play "GameOver"
    }
}