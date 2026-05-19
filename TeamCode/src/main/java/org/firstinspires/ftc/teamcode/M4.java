package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "M4TeleOp", group = "")
public class M4 extends LinearOpMode {

    // Declare hardware
    private DcMotor frontLeft    = null;
    private DcMotor frontRight   = null;
    private DcMotor backLeft     = null;
    private DcMotor backRight    = null;
    private DcMotor leftLauncher  = null;
    private DcMotor rightLauncher = null;
    private DcMotor leftExtra    = null;
    private DcMotor rightExtra   = null;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize hardware from the hardware map
        frontLeft    = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight   = hardwareMap.get(DcMotor.class, "Front Right");
        backLeft     = hardwareMap.get(DcMotor.class, "Back Left");
        backRight    = hardwareMap.get(DcMotor.class, "Back Right");
        leftLauncher  = hardwareMap.get(DcMotor.class, "Left Launcher");
        rightLauncher = hardwareMap.get(DcMotor.class, "Right Launcher");
        leftExtra    = hardwareMap.get(DcMotor.class, "LeftEXTRA");
        rightExtra   = hardwareMap.get(DcMotor.class, "RightEXTRA");

        // Send telemetry message to indicate initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // ── One-time setup after START ──────────────────────────────────────

        // Zero-power behavior: BRAKE on all drive motors
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Drive motor directions (mecanum layout)
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        // Launcher / extra motor default directions
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        leftExtra.setDirection(DcMotor.Direction.FORWARD);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        rightExtra.setDirection(DcMotor.Direction.REVERSE);

        // ── Main loop: run until the end of the match ───────────────────────
        // (driver presses STOP)
        while (opModeIsActive()) {

            // ── POV Drive ──────────────────────────────────────────────────
            // Left joystick: forward/back and strafe; right joystick: rotate.
            // Note: pushing the stick forward gives a negative value, so negate.
            double axial   = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine joystick inputs for each mecanum wheel
            double leftFrontPower  =  axial + lateral + yaw;
            double rightFrontPower =  axial - lateral - yaw;
            double leftBackPower   =  axial - lateral + yaw;
            double rightBackPower  =  axial + lateral - yaw;

            // Normalize so that no wheel power exceeds 100 %
            // This preserves the desired motion ratio between wheels.
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

            // Send power to drive wheels
            frontLeft.setPower(leftFrontPower);
            frontRight.setPower(rightFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);

            // ── Gamepad 2 – Launcher & Extra motors ────────────────────────

            if (gamepad2.left_bumper) {
                // Left bumper: run launchers inward (intake/collect direction)
                leftLauncher.setDirection(DcMotor.Direction.REVERSE);
                rightLauncher.setDirection(DcMotor.Direction.FORWARD);
                leftLauncher.setPower(0.5);
                rightLauncher.setPower(0.5);

            } else if (gamepad2.right_bumper) {
                // Right bumper: run extra motors outward
                leftExtra.setDirection(DcMotor.Direction.FORWARD);
                rightExtra.setDirection(DcMotor.Direction.REVERSE);
                leftExtra.setPower(0.5);
                rightExtra.setPower(0.5);

            } else if (gamepad2.dpad_up) {
                // Right bumper: run extra motors outward
                leftExtra.setDirection(DcMotor.Direction.FORWARD);
                rightExtra.setDirection(DcMotor.Direction.REVERSE);
                leftExtra.setPower(1);
                rightExtra.setPower(1);

            } else if (gamepad2.dpad_down) {
                // Right bumper: run extra motors outward
                leftExtra.setDirection(DcMotor.Direction.FORWARD);
                rightExtra.setDirection(DcMotor.Direction.REVERSE);
                leftExtra.setPower(0.75);
                rightExtra.setPower(0.75);


            } else if (gamepad2.y) {   // Triangle / Y button
                // Y: run launchers + extras together at reduced power (shoot/launch)
                leftLauncher.setDirection(DcMotor.Direction.FORWARD);
                leftExtra.setDirection(DcMotor.Direction.REVERSE);
                rightLauncher.setDirection(DcMotor.Direction.REVERSE);
                rightExtra.setDirection(DcMotor.Direction.FORWARD);

                leftExtra.setPower(0.35);
                rightExtra.setPower(0.35);
                leftLauncher.setPower(0.45);
                rightLauncher.setPower(0.45);

            } else {
                // Default (no bumper / triangle pressed): stop launchers & extras
                leftLauncher.setDirection(DcMotor.Direction.REVERSE);
                leftExtra.setDirection(DcMotor.Direction.FORWARD);
                rightLauncher.setDirection(DcMotor.Direction.FORWARD);
                rightExtra.setDirection(DcMotor.Direction.REVERSE);

                leftLauncher.setPower(0);
                rightLauncher.setPower(0);
                leftExtra.setPower(0);
                rightExtra.setPower(0);
            }

            // ── Telemetry ──────────────────────────────────────────────────
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}