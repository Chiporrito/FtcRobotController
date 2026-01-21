package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "M4 (Clean TeleOp)")
public class M4 extends LinearOpMode {

    // Drive motors
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    // Launcher / intake motors
    private DcMotor leftLauncher;
    private DcMotor rightLauncher;
    private DcMotor leftExtra;
    private DcMotor rightExtra;

    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();

        // ===== Hardware Map =====
        backLeft      = hardwareMap.get(DcMotor.class, "Back Left");
        backRight     = hardwareMap.get(DcMotor.class, "Back Right");
        frontLeft     = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight    = hardwareMap.get(DcMotor.class, "Front Right");

        leftLauncher  = hardwareMap.get(DcMotor.class, "Left Launcher");
        rightLauncher = hardwareMap.get(DcMotor.class, "Right Launcher");
        leftExtra     = hardwareMap.get(DcMotor.class, "LeftEXTRA");
        rightExtra    = hardwareMap.get(DcMotor.class, "RightEXTRA");

        // ===== Motor Directions =====
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD); // as requested

        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        leftExtra.setDirection(DcMotor.Direction.FORWARD);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        rightExtra.setDirection(DcMotor.Direction.REVERSE);

        // ===== Brake when stopped =====
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // ===== Main Loop =====
        while (opModeIsActive()) {

            // POV mecanum drive
            double axial   = -gamepad1.left_stick_y;   // forward/back
            double lateral = -gamepad1.left_stick_x;   // strafe
            double yaw     =  gamepad1.right_stick_x;  // rotate

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize motor powers
            double max = Math.max(
                    Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
            );

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Set drive power
            frontLeft.setPower(leftFrontPower);
            frontRight.setPower(rightFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);

            // ===== Launcher Controls =====

            // FULL POWER (square)
            if (gamepad2.square) {
                leftLauncher.setPower(1.0);
                rightLauncher.setPower(1.0);
                leftExtra.setPower(1.0);
                rightExtra.setPower(1.0);
            }
            // LOW POWER (triangle)
            else if (gamepad2.triangle) {
                leftLauncher.setPower(0.35);
                rightLauncher.setPower(0.35);
                leftExtra.setPower(0.35);
                rightExtra.setPower(0.35);
            }
            // OFF
            else {
                leftLauncher.setPower(0);
                rightLauncher.setPower(0);
                leftExtra.setPower(0);
                rightExtra.setPower(0);
            }

            telemetry.addData("Run Time", runtime.toString());
            telemetry.update();
        }
    }
}
