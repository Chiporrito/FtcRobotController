package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Grinder_RED — adapted from Grinder_BLUE.
 * Differences from BLUE:
 *   1. TARGET_TAG_ID = 24
 *   2. Turns LEFT (CCW) after reversing instead of right
 *
 * State flow: DRIVE_FORWARD → SEARCH → ALIGN → APPROACH → FINE_ALIGN → SHOOT → REVERSE → TURN_LEFT → DONE
 */
@Autonomous(name = "Grinder RED", group = "")
public class Grinder_RED extends LinearOpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftLauncher, rightLauncher;
    private Limelight3A limelight;

    // ── Tuning constants ──────────────────────────────────────────────────────

    private static final int    TARGET_TAG_ID           = 24;
    private static final double DRIVE_FORWARD_SEC       = 2.0;
    private static final double REVERSE_SEC             = 3.5;
    private static final double TURN_LEFT_SEC           = 0.4;   // CHANGED: was TURN_RIGHT_SEC
    private static final double TARGET_AREA_THRESHOLD   = 0.05;
    private static final double TX_TOLERANCE_DEG        = 3.5;
    private static final double ALIGN_KP                = 0.045;
    private static final double ALIGN_MAX_POWER         = 0.65;
    private static final double APPROACH_SPEED          = 0.60;
    private static final double APPROACH_YAW_CLAMP      = 0.20;
    private static final double SEARCH_TURN_POWER       = 0.30;
    private static final double FINE_ALIGN_SETTLE_SEC   = 0.5;
    private static final double CLOSE_ENOUGH_TA         = 0.03;
    private static final double FINE_ALIGN_TIMEOUT_SEC  = 2.0;
    private static final double RIGHT_BUMPER_DELAY_SEC  = 0.2;
    private static final double SHOOT_TOTAL_SEC         = 2.0;
    private static final double SHOOT_POWER             = 0.5;

    // ── State machine ─────────────────────────────────────────────────────────
    private enum State {
        DRIVE_FORWARD,
        SEARCH,
        ALIGN,
        APPROACH,
        FINE_ALIGN,
        SHOOT,
        REVERSE,
        TURN_LEFT,   // CHANGED: was TURN_RIGHT
        DONE
    }

    // ── Entry point ───────────────────────────────────────────────────────────
    @Override
    public void runOpMode() {

        initHardware();

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);
        telemetry.addData("Status", "Ready — press START");
        telemetry.update();

        waitForStart();

        ElapsedTime runtime    = new ElapsedTime();
        ElapsedTime stateTimer = new ElapsedTime();
        stateTimer.reset();

        State  state = State.DRIVE_FORWARD;
        double maxTA = 0.0;

        // ── Main loop ─────────────────────────────────────────────────────────
        while (opModeIsActive()) {

            boolean tagSeen = false;
            double  tx      = 0.0;
            double  ta      = 0.0;

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    if (fid.getFiducialId() == TARGET_TAG_ID) {
                        tagSeen = true;
                        tx      = fid.getTargetXDegrees();
                        ta      = fid.getTargetArea();
                        break;
                    }
                }
            }

            if (tagSeen && ta > maxTA) maxTA = ta;

            switch (state) {

                case DRIVE_FORWARD:
                    if (stateTimer.seconds() >= DRIVE_FORWARD_SEC) {
                        stopDrive();
                        state = State.SEARCH;
                        stateTimer.reset();
                    } else {
                        driveWithYaw(APPROACH_SPEED, 0);
                    }
                    break;

                case SEARCH:
                    if (tagSeen) {
                        stopDrive();
                        state = State.ALIGN;
                        stateTimer.reset();
                    } else {
                        rotateCCW(SEARCH_TURN_POWER);
                    }
                    break;

                case ALIGN:
                    if (!tagSeen) {
                        stopDrive();
                        state = State.SEARCH;
                        break;
                    }
                    if (Math.abs(tx) <= TX_TOLERANCE_DEG) {
                        stopDrive();
                        state = State.APPROACH;
                        stateTimer.reset();
                    } else {
                        double yaw = clamp(tx * ALIGN_KP, -ALIGN_MAX_POWER, ALIGN_MAX_POWER);
                        rotateInPlace(yaw);
                    }
                    break;

                case APPROACH:
                    if (!tagSeen) {
                        stopDrive();
                        state = (maxTA >= CLOSE_ENOUGH_TA) ? State.SHOOT : State.ALIGN;
                        stateTimer.reset();
                        break;
                    }
                    if (ta >= TARGET_AREA_THRESHOLD) {
                        stopDrive();
                        state = State.FINE_ALIGN;
                        stateTimer.reset();
                    } else {
                        double yawCorr = clamp(tx * ALIGN_KP, -APPROACH_YAW_CLAMP, APPROACH_YAW_CLAMP);
                        driveWithYaw(APPROACH_SPEED, yawCorr);
                    }
                    break;

                case FINE_ALIGN:
                    if (stateTimer.seconds() >= FINE_ALIGN_TIMEOUT_SEC) {
                        stopDrive();
                        state = State.SHOOT;
                        stateTimer.reset();
                    } else if (tagSeen && Math.abs(tx) > TX_TOLERANCE_DEG) {
                        double yaw = clamp(tx * ALIGN_KP, -ALIGN_MAX_POWER, ALIGN_MAX_POWER);
                        rotateInPlace(yaw);
                    } else {
                        stopDrive();
                        if (stateTimer.seconds() >= FINE_ALIGN_SETTLE_SEC) {
                            state = State.SHOOT;
                            stateTimer.reset();
                        }
                    }
                    break;

                case SHOOT:
                    stopDrive();
                    double elapsed = stateTimer.seconds();

                    if (elapsed >= SHOOT_TOTAL_SEC) {
                        stopAll();
                        state = State.REVERSE;
                        stateTimer.reset();

                    } else if (elapsed < RIGHT_BUMPER_DELAY_SEC) {
                        leftLauncher.setPower(0);
                        rightLauncher.setPower(0);

                    } else {
                        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
                        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
                        leftLauncher.setPower(SHOOT_POWER);
                        rightLauncher.setPower(SHOOT_POWER);
                    }
                    break;

                case REVERSE:
                    if (stateTimer.seconds() >= REVERSE_SEC) {
                        stopDrive();
                        state = State.TURN_LEFT;   // CHANGED: was TURN_RIGHT
                        stateTimer.reset();
                    } else {
                        driveWithYaw(-APPROACH_SPEED, 0);
                    }
                    break;

                case TURN_LEFT:                    // CHANGED: was TURN_RIGHT
                    if (stateTimer.seconds() >= TURN_LEFT_SEC) {
                        stopDrive();
                        state = State.DONE;
                    } else {
                        rotateCCW(SEARCH_TURN_POWER);  // CHANGED: was rotateCW
                    }
                    break;

                case DONE:
                    stopAll();
                    break;
            }

            // ── Telemetry ─────────────────────────────────────────────────────
            telemetry.addData("State",    state);
            telemetry.addData("Tag seen", tagSeen);
            telemetry.addData("TX",       "%.2f deg", tx);
            telemetry.addData("TA",       "%.4f %%",  ta);
            telemetry.addData("Max TA",   "%.4f %%",  maxTA);
            telemetry.addData("Runtime",  "%.1f s",   runtime.seconds());
            telemetry.update();
        }

        limelight.stop();
    }

    // ── Hardware init ─────────────────────────────────────────────────────────
    private void initHardware() {

        frontLeft     = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight    = hardwareMap.get(DcMotor.class, "Front Right");
        backLeft      = hardwareMap.get(DcMotor.class, "Back Left");
        backRight     = hardwareMap.get(DcMotor.class, "Back Right");
        leftLauncher  = hardwareMap.get(DcMotor.class, "Left Launcher");
        rightLauncher = hardwareMap.get(DcMotor.class, "Right Launcher");
        limelight     = hardwareMap.get(Limelight3A.class, "limelight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
    }

    // ── Drive helpers ─────────────────────────────────────────────────────────

    private void driveWithYaw(double forward, double yaw) {
        double left  = forward + yaw;
        double right = forward - yaw;
        double max   = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) { left /= max; right /= max; }
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }

    private void rotateInPlace(double yaw) {
        frontLeft.setPower(yaw);
        backLeft.setPower(yaw);
        frontRight.setPower(-yaw);
        backRight.setPower(-yaw);
    }

    private void rotateCCW(double power) { rotateInPlace(-Math.abs(power)); }
    private void rotateCW(double power)  { rotateInPlace( Math.abs(power)); }

    private void stopDrive() {
        frontLeft.setPower(0);  frontRight.setPower(0);
        backLeft.setPower(0);   backRight.setPower(0);
    }

    private void stopAll() {
        stopDrive();
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}