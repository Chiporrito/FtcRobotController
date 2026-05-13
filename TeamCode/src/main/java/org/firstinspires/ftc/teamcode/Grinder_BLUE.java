package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Grinder_BLUE — adapted directly from the proven M4Auto template.
 * Only changes from M4Auto:
 *   1. TARGET_TAG_ID = 5 (was 20)
 *   2. No leftExtra / rightExtra motors (Grinder doesn't have them)
 *   3. Motor directions matched to GRINDER_GRINDS TeleOp
 *   4. TURN_RIGHT instead of TURN_LEFT at the end
 *
 * State flow: DRIVE_FORWARD → SEARCH → ALIGN → APPROACH → FINE_ALIGN → SHOOT → REVERSE → TURN_RIGHT → DONE
 */
@Autonomous(name = "Grinder_BLUE", group = "")
public class Grinder_BLUE extends LinearOpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftLauncher, rightLauncher;
    private Limelight3A limelight;

    // ── Tuning constants — identical to M4Auto except where noted ─────────────

    /** AprilTag ID to track. Grinder uses tag 5. */
    private static final int    TARGET_TAG_ID           = 5;

    /** Seconds to drive straight forward before searching. */
    private static final double DRIVE_FORWARD_SEC       = 2.0;

    /** Seconds to drive backward after shooting. */
    private static final double REVERSE_SEC             = 3.5;

    /** Seconds to spin right after reversing. */
    private static final double TURN_RIGHT_SEC          = 0.4;

    /** Stop when target area reaches this value. */
    private static final double TARGET_AREA_THRESHOLD   = 0.05;

    /** Horizontal dead-band — robot is centred when |tx| < this. */
    private static final double TX_TOLERANCE_DEG        = 3.5;

    /** P-controller gain for yaw correction. */
    private static final double ALIGN_KP                = 0.045;

    /** Max rotation power the P-controller can output. */
    private static final double ALIGN_MAX_POWER         = 0.65;

    /** Forward drive speed during APPROACH. */
    private static final double APPROACH_SPEED          = 0.60;

    /** Max yaw correction applied while driving forward. */
    private static final double APPROACH_YAW_CLAMP      = 0.20;

    /** Rotation speed while scanning for the tag in SEARCH. */
    private static final double SEARCH_TURN_POWER       = 0.30;

    /** Seconds of stillness required before SHOOT begins. */
    private static final double FINE_ALIGN_SETTLE_SEC   = 0.5;

    /** If tag lost during APPROACH but TA reached this, skip to SHOOT. */
    private static final double CLOSE_ENOUGH_TA         = 0.03;

    /** Force SHOOT after this many seconds in FINE_ALIGN. */
    private static final double FINE_ALIGN_TIMEOUT_SEC  = 2.0;

    /** Delay before launchers fire (phase 2 of shoot). */
    private static final double RIGHT_BUMPER_DELAY_SEC  = 0.2;

    /** Total shoot duration in seconds. */
    private static final double SHOOT_TOTAL_SEC         = 2.0;

    /** Launcher power. */
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
        TURN_RIGHT,
        DONE
    }

    // ── Entry point ───────────────────────────────────────────────────────────
    @Override
    public void runOpMode() {

        initHardware();

        // ── Limelight — identical to M4Auto, start() BEFORE waitForStart() ───
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

            // ── Poll Limelight — identical to M4Auto ──────────────────────────
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

            // ── State machine — identical to M4Auto ───────────────────────────
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
                    // Grinder has no leftExtra/rightExtra — launchers only.
                    // Phase 1: wait. Phase 2: fire launchers.
                    stopDrive();
                    double elapsed = stateTimer.seconds();

                    if (elapsed >= SHOOT_TOTAL_SEC) {
                        stopAll();
                        state = State.REVERSE;
                        stateTimer.reset();

                    } else if (elapsed < RIGHT_BUMPER_DELAY_SEC) {
                        // Phase 1 — wait (no extra motors on this robot)
                        leftLauncher.setPower(0);
                        rightLauncher.setPower(0);

                    } else {
                        // Phase 2 — fire launchers
                        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
                        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
                        leftLauncher.setPower(SHOOT_POWER);
                        rightLauncher.setPower(SHOOT_POWER);
                    }
                    break;

                case REVERSE:
                    if (stateTimer.seconds() >= REVERSE_SEC) {
                        stopDrive();
                        state = State.TURN_RIGHT;
                        stateTimer.reset();
                    } else {
                        driveWithYaw(-APPROACH_SPEED, 0);
                    }
                    break;

                case TURN_RIGHT:
                    if (stateTimer.seconds() >= TURN_RIGHT_SEC) {
                        stopDrive();
                        state = State.DONE;
                    } else {
                        rotateCW(SEARCH_TURN_POWER);
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

        // ── Directions: matched exactly to GRINDER_GRINDS TeleOp ─────────────
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

    // ── Drive helpers — identical to M4Auto ───────────────────────────────────

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