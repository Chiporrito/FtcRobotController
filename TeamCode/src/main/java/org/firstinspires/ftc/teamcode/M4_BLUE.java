package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * M4Auto — Total recode. Limelight 3A AprilTag autonomous for FTC DECODE.
 *
 * Key fixes in this version:
 *   1. limelight.start() called BEFORE waitForStart() — required for reliable data.
 *   2. setPollRate(100) for fast 100Hz polling.
 *   3. Detection uses getFiducialResults() — the correct API for AprilTags.
 *   4. Accepts ANY fiducial (not filtering by ID) since only one tag is on field.
 *   5. tx/ta sourced from the FiducialResult object directly, not from result root.
 *
 * State flow:  SEARCH → ALIGN → APPROACH → FINE_ALIGN → SHOOT → DONE
 */
@Autonomous(name = "M4 BLUE", group = "")
public class M4_BLUE extends LinearOpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftLauncher, rightLauncher, leftExtra, rightExtra;
    private Limelight3A limelight;

    // ── Tuning constants ──────────────────────────────────────────────────────

    /**
     * Stopping distance from wall expressed as target area percentage.
     * Larger value = stop closer to wall.
     * Watch TA on telemetry: start at 3.5, tune up/down from there.
     */
    private static final int    TARGET_TAG_ID          = 20;

    /** Seconds to drive straight forward before starting the search spin. */
    private static final double DRIVE_FORWARD_SEC = 2;

    /** Seconds to drive backwards after shooting. */
    private static final double REVERSE_SEC   = 3.5;

    /** Seconds to spin left on axis after reversing. */
    private static final double TURN_LEFT_SEC = 0.4;
    private static final double TARGET_AREA_THRESHOLD = 0.05;

    /**
     * Horizontal dead-band in degrees.
     * Robot is considered centred when |tx| < this value.
     */
    private static final double TX_TOLERANCE_DEG = 3.5;

    /**
     * P-controller gain for yaw correction.
     * Too low = slow to align. Too high = oscillates. Start at 0.030.
     */
    private static final double ALIGN_KP = 0.045;

    /** Maximum rotation power the P-controller can output. */
    private static final double ALIGN_MAX_POWER = 0.65;

    /** Forward drive speed during APPROACH. */
    private static final double APPROACH_SPEED = 0.60;

    /** Maximum yaw correction applied while driving forward. */
    private static final double APPROACH_YAW_CLAMP = 0.20;

    /** Rotation speed while scanning for the tag in SEARCH. */
    private static final double SEARCH_TURN_POWER = 0.30;

    /** Seconds of settled stillness required before SHOOT begins. */
    private static final double FINE_ALIGN_SETTLE_SEC = 0.5;

    /**
     * If we lose the tag during APPROACH but TA previously reached this value,
     * the robot is already close enough — skip straight to SHOOT.
     */
    private static final double CLOSE_ENOUGH_TA = 0.03;

    /** If FINE_ALIGN takes longer than this, force-advance to SHOOT anyway. */
    private static final double FINE_ALIGN_TIMEOUT_SEC = 2.0;

    /**
     * Delay (seconds) before launchers join the extras.
     * Mirrors: hold right bumper, then 0.2s later hold left bumper.
     */
    private static final double RIGHT_BUMPER_DELAY_SEC = 0.2;

    /** Total shoot duration in seconds. */
    private static final double SHOOT_TOTAL_SEC = 2.0;

    /** All shoot motor power — 0.5 as requested. */
    private static final double SHOOT_POWER = 0.645;

    // ── State machine ─────────────────────────────────────────────────────────
    private enum State {
        DRIVE_FORWARD,
        SEARCH,
        ALIGN,
        APPROACH,
        FINE_ALIGN,
        SHOOT,
        REVERSE,
        TURN_LEFT,
        DONE
    }

    // ── Entry point ───────────────────────────────────────────────────────────
    @Override
    public void runOpMode() {

        initHardware();

        // ── Limelight init — start() MUST be before waitForStart() ────────────
        limelight.pipelineSwitch(0);  // slot 0 = your AprilTags pipeline
        limelight.start();            // start BEFORE waitForStart

        telemetry.setMsTransmissionInterval(11);
        telemetry.addData("Status", "Ready — press START");
        telemetry.update();

        waitForStart();

        ElapsedTime runtime    = new ElapsedTime();
        ElapsedTime stateTimer = new ElapsedTime();
        stateTimer.reset(); // reset HERE so DRIVE_FORWARD timer starts exactly at match start
        State state = State.DRIVE_FORWARD;
        double maxTA = 0.0; // tracks the closest we've been to the tag

        // ── Main loop ─────────────────────────────────────────────────────────
        while (opModeIsActive()) {

            // ── Poll Limelight ────────────────────────────────────────────────
            //
            // We use getFiducialResults() — the correct API for AprilTag pipelines.
            // tx and ta are pulled from the FiducialResult object, not from the
            // result root (result.getTx() is the crosshair offset to the primary
            // target and may differ from the fiducial-specific value).
            //
            // Since only one AprilTag exists on the field during this auto, we
            // accept the first fiducial result we find without filtering by ID.

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

            // Track closest approach so we know if we've been near the tag
            if (tagSeen && ta > maxTA) maxTA = ta;

            // ── State machine ─────────────────────────────────────────────────
            switch (state) {

                // ─────────────────────────────────────────────────────────────
                case DRIVE_FORWARD:
                    // Drive straight forward for DRIVE_FORWARD_SEC seconds.
                    // No tag checking here — just get into the field first.
                    // Once timer expires, transition to SEARCH and start rotating.
                    if (stateTimer.seconds() >= DRIVE_FORWARD_SEC) {
                        stopDrive();
                        state = State.SEARCH;
                        stateTimer.reset();
                    } else {
                        driveWithYaw(APPROACH_SPEED, 0);
                    }
                    break;

                // ─────────────────────────────────────────────────────────────
                case SEARCH:
                    // Rotate CCW slowly until a fiducial appears.
                    // Change rotateCCW to rotateCW if starting on the other side.
                    if (tagSeen) {
                        stopDrive();
                        state = State.ALIGN;
                        stateTimer.reset();
                    } else {
                        rotateCCW(SEARCH_TURN_POWER);
                    }
                    break;

                // ─────────────────────────────────────────────────────────────
                case ALIGN:
                    // P-controller zeroes horizontal offset.
                    // tx > 0 → tag is right of centre → rotate CW  (+yaw)
                    // tx < 0 → tag is left of centre  → rotate CCW (-yaw)
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

                // ─────────────────────────────────────────────────────────────
                case APPROACH:
                    // Drive straight toward the tag with live yaw correction.
                    // Stop when target area hits the threshold.
                    if (!tagSeen) {
                        stopDrive();
                        // If we were already close when we lost the tag, skip
                        // re-alignment and go straight to SHOOT — this prevents
                        // infinite spinning when the tag goes out of FOV up close.
                        if (maxTA >= CLOSE_ENOUGH_TA) {
                            state = State.SHOOT;
                        } else {
                            state = State.ALIGN;
                        }
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

                // ─────────────────────────────────────────────────────────────
                case FINE_ALIGN:
                    // Final centering correction. Settle clock resets while still
                    // correcting — only advance to SHOOT after being stable for
                    // FINE_ALIGN_SETTLE_SEC seconds.
                    // TIMEOUT: if we can't settle within FINE_ALIGN_TIMEOUT_SEC
                    // (e.g. tag too close to see), force-advance to SHOOT anyway.
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

                // ─────────────────────────────────────────────────────────────
                case SHOOT:
                    // Shoot sequence — mirrors holding right bumper then left bumper in M4TeleOp.
                    //
                    // Phase 1 [0 → RIGHT_BUMPER_DELAY_SEC]  (right bumper only)
                    //   Extra motors run outward at 0.5 power.
                    //   Launchers stay off — identical to holding right bumper in TeleOp.
                    //
                    // Phase 2 [RIGHT_BUMPER_DELAY_SEC → SHOOT_TOTAL_SEC]  (both bumpers)
                    //   Launchers join in — identical to also holding left bumper in TeleOp.
                    //   All motors at 0.5 power for the remainder of the 2s window.
                    //
                    // Robot stays stationary throughout.

                    stopDrive();
                    double elapsed = stateTimer.seconds();

                    if (elapsed >= SHOOT_TOTAL_SEC) {
                        // Done — cut everything
                        stopAll();
                        state = State.REVERSE;
                        stateTimer.reset();

                    } else if (elapsed < RIGHT_BUMPER_DELAY_SEC) {
                        // Phase 1 — right bumper only (extra motors outward)
                        leftExtra.setDirection(DcMotor.Direction.FORWARD);
                        rightExtra.setDirection(DcMotor.Direction.REVERSE);
                        leftExtra.setPower(SHOOT_POWER);
                        rightExtra.setPower(SHOOT_POWER);
                        leftLauncher.setPower(0);
                        rightLauncher.setPower(0);

                    } else {
                        // Phase 2 — right bumper + left bumper (extras + launchers)
                        leftExtra.setDirection(DcMotor.Direction.FORWARD);
                        rightExtra.setDirection(DcMotor.Direction.REVERSE);
                        leftExtra.setPower(SHOOT_POWER);
                        rightExtra.setPower(SHOOT_POWER);

                        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
                        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
                        leftLauncher.setPower(SHOOT_POWER);
                        rightLauncher.setPower(SHOOT_POWER);
                    }
                    break;

                // ─────────────────────────────────────────────────────────────
                case REVERSE:
                    // Drive straight backwards for REVERSE_SEC seconds.
                    if (stateTimer.seconds() >= REVERSE_SEC) {
                        stopDrive();
                        state = State.TURN_LEFT;
                        stateTimer.reset();
                    } else {
                        driveWithYaw(-APPROACH_SPEED, 0);
                    }
                    break;

                // ─────────────────────────────────────────────────────────────
                case TURN_LEFT:
                    // Spin left (CCW) on own axis for TURN_LEFT_SEC seconds.
                    if (stateTimer.seconds() >= TURN_LEFT_SEC) {
                        stopDrive();
                        state = State.DONE;
                    } else {
                        rotateCW(SEARCH_TURN_POWER);
                    }
                    break;

                // ─────────────────────────────────────────────────────────────
                case DONE:
                    stopAll();
                    break;
            }

            // ── Driver Station telemetry ──────────────────────────────────────
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

        frontLeft  = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        backLeft   = hardwareMap.get(DcMotor.class, "Back Left");
        backRight  = hardwareMap.get(DcMotor.class, "Back Right");

        leftLauncher  = hardwareMap.get(DcMotor.class, "Left Launcher");
        rightLauncher = hardwareMap.get(DcMotor.class, "Right Launcher");
        leftExtra     = hardwareMap.get(DcMotor.class, "LeftEXTRA");
        rightExtra    = hardwareMap.get(DcMotor.class, "RightEXTRA");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        leftExtra.setDirection(DcMotor.Direction.FORWARD);
        rightExtra.setDirection(DcMotor.Direction.REVERSE);
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
        leftLauncher.setPower(0);  rightLauncher.setPower(0);
        leftExtra.setPower(0);     rightExtra.setPower(0);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}