package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/**
 * LimelightDiag — Total recode following the EXACT official Limelight FTC sample pattern.
 *
 * Key fixes vs previous versions:
 *   1. limelight.start() is called BEFORE waitForStart() — this is required.
 *      Calling it after causes the Limelight to miss its init window.
 *   2. setPollRate(100) sets 100Hz polling for fast, reliable data.
 *   3. telemetry.setMsTransmissionInterval(11) matches the Limelight's update rate.
 *   4. getFiducialResults() is used correctly as the primary detection method.
 *   5. All raw values are printed so you can see exactly what's happening.
 *
 * Run this first. When you point at the tag you should see "Fiducial ID: 5".
 * Once confirmed, M4Auto will work.
 */
@TeleOp(name = "LL Diagnostics", group = "Debug")
public class LimelightDiag extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        // ── Init ─────────────────────────────────────────────────────────────
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11); // match Limelight update rate

        limelight.pipelineSwitch(0);
        limelight.start();          // START BEFORE waitForStart — this is critical

        telemetry.addData(">", "Limelight started. Press PLAY.");
        telemetry.update();

        waitForStart();

        // ── Main loop ─────────────────────────────────────────────────────────
        while (opModeIsActive()) {

            // ── Status ───────────────────────────────────────────────────────
            LLStatus status = limelight.getStatus();
            telemetry.addData("LL Name",     status.getName());
            telemetry.addData("LL Temp/FPS", "%.1fC  %dFPS",
                    status.getTemp(), (int) status.getFps());
            telemetry.addData("Pipeline",    "Index:%d  Type:%s",
                    status.getPipelineIndex(), status.getPipelineType());
            telemetry.addLine("─────────────────────────────");

            // ── Raw result ───────────────────────────────────────────────────
            LLResult result = limelight.getLatestResult();

            if (result == null) {
                telemetry.addData("RESULT", "NULL — Limelight not sending data");
                telemetry.addData("Fix", "Check USB cable / power cycle Control Hub");

            } else {
                // Print everything so we can see what's coming through
                telemetry.addData("isValid",  result.isValid());
                telemetry.addData("TX",       "%.3f", result.getTx());
                telemetry.addData("TY",       "%.3f", result.getTy());
                telemetry.addData("TA",       "%.4f", result.getTa());
                telemetry.addLine("─────────────────────────────");

                // ── Fiducial (AprilTag) results ───────────────────────────────
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                telemetry.addData("Fiducials found", fiducials.size());

                if (fiducials.isEmpty()) {
                    telemetry.addData("TAGS", "NONE DETECTED");
                    telemetry.addData("Fix 1", "Pipeline type must be AprilTags");
                    telemetry.addData("Fix 2", "Tag family must be tag36h11");
                    telemetry.addData("Fix 3", "Point camera directly at tag");
                } else {
                    for (LLResultTypes.FiducialResult fid : fiducials) {
                        telemetry.addData("Fiducial ID", fid.getFiducialId());
                        telemetry.addData("Family",      fid.getFamily());
                        telemetry.addData("TX degrees",  "%.2f", fid.getTargetXDegrees());
                        telemetry.addData("TY degrees",  "%.2f", fid.getTargetYDegrees());
                        telemetry.addData("Area",        "%.4f", fid.getTargetArea());
                    }
                    telemetry.addLine("✓ Tag detected! M4Auto is ready.");
                }
            }

            telemetry.update();
        }

        limelight.stop();
    }
}