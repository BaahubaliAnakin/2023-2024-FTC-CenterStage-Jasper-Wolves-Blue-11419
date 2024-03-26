package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "PropDetectionTest2")
public class PropDetectionTest2 extends LinearOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private org.firstinspires.ftc.teamcode.PropDetectionProcessor propDetector;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        propDetector = new org.firstinspires.ftc.teamcode.PropDetectionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), propDetector);

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Wait for the DS start button to be touched.``
        waitForStart();

        if (opModeIsActive()) {
            visionPortal.getProcessorEnabled(propDetector);
            telemetry.addLine(String.valueOf(propDetector.getLocation()));
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }
}
