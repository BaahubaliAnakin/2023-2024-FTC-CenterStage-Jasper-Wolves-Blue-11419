package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous
public class AutoBlueFarCV extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Vision
    private PropDetectionProcessor propDetector;
    private VisionPortal visionPortal;

    PropDetectionProcessor.Location location;

    @Override
    public void runOpMode() throws InterruptedException {

        // ----- HARDWARE INITIALIZATION ---- //
        propDetector = new PropDetectionProcessor();
        propDetector.propColor = PropDetectionProcessor.Prop.BLUE;
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), propDetector);

        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "leftfrontmotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "leftbackmotor");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "rightfrontmotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "rightbackmotor");
        Servo as1 = hardwareMap.servo.get("as1");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, 62, 199.491);
        drive.setPoseEstimate(startPose);

        //  CENTER //

        Trajectory moveForwardCenter = drive.trajectoryBuilder(startPose)
                .forward(30.5)
                .build();

        Trajectory moveBackCenter = drive.trajectoryBuilder(moveForwardCenter.end())
                .back(14.5)
                .build();

        //   LEFT   //
        Trajectory moveForwardLeft = drive.trajectoryBuilder(startPose)
                .forward(28)
                .build();

        Trajectory moveStrafeLeft = drive.trajectoryBuilder(moveForwardLeft.end())
                .strafeLeft(10)
                .build();

        Trajectory moveBackLeft = drive.trajectoryBuilder(moveStrafeLeft.end())
                .back(4.5)
                .build();

        //   RIGHT  //
        Trajectory moveForwardRight = drive.trajectoryBuilder(startPose)
                .forward(28.5)
                .build();

        Trajectory moveStrafeRight = drive.trajectoryBuilder(moveForwardRight.end())
                .strafeRight(13.5)
                .build();

        Trajectory moveBackRight = drive.trajectoryBuilder(moveStrafeRight.end())
                .back(7)
                .build();


        // Start Code //



        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine(String.valueOf(propDetector.getLocation()));
            telemetry.update();

            location = propDetector.getLocation();
        }

        // ----- START PRESSED ----- //
        runtime.reset();

        if(isStopRequested()) return;


        // ----- RUNNING OP-MODE ----- //
        telemetry.addData("Location: ",location);
        telemetry.update();

        // prepare for movement (set arm to down, lift slide)
        visionPortal.close();

        if (location == PropDetectionProcessor.Location.Left) {
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardLeft);
            drive.followTrajectory(moveStrafeLeft);
            drive.followTrajectory(moveBackLeft);

        } else if (location == PropDetectionProcessor.Location.Center) {
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardCenter);
            drive.followTrajectory(moveBackCenter);

        } else if (location == PropDetectionProcessor.Location.Right) {
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardRight);
            drive.followTrajectory(moveStrafeRight);
            drive.followTrajectory(moveBackRight);

        }



    }
}
