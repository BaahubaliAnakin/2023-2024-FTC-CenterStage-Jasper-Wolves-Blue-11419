package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class AutoRedCloseCVSpline extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Vision
    private PropDetectionProcessor propDetector;
    private VisionPortal visionPortal;

    PropDetectionProcessor.Location location;

    @Override
    public void runOpMode() throws InterruptedException {

        // ----- HARDWARE INITIALIZATION ---- //
        propDetector = new PropDetectionProcessor();
        propDetector.propColor = PropDetectionProcessor.Prop.RED;
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), propDetector);

        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "leftfrontmotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "leftbackmotor");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "rightfrontmotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "rightbackmotor");
        Servo as1 = hardwareMap.servo.get("as1");
        Servo leftServo = hardwareMap.servo.get("leftarmservo");
        Servo leftClawServo = hardwareMap.servo.get("leftClawServo");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(11.6, -62, -199.491);
        drive.setPoseEstimate(startPose);
        leftClawServo.setPosition(0.32);


        //  CENTER //

        Trajectory moveForwardCenter = drive.trajectoryBuilder(startPose)
                .forward(32.5)
                .build();

        Trajectory moveBackCenter = drive.trajectoryBuilder(moveForwardCenter.end())
                .back(16)
                .build();

        Trajectory moveSplineCenter = drive.trajectoryBuilder(moveBackCenter.end())
                .splineTo(new Vector2d(41, -37), Math.toRadians(0))
                .build();
        
        //Trajectory moveStrafeCenter2 = drive.trajectoryBuilder(moveBackCenter2.end())
        //        .strafeRight(16.5)
        //        .build();

        Trajectory moveForwardCenter2 = drive.trajectoryBuilder(moveSplineCenter.end())
                .forward(11)
                .build();

        Trajectory moveBackCenter4 = drive.trajectoryBuilder(moveForwardCenter2.end())
                .back(7)
                .build();

        Trajectory moveStrafeCenter3 = drive.trajectoryBuilder(moveBackCenter4.end())
                .strafeRight(25)
                .build();

        Trajectory moveForwardCenter4 = drive.trajectoryBuilder(moveStrafeCenter3.end())
                .forward(14)
                .build();


        //   LEFT   //
        Trajectory moveForwardLeft = drive.trajectoryBuilder(startPose)
                .forward(28.15)
                .build();

        Trajectory moveStrafeLeft = drive.trajectoryBuilder(moveForwardLeft.end())
                .strafeLeft(9.8)
                .build();

        Trajectory moveBackLeft = drive.trajectoryBuilder(moveStrafeLeft.end())
                .back(4.65)
                .build();

        Trajectory moveStrafeRightLeft = drive.trajectoryBuilder(moveBackLeft.end())
                .strafeRight(18)
                .build();

        Trajectory moveBackLeft2 = drive.trajectoryBuilder(moveStrafeRightLeft.end())
                .back(8)
                .build();

        Trajectory moveSplineLeft = drive.trajectoryBuilder(moveBackLeft2.end())
                .splineTo(new Vector2d(41, -29), Math.toRadians(0))
                .build();

        Trajectory moveForwardLeft3 = drive.trajectoryBuilder(moveSplineLeft.end())
                .forward(11)
                .build();

        Trajectory moveBackLeft3 = drive.trajectoryBuilder(moveForwardLeft3.end())
                .back(6)
                .build();

        Trajectory moveStrafeLeft3 = drive.trajectoryBuilder(moveBackLeft3.end())
                .strafeRight(31)
                .build();

        Trajectory moveBackLeft4 = drive.trajectoryBuilder(moveStrafeLeft3.end())
                .forward(14)
                .build();

        //   RIGHT  //
        Trajectory moveForwardRight = drive.trajectoryBuilder(startPose)
                .forward(25)
                .build();

        Trajectory moveStrafeRight = drive.trajectoryBuilder(moveForwardRight.end())
                .strafeRight(14.5)
                .build();

        Trajectory moveBackRight = drive.trajectoryBuilder(moveStrafeRight.end())
                .back(20)
                .build();

        Trajectory moveSplineRight = drive.trajectoryBuilder(moveBackRight.end())
                .lineToSplineHeading(new Pose2d(41, -43.65, Math.toRadians(0)))
                .build();

        Trajectory moveForwardRight2 = drive.trajectoryBuilder(moveSplineRight.end())
                .forward(11)
                .build();

        Trajectory moveBackRight3 = drive.trajectoryBuilder(moveForwardRight2.end())
                .back(7)
                .build();

        Trajectory moveStrafeRight3 = drive.trajectoryBuilder(moveBackRight3.end())
                .strafeRight(18)
                .build();

        Trajectory moveBackRight4 = drive.trajectoryBuilder(moveStrafeRight3.end())
                .forward(14)
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

        if (location == PropDetectionProcessor.Location.Left) {
            leftClawServo.setPosition(0.31);
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardLeft);
            drive.followTrajectory(moveStrafeLeft);
            drive.followTrajectory(moveBackLeft);
            drive.followTrajectory(moveStrafeRightLeft);
            drive.followTrajectory(moveBackLeft2);
            drive.followTrajectory(moveSplineLeft);
            sleep(50);
            drive.followTrajectory(moveForwardLeft3);
            leftClawServo.setPosition(0.32);
            leftServo.setPosition(0.755);
            sleep(750);
            leftClawServo.setPosition(0.5);
            sleep(350);
            drive.followTrajectory(moveBackLeft3);
            sleep(350);
            leftClawServo.setPosition(0.35);
            sleep(200);
            leftServo.setPosition(0.5);
            sleep(750);
            drive.followTrajectory(moveStrafeLeft3);
            drive.followTrajectory(moveBackLeft4);


        } else if (location == PropDetectionProcessor.Location.Center) {
            leftClawServo.setPosition(0.31);
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardCenter);
            drive.followTrajectory(moveBackCenter);
            drive.followTrajectory(moveSplineCenter);
            sleep(50);
            drive.followTrajectory(moveForwardCenter2);
            leftClawServo.setPosition(0.32);
            leftServo.setPosition(0.755);
            sleep(750);
            leftClawServo.setPosition(0.5);
            sleep(350);
            drive.followTrajectory(moveBackCenter4);
            sleep(200);
            leftClawServo.setPosition(0.35);
            sleep(350);
            leftServo.setPosition(0.5);
            sleep(750);
            drive.followTrajectory(moveStrafeCenter3);
            drive.followTrajectory(moveForwardCenter4);

        } else if (location == PropDetectionProcessor.Location.Right) {
            leftClawServo.setPosition(0.31);
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardRight);
            drive.followTrajectory(moveStrafeRight);
            drive.followTrajectory(moveBackRight);
            drive.followTrajectory(moveSplineRight);
            sleep(50);
            drive.followTrajectory(moveForwardRight2);
            leftClawServo.setPosition(0.32);
            leftServo.setPosition(0.755);
            sleep(750);
            leftClawServo.setPosition(0.5);
            sleep(350);
            drive.followTrajectory(moveBackRight3);
            sleep(200);
            leftClawServo.setPosition(0.35);
            sleep(350);
            leftServo.setPosition(0.5);
            sleep(750);
            drive.followTrajectory(moveStrafeRight3);
            drive.followTrajectory(moveBackRight4);
        }
        telemetry.addData("Location: ",location);
        telemetry.update();

        // prepare for movement (set arm to down, lift slide)
        visionPortal.close();


    }
}
