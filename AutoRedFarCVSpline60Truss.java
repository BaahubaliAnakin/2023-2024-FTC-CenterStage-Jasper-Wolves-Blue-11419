package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous
public class AutoRedFarCVSpline60Truss extends LinearOpMode {

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
        Servo rightClawServo = hardwareMap.servo.get("rightClawServo");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -62, -199.491);
        drive.setPoseEstimate(startPose);
        leftClawServo.setPosition(0.32);
        rightClawServo.setPosition(0.3);

        //  CENTER //
        // Trajectory moveForwardStrafeRightCenter = drive.trajectoryBuilder(startPose)
        //        .strafeLeft(3)
        //       .build();

        Trajectory moveForwardCenter = drive.trajectoryBuilder(startPose)
                .forward(31.95)
                .build();

        Trajectory moveBackCenter = drive.trajectoryBuilder(moveForwardCenter.end())
                .lineToSplineHeading(new Pose2d(-36, -59.5, Math.toRadians(0)))
                .build();

        //Trajectory moveStrafeLeft2 = drive.trajectoryBuilder(moveBackCenter.end())
        //        .strafeLeft(20)
        //        .build();
        Trajectory moveForwardCenter4 = drive.trajectoryBuilder(moveBackCenter.end())
                .forward(55)
                .build();

        Trajectory moveSplineCenter = drive.trajectoryBuilder(moveForwardCenter4.end())
                .lineToSplineHeading(new Pose2d(53.4, -36.85, Math.toRadians(0)))
                .build();

        Trajectory moveSplineCenter2 = drive.trajectoryBuilder(moveSplineCenter.end())
                .lineToSplineHeading(new Pose2d(8, -59.45, Math.toRadians(180)))
                .build();

        Trajectory moveForwardCenter3 = drive.trajectoryBuilder(moveSplineCenter2.end())
                .forward(47)
                .build();

        Trajectory moveLineCenter = drive.trajectoryBuilder(moveForwardCenter3.end())
                .lineToConstantHeading(new Vector2d(-51.5, -33.95))
                .build();

        Trajectory moveForwardCenter5 = drive.trajectoryBuilder(moveLineCenter.end())
                .forward(4.15)
                .build();

        Trajectory moveBackCenter5 = drive.trajectoryBuilder(moveForwardCenter5.end())
                .back(3)
                .build();

        Trajectory moveSplineCenter3 = drive.trajectoryBuilder(moveBackCenter5.end())
                .lineToSplineHeading(new Pose2d(-48, -10.3, Math.toRadians(0)))
                .build();

        Trajectory moveForwardCenter6 = drive.trajectoryBuilder(moveSplineCenter3.end())
                .forward(72)
                .build();

        Trajectory moveLineCenter2 = drive.trajectoryBuilder(moveForwardCenter6.end())
                .lineToConstantHeading(new Vector2d(52.1, -30.5))
                .build();

        Trajectory moveStrafeCenter2 = drive.trajectoryBuilder(moveLineCenter2.end())
                .strafeRight(3.5)
                .build();

        Trajectory moveBackCenter3 = drive.trajectoryBuilder(moveStrafeCenter2.end())
                .back(7)
                .build();



        //   LEFT   //
        Trajectory moveForwardLeft = drive.trajectoryBuilder(startPose)
                .forward(26.5)
                .build();
        Trajectory moveStrafeLeft = drive.trajectoryBuilder(moveForwardLeft.end())
                .strafeLeft(9)
                .build();
        Trajectory moveBackLeft = drive.trajectoryBuilder(moveStrafeLeft.end())
                .lineToSplineHeading(new Pose2d(-37, -59.5, Math.toRadians(0)))
                .build();
        //Trajectory moveStrafeRightLeft = drive.trajectoryBuilder(moveBackLeft.end())
        //        .strafeRight(14.5)
        //        .build();
        Trajectory moveForwardLeft4 = drive.trajectoryBuilder(moveBackLeft.end())
                .forward(50)
                .build();
        Trajectory moveLineLeft = drive.trajectoryBuilder(moveForwardLeft4.end())
                .lineToConstantHeading(new Vector2d(52, -29))
                .build();
        Trajectory moveSplineLeft2 = drive.trajectoryBuilder(moveLineLeft.end())
                .lineToSplineHeading(new Pose2d(8, -59.175, Math.toRadians(180)))
                .build();

        Trajectory moveForwardLeft5 = drive.trajectoryBuilder(moveSplineLeft2.end())
                .forward(35)
                .build();

        Trajectory moveSplineLeft3 = drive.trajectoryBuilder(moveForwardLeft5.end())
                .splineTo(new Vector2d(-46, -10.65), Math.toRadians(180))
                .build();

        //Trajectory moveSplineRight4 = drive.trajectoryBuilder(moveSplineRight3.end())
        //        .lineToSplineHeading(new Pose2d(-46, 8.15, Math.toRadians(180)))
        //        .build();

        Trajectory moveForwardLeft6 = drive.trajectoryBuilder(moveSplineLeft3.end())
                .forward(9.52)
                .build();

        Trajectory moveLineLeft2 = drive.trajectoryBuilder(moveForwardLeft6.end())
                .lineToConstantHeading(new Vector2d(-44, -12.125))
                .build();

        Trajectory moveBackLeft5 = drive.trajectoryBuilder(moveLineLeft2.end())
                .back(72)
                .build();

        Trajectory moveSplineLeft5 = drive.trajectoryBuilder(moveBackLeft5.end())
                .lineToSplineHeading(new Pose2d(52.3, -37, Math.toRadians(0)))
                .build();

        //Trajectory moveForwardRight7 = drive.trajectoryBuilder(moveSplineRight5.end())
        //.forward(3.12)
        //       .build();

        Trajectory moveStrafeLeft2 = drive.trajectoryBuilder(moveSplineLeft5.end())
                .strafeRight(4)
                .build();

        Trajectory moveBackLeft4 = drive.trajectoryBuilder(moveStrafeLeft2.end())
                .back(7)
                .build();

        //   RIGHT  //
        Trajectory moveForwardRight = drive.trajectoryBuilder(startPose, true)
                .forward(28.5)
                .build();

        Trajectory moveStrafeRight = drive.trajectoryBuilder(moveForwardRight.end())
                .strafeRight(13.5)
                .build();

        Trajectory moveBackRight = drive.trajectoryBuilder(moveStrafeRight.end())
                .lineToSplineHeading(new Pose2d(-37, -59.5, Math.toRadians(0)))
                .build();

        Trajectory moveForwardRight5 = drive.trajectoryBuilder(moveBackRight.end())
                .forward(61)
                .build();

        Trajectory moveSplineRight = drive.trajectoryBuilder(moveBackRight.end())
                .lineToConstantHeading(new Vector2d(52, -43.65))
                .build();

        Trajectory moveSplineRight2 = drive.trajectoryBuilder(moveSplineRight.end())
                .lineToSplineHeading(new Pose2d(8, -59.175, Math.toRadians(180)))
                .build();

        Trajectory moveForwardRight4 = drive.trajectoryBuilder(moveSplineRight2.end())
                .forward(47)
                .build();

        Trajectory moveLineRight = drive.trajectoryBuilder(moveForwardRight4.end())
                .lineToConstantHeading(new Vector2d(-51.5, -33.95))
                .build();

        Trajectory moveForwardRight6 = drive.trajectoryBuilder(moveLineRight.end())
                .forward(4.25)
                .build();

        Trajectory moveBackRight5 = drive.trajectoryBuilder(moveForwardRight6.end())
                .back(3)
                .build();

        Trajectory moveSplineRight3 = drive.trajectoryBuilder(moveBackRight5.end())
                .lineToSplineHeading(new Pose2d(-48, -10.85, Math.toRadians(0)))
                .build();

        Trajectory moveForwardRight7 = drive.trajectoryBuilder(moveSplineRight3.end())
                .forward(72)
                .build();

        Trajectory moveLineRight2 = drive.trajectoryBuilder(moveForwardRight7.end())
                .lineToConstantHeading(new Vector2d(53.25, -32))
                .build();

        Trajectory moveStrafeRight2 = drive.trajectoryBuilder(moveLineRight2.end())
                .strafeLeft(3.1)
                .build();

        Trajectory moveBackRight4 = drive.trajectoryBuilder(moveStrafeRight2.end())
                .back(8.5)
                .build();


        {
        }




        // Start Code //



        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine(String.valueOf(propDetector.getLocation()));
            telemetry.update();

            location = propDetector.getLocation();
        }

        // ----- START PRESSED ----- //
        runtime.reset();

        waitForStart();

        if(isStopRequested()) return;


        // ----- RUNNING OP-MODE ----- //
        visionPortal.close();

        if (location == PropDetectionProcessor.Location.Left) {
            leftClawServo.setPosition(0.32);
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardLeft);
            drive.followTrajectory(moveBackLeft);
            leftServo.setPosition(0.76);
            //leftServo.setPosition(0.33);
            //clawServo.setPosition(0.4);
            sleep(100);
            //sleep(1000);
            //clawServo.setPosition(1.0);
            drive.followTrajectory(moveForwardLeft4);
            leftServo.setPosition(0.7325);
            drive.followTrajectory(moveLineLeft);
            leftClawServo.setPosition(0.32);
            sleep(350);
            leftClawServo.setPosition(0.5);
            sleep(750);
            leftServo.setPosition(0.5);
            sleep(100);
            leftClawServo.setPosition(0.32);
            sleep(550);
            drive.followTrajectory(moveSplineLeft2);
            drive.followTrajectory(moveForwardLeft5);
            leftServo.setPosition(1.0);
            leftClawServo.setPosition(0.55);
            drive.followTrajectory(moveLineLeft2);
            drive.followTrajectory(moveForwardLeft6);
            leftClawServo.setPosition(0.3);
            sleep(200);
            drive.turn(Math.toRadians(-19));
            drive.followTrajectory(moveBackLeft5);
            sleep(75);
            leftServo.setPosition(0.65);
            drive.followTrajectory(moveSplineLeft3);
            leftServo.setPosition(0.99);
            drive.followTrajectory(moveForwardLeft6);
            leftServo.setPosition(0.755);
            leftClawServo.setPosition(0.32);
            sleep(675);
            leftClawServo.setPosition(0.5);
            sleep(80);
            leftClawServo.setPosition(0.32);
            sleep(295);
            drive.followTrajectory(moveStrafeLeft2);
            leftClawServo.setPosition(0.5);
            drive.followTrajectory(moveBackLeft4);
            sleep(200);
            leftClawServo.setPosition(0.32);
            sleep(350);
            leftServo.setPosition(0.5);
            sleep(550);


        } else if (location == PropDetectionProcessor.Location.Center) {
            leftClawServo.setPosition(0.32);
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardCenter);
            drive.followTrajectory(moveBackCenter);
            leftServo.setPosition(0.76);
            //leftServo.setPosition(0.33);
            //clawServo.setPosition(0.4);
            sleep(100);
            //sleep(1000);
            //clawServo.setPosition(1.0);
            drive.followTrajectory(moveForwardCenter4);
            leftServo.setPosition(0.7325);
            drive.followTrajectory(moveSplineCenter);
            leftClawServo.setPosition(0.32);
            sleep(350);
            leftClawServo.setPosition(0.5);
            sleep(750);
            leftServo.setPosition(0.5);
            sleep(100);
            leftClawServo.setPosition(0.32);
            sleep(550);
            leftServo.setPosition(0.76);
            drive.followTrajectory(moveSplineCenter2);
            drive.followTrajectory(moveForwardCenter3);
            leftServo.setPosition(1.0);
            leftClawServo.setPosition(0.55);
            drive.followTrajectory(moveLineCenter);
            drive.followTrajectory(moveForwardCenter5);
            leftClawServo.setPosition(0.3);
            sleep(200);
            drive.turn(Math.toRadians(-19));
            drive.followTrajectory(moveBackCenter5);
            sleep(75);
            leftServo.setPosition(0.65);
            drive.followTrajectory(moveSplineCenter3);
            leftServo.setPosition(0.99);
            drive.followTrajectory(moveForwardCenter6);
            leftServo.setPosition(0.755);
            drive.followTrajectory(moveLineCenter2);
            leftClawServo.setPosition(0.32);
            sleep(675);
            leftClawServo.setPosition(0.5);
            sleep(75);
            leftClawServo.setPosition(0.32);
            sleep(295);
            drive.followTrajectory(moveStrafeCenter2);
            leftClawServo.setPosition(0.5);
            drive.followTrajectory(moveBackCenter3);
            sleep(200);
            leftClawServo.setPosition(0.32);
            sleep(350);
            leftServo.setPosition(0.5);
            sleep(550);
            //drive.followTrajectory(moveStrafeLeftCenter3);
            //drive.followTrajectory(moveBackCenter4);

        } else if (location == PropDetectionProcessor.Location.Right) {
            leftClawServo.setPosition(0.32);
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardRight);
            drive.followTrajectory(moveBackCenter);
            leftServo.setPosition(0.76);
            //leftServo.setPosition(0.33);
            //clawServo.setPosition(0.4);
            sleep(100);
            //sleep(1000);
            //clawServo.setPosition(1.0);
            drive.followTrajectory(moveForwardRight5);
            leftServo.setPosition(0.7325);
            drive.followTrajectory(moveSplineRight);
            leftClawServo.setPosition(0.32);
            sleep(350);
            leftClawServo.setPosition(0.5);
            sleep(750);
            leftServo.setPosition(0.5);
            sleep(100);
            leftClawServo.setPosition(0.32);
            sleep(550);
            drive.followTrajectory(moveSplineRight2);
            drive.followTrajectory(moveForwardRight4);
            leftServo.setPosition(1.0);
            leftClawServo.setPosition(0.55);
            drive.followTrajectory(moveLineRight);
            drive.followTrajectory(moveForwardRight6);
            leftClawServo.setPosition(0.3);
            sleep(200);
            drive.turn(Math.toRadians(-19));
            drive.followTrajectory(moveBackRight5);
            sleep(75);
            leftServo.setPosition(0.65);
            drive.followTrajectory(moveSplineRight3);
            leftServo.setPosition(0.99);
            drive.followTrajectory(moveForwardRight7);
            leftServo.setPosition(0.755);
            drive.followTrajectory(moveLineRight2);
            leftClawServo.setPosition(0.32);
            sleep(675);
            leftClawServo.setPosition(0.5);
            sleep(80);
            leftClawServo.setPosition(0.32);
            sleep(295);
            drive.followTrajectory(moveStrafeRight2);
            leftClawServo.setPosition(0.5);
            drive.followTrajectory(moveBackRight4);
            sleep(200);
            leftClawServo.setPosition(0.32);
            sleep(350);
            leftServo.setPosition(0.5);
            sleep(550);
            //drive.followTrajectory(moveStrafeRight3);
        }

        telemetry.addData("Location: ",location);
        telemetry.update();

        // prepare for movement (set arm to down, lift slide)


    }
}
