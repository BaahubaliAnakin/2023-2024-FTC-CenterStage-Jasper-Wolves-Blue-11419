package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;



@Autonomous
public class AutoBlueCloseCVSpline60 extends LinearOpMode {

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

        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftfrontmotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftbackmotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightfrontmotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "rightbackmotor");
        Servo as1 = hardwareMap.servo.get("as1");
        Servo leftServo = hardwareMap.servo.get("leftarmservo");
        Servo leftClawServo = hardwareMap.servo.get("leftClawServo");
        Servo rightClawServo = hardwareMap.servo.get("rightClawServo");


        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(11.6, 62, 199.491);
        drive.setPoseEstimate(startPose);
        leftClawServo.setPosition(0.32);
        rightClawServo.setPosition(0.3);

        //  CENTER //

        Trajectory moveForwardCenter = drive.trajectoryBuilder(startPose)
                .forward(31.5)
                .build();

        Trajectory moveBackCenter = drive.trajectoryBuilder(moveForwardCenter.end())
                .back(14)
                .build();

        Trajectory moveSplineCenter = drive.trajectoryBuilder(moveBackCenter.end())
                .splineTo(new Vector2d(41, 34.425), Math.toRadians(0))
                .build();

        //Trajectory moveStrafeCenter2 = drive.trajectoryBuilder(moveBackCenter2.end())
        //        .strafeRight(16.5)
        //        .build();

        Trajectory moveForwardCenter2 = drive.trajectoryBuilder(moveSplineCenter.end())
                .forward(10.705)
                .build();

        Trajectory moveBackCenter4 = drive.trajectoryBuilder(moveForwardCenter2.end())
                .back(7)
                .build();

        Trajectory moveSplineCenter2 = drive.trajectoryBuilder(moveBackCenter4.end())
                .lineToSplineHeading(new Pose2d(8, 58, Math.toRadians(180)))
                .build();

        Trajectory moveForwardCenter3 = drive.trajectoryBuilder(moveSplineCenter2.end())
                .forward(47)
                .build();

        Trajectory moveLineCenter = drive.trajectoryBuilder(moveForwardCenter3.end())
                .lineToConstantHeading(new Vector2d(-51.5, 29.75))
                .build();

        Trajectory moveForwardCenter4 = drive.trajectoryBuilder(moveLineCenter.end())
                .forward(4.15)
                .build();

        Trajectory moveBackCenter5 = drive.trajectoryBuilder(moveForwardCenter4.end())
                .back(3)
                .build();

        Trajectory moveSplineCenter3 = drive.trajectoryBuilder(moveBackCenter5.end())
                .lineToSplineHeading(new Pose2d(-48, 10.3, Math.toRadians(0)))
                .build();

        Trajectory moveForwardCenter5 = drive.trajectoryBuilder(moveSplineCenter3.end())
                .forward(72)
                .build();

        Trajectory moveLineCenter2 = drive.trajectoryBuilder(moveForwardCenter5.end())
                .lineToConstantHeading(new Vector2d(46.5, 30.5))
                .build();

        Trajectory moveForwardCenter6 = drive.trajectoryBuilder(moveLineCenter2.end())
                .forward(5.15)
                .build();

        Trajectory moveStrafeCenter2 = drive.trajectoryBuilder(moveForwardCenter6.end())
                .strafeLeft(3.85)
                .build();

        Trajectory moveBackCenter3 = drive.trajectoryBuilder(moveStrafeCenter2.end())
                .back(7)
                .build();


        //   LEFT   //
        Trajectory moveForwardLeft = drive.trajectoryBuilder(startPose)
                .forward(27)
                .build();

        Trajectory moveStrafeLeft = drive.trajectoryBuilder(moveForwardLeft.end())
                .strafeLeft(9.45)
                .build();

        Trajectory moveBackLeft2 = drive.trajectoryBuilder(moveStrafeLeft.end())
                .back(20)
                .build();

        Trajectory moveSplineLeft = drive.trajectoryBuilder(moveBackLeft2.end())
                .lineToSplineHeading(new Pose2d(41, 41.5, Math.toRadians(0)))
                .build();

        Trajectory moveForwardLeft3 = drive.trajectoryBuilder(moveSplineLeft.end())
                .forward(10.3)
                .build();

        Trajectory moveBackLeft3 = drive.trajectoryBuilder(moveForwardLeft3.end())
                .back(7)
                .build();

        Trajectory moveSplineLeft2 = drive.trajectoryBuilder(moveBackLeft3.end())
                .lineToSplineHeading(new Pose2d(8, 58, Math.toRadians(180)))
                .build();

        Trajectory moveForwardLeft4 = drive.trajectoryBuilder(moveSplineLeft2.end())
                .forward(47)
                .build();

        Trajectory moveLineLeft = drive.trajectoryBuilder(moveForwardLeft4.end())
                .lineToConstantHeading(new Vector2d(-51.5, 29))
                .build();

        Trajectory moveForwardLeft5 = drive.trajectoryBuilder(moveLineLeft.end())
                .forward(4.25)
                .build();

        Trajectory moveBackLeft5 = drive.trajectoryBuilder(moveForwardLeft5.end())
                .back(1.15)
                .build();

        Trajectory moveSplineLeft3 = drive.trajectoryBuilder(moveBackLeft5.end())
                .lineToSplineHeading(new Pose2d(-48, 10.25, Math.toRadians(0)))
                .build();

        Trajectory moveForwardLeft6 = drive.trajectoryBuilder(moveSplineLeft3.end())
                .forward(72)
                .build();

        Trajectory moveLineLeft2 = drive.trajectoryBuilder(moveForwardLeft6.end())
                .lineToConstantHeading(new Vector2d(46.5, 32.25))
                .build();

        Trajectory moveForwardLeft7 = drive.trajectoryBuilder(moveLineLeft2.end())
                .forward(4.7)
                .build();

        Trajectory moveStrafeLeft2 = drive.trajectoryBuilder(moveForwardLeft7.end())
                .strafeLeft(4.3)
                .build();

        Trajectory moveBackLeft4 = drive.trajectoryBuilder(moveStrafeLeft2.end())
                .back(7)
                .build();


        //   RIGHT  //
        Trajectory moveForwardRight = drive.trajectoryBuilder(startPose)
                .forward(29.25)
                .build();

        Trajectory moveStrafeRight = drive.trajectoryBuilder(moveForwardRight.end())
                .strafeRight(14.85)
                .build();

        Trajectory moveBackRight = drive.trajectoryBuilder(moveStrafeRight.end())
                .back(4.65)
                .build();

        Trajectory moveStrafeRightLeft = drive.trajectoryBuilder(moveBackRight.end())
                .strafeLeft(18)
                .build();

        Trajectory moveSplineRight = drive.trajectoryBuilder(moveStrafeRightLeft.end())
                .splineTo(new Vector2d(51,27.5), Math.toRadians(0))
                .build();

        //Trajectory moveForwardRight2 = drive.trajectoryBuilder(moveSplineRight.end())
        //        .forward(10.5)
        //.build();

        Trajectory moveSplineRight2 = drive.trajectoryBuilder(moveSplineRight.end())
                .lineToSplineHeading(new Pose2d(8, 57.5, Math.toRadians(180)))
                .build();

        Trajectory moveForwardRight4 = drive.trajectoryBuilder(moveSplineRight2.end())
                .forward(32.95)
                .build();

        Trajectory moveSplineRight3 = drive.trajectoryBuilder(moveForwardRight4.end())
                .splineTo(new Vector2d(-46, 8.2), Math.toRadians(180))
                .build();

        //Trajectory moveSplineRight4 = drive.trajectoryBuilder(moveSplineRight3.end())
        //        .lineToSplineHeading(new Pose2d(-46, 8.15, Math.toRadians(180)))
        //        .build();

        Trajectory moveForwardRight5 = drive.trajectoryBuilder(moveSplineRight3.end())
                .forward(9)
                .build();

        Trajectory moveLineRight = drive.trajectoryBuilder(moveForwardRight5.end())
                .lineToConstantHeading(new Vector2d(-44, 11.75))
                .build();

        Trajectory moveBackRight5 = drive.trajectoryBuilder(moveLineRight.end())
                .back(72)
                .build();

        Trajectory moveSplineRight5 = drive.trajectoryBuilder(moveBackRight5.end())
                .lineToSplineHeading(new Pose2d(52, 35, Math.toRadians(0)))
                .build();

        //Trajectory moveForwardRight7 = drive.trajectoryBuilder(moveSplineRight5.end())
        //.forward(3.12)
         //       .build();

        Trajectory moveStrafeRight2 = drive.trajectoryBuilder(moveSplineRight5.end())
                .strafeLeft(4)
                .build();

        Trajectory moveBackRight4 = drive.trajectoryBuilder(moveStrafeRight2.end())
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

        if (location == PropDetectionProcessor.Location.Left) {
            leftClawServo.setPosition(0.32);
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardLeft);
            drive.followTrajectory(moveStrafeLeft);
            drive.followTrajectory(moveBackLeft2);
            drive.followTrajectory(moveSplineLeft);
            sleep(50);
            drive.followTrajectory(moveForwardLeft3);
            leftClawServo.setPosition(0.32);
            leftServo.setPosition(0.7575);
            sleep(750);
            leftClawServo.setPosition(0.5);
            sleep(500);
            drive.followTrajectory(moveBackLeft3);
            sleep(350);
            leftClawServo.setPosition(0.35);
            sleep(200);
            leftServo.setPosition(0.5);
            sleep(750);
            leftServo.setPosition(0.76);
            drive.followTrajectory(moveSplineLeft2);
            drive.followTrajectory(moveForwardLeft4);
            leftServo.setPosition(1.0);
            rightClawServo.setPosition(0.15);
            drive.followTrajectory(moveLineLeft);
            drive.followTrajectory(moveForwardLeft5);
            rightClawServo.setPosition(0.375);
            sleep(200);
            drive.turn(Math.toRadians(-17));
            drive.followTrajectory(moveBackLeft5);
            sleep(75);
            leftServo.setPosition(0.65);
            drive.followTrajectory(moveSplineLeft3);
            leftServo.setPosition(0.99);
            drive.followTrajectory(moveForwardLeft6);
            leftServo.setPosition(0.73);
            drive.followTrajectory(moveLineLeft2);
            drive.followTrajectory(moveForwardLeft7);
            rightClawServo.setPosition(0.4);
            sleep(750);
            rightClawServo.setPosition(0.2);
            sleep(75);
            rightClawServo.setPosition(0.4);
            sleep(350);
            drive.followTrajectory(moveStrafeLeft2);
            rightClawServo.setPosition(0.2);
            drive.followTrajectory(moveBackLeft4);
            sleep(200);
            rightClawServo.setPosition(0.4);
            sleep(350);
            leftServo.setPosition(0.5);
            sleep(550);
            // drive.followTrajectory(moveStrafeLeft3);
            // drive.followTrajectory(moveBackLeft4);


        } else if (location == PropDetectionProcessor.Location.Center) {
            leftClawServo.setPosition(0.32);
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardCenter);
            drive.followTrajectory(moveBackCenter);
            drive.followTrajectory(moveSplineCenter);
            sleep(45);
            drive.followTrajectory(moveForwardCenter2);
            leftClawServo.setPosition(0.32);
            leftServo.setPosition(0.76);
            sleep(750);
            leftClawServo.setPosition(0.5);
            sleep(350);
            drive.followTrajectory(moveBackCenter4);
            sleep(200);
            leftClawServo.setPosition(0.4);
            sleep(350);
            leftServo.setPosition(0.5);
            sleep(550);
            leftServo.setPosition(0.755);
            drive.followTrajectory(moveSplineCenter2);
            leftServo.setPosition(1.0);
            drive.followTrajectory(moveForwardCenter3);
            leftServo.setPosition(1.0);
            rightClawServo.setPosition(0.15);
            drive.followTrajectory(moveLineCenter);
            drive.followTrajectory(moveForwardCenter4);
            rightClawServo.setPosition(0.375);
            sleep(200);
            drive.turn(Math.toRadians(-17));
            drive.followTrajectory(moveBackCenter5);
            sleep(75);
            leftServo.setPosition(0.65);
            drive.followTrajectory(moveSplineCenter3);
            leftServo.setPosition(0.99);
            drive.followTrajectory(moveForwardCenter5);
            leftServo.setPosition(0.73);
            drive.followTrajectory(moveLineCenter2);
            drive.followTrajectory(moveForwardCenter6);
            rightClawServo.setPosition(0.4);
            sleep(750);
            rightClawServo.setPosition(0.2);
            sleep(75);
            rightClawServo.setPosition(0.4);
            sleep(350);
            drive.followTrajectory(moveStrafeCenter2);
            rightClawServo.setPosition(0.2);
            drive.followTrajectory(moveBackCenter3);
            sleep(200);
            rightClawServo.setPosition(0.4);
            sleep(350);
            leftServo.setPosition(0.5);
            sleep(550);
            //drive.followTrajectory(moveStrafeCenter3);
            //drive.followTrajectory(moveForwardCenter4);

        } else if (location == PropDetectionProcessor.Location.Right) {
            leftClawServo.setPosition(0.32);
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardRight);
            drive.followTrajectory(moveStrafeRight);
            drive.followTrajectory(moveBackRight);
            drive.followTrajectory(moveStrafeRightLeft);
            leftServo.setPosition(0.7575);
            drive.followTrajectory(moveSplineRight);
            leftClawServo.setPosition(0.32);
            sleep(575);
            leftClawServo.setPosition(0.5);
            sleep(475);
            leftClawServo.setPosition(0.35);
            sleep(575);
            leftServo.setPosition(0.76);
            drive.followTrajectory(moveSplineRight2);
            drive.followTrajectory(moveForwardRight4);
            drive.followTrajectory(moveSplineRight3);
            leftServo.setPosition(1.0);
            //drive.followTrajectory(moveSplineRight4);
            rightClawServo.setPosition(0.125);
            drive.followTrajectory(moveForwardRight5);
            rightClawServo.setPosition(0.395);
            sleep(350);
            drive.turn(Math.toRadians(17));
            drive.followTrajectory(moveLineRight);
            drive.followTrajectory(moveBackRight5);
            leftServo.setPosition(0.73);
            drive.followTrajectory(moveSplineRight5);
            rightClawServo.setPosition(0.375);
            sleep(75);
            rightClawServo.setPosition(0.2);
            sleep(75);
            rightClawServo.setPosition(0.4);
            sleep(175);
            drive.followTrajectory(moveStrafeRight2);
            rightClawServo.setPosition(0.2);
            sleep(200);
            leftServo.setPosition(0.5);
            drive.followTrajectory(moveBackRight4);
            // drive.followTrajectory(moveStrafeRight3);
            // drive.followTrajectory(moveBackRight4);
        }
        telemetry.addData("Location: ",location);
        telemetry.update();

        // prepare for movement (set arm to down, lift slide)
        visionPortal.close();


    }
}
