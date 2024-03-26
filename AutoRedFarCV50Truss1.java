package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class AutoRedFarCV50Truss1 extends LinearOpMode {

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
                .forward(32.5)
                .build();
        Trajectory moveBackCenter = drive.trajectoryBuilder(moveForwardCenter.end())
                .back(10)
                .build();
        //Trajectory moveStrafeLeft2 = drive.trajectoryBuilder(moveBackCenter.end())
        //        .strafeLeft(20)
        //        .build();
        Trajectory moveForwardCenter2 = drive.trajectoryBuilder(moveBackCenter.end())
                .back(19)
                .build();
        Trajectory moveForwardCenter4 = drive.trajectoryBuilder(new Pose2d(moveForwardCenter2.end().getX(), moveForwardCenter2.end().getY(), Math.toRadians(0)))
                .forward(70)
                .build();
        Trajectory moveStrafeRightCenter2 = drive.trajectoryBuilder(moveForwardCenter4.end())
                .strafeLeft(18)
                .build();
        Trajectory moveBackCenter3 = drive.trajectoryBuilder(moveStrafeRightCenter2.end())
                .forward(14)
                .build();
        Trajectory moveStrafeLeftCenter4 = drive.trajectoryBuilder(moveBackCenter3.end())
                .strafeLeft(4.25)
                .build();
        Trajectory moveBackCenter5 = drive.trajectoryBuilder(moveStrafeLeftCenter4.end())
                .forward(4.9)
                .build();
        Trajectory moveForwardCenter5 = drive.trajectoryBuilder(moveStrafeLeftCenter4.end())
                .back(2)
                .build();
        Trajectory moveStrafeRightCenter3 = drive.trajectoryBuilder(moveForwardCenter5.end())
                .strafeRight(13)
                .build();
        Trajectory moveBackCenter4 = drive.trajectoryBuilder(moveStrafeRightCenter3.end())
                .forward(10)
                .build();


        //   LEFT   //
        Trajectory moveForwardLeft = drive.trajectoryBuilder(startPose)
                .forward(26.5)
                .build();
        Trajectory moveStrafeLeft = drive.trajectoryBuilder(moveForwardLeft.end())
                .strafeLeft(9)
                .build();
        Trajectory moveBackLeft = drive.trajectoryBuilder(moveStrafeLeft.end())
                .back(10)
                .build();
        //Trajectory moveStrafeRightLeft = drive.trajectoryBuilder(moveBackLeft.end())
        //        .strafeRight(14.5)
        //        .build();
        Trajectory moveForwardLeft2 = drive.trajectoryBuilder(moveBackLeft.end())
                .back(12.5)
                .build();
        Trajectory moveForwardLeft4 = drive.trajectoryBuilder(new Pose2d(moveForwardLeft2.end().getX(), moveForwardLeft2.end().getY(), Math.toRadians(0)))
                .forward(80)
                .build();
        Trajectory moveStrafeRightLeft2 = drive.trajectoryBuilder(moveForwardLeft4.end())
                .strafeLeft(28.7)
                .build();
        Trajectory moveBackLeft3 = drive.trajectoryBuilder(moveStrafeRightLeft2.end())
                .forward(17.8)
                .build();
        Trajectory moveBackLeft5 = drive.trajectoryBuilder(moveBackLeft3.end())
                .forward(4.25)
                .build();
        Trajectory moveForwardLeft5 = drive.trajectoryBuilder(moveBackLeft3.end())
                .back(4)
                .build();
        Trajectory moveStrafeRightLeft3 = drive.trajectoryBuilder(moveForwardLeft5.end())
                .strafeLeft(10)
                .build();
        Trajectory moveBackLeft4 = drive.trajectoryBuilder(moveStrafeRightLeft3.end())
                .forward(10)
                .build();

        //   RIGHT  //
        Trajectory moveForwardRight = drive.trajectoryBuilder(startPose, true)
                .forward(28.5)
                .build();
        Trajectory moveStrafeRight = drive.trajectoryBuilder(moveForwardRight.end())
                .strafeRight(14.8)
                .build();
        Trajectory moveBackRight = drive.trajectoryBuilder(moveStrafeRight.end())
                .back(4.5)
                .build();
        Trajectory moveStrafeLeftRight = drive.trajectoryBuilder(moveBackRight.end())
                .strafeLeft(14.8)
                .build();
        Trajectory moveForwardRight4 = drive.trajectoryBuilder(moveStrafeLeftRight.end())
                .back(20)
                .build();
        Trajectory moveForwardRight5 = drive.trajectoryBuilder(new Pose2d(moveForwardRight4.end().getX(), moveForwardRight4.end().getY(), Math.toRadians(0)))
                .forward(70)
                .build();
        Trajectory moveStrafeRight2 = drive.trajectoryBuilder(moveForwardRight5.end())
                .strafeLeft(16.35)
                .build();
        Trajectory moveBackRight3 = drive.trajectoryBuilder(moveStrafeRight2.end())
                .forward(18)
                .build();
        Trajectory moveBackRight5 = drive.trajectoryBuilder(moveBackRight3.end())
                .forward(1.25)
                .build();
        Trajectory moveForwardRight6 = drive.trajectoryBuilder(moveBackRight5.end())
                .back(8)
                .build();
        Trajectory moveStrafeRight3 = drive.trajectoryBuilder(moveForwardRight6.end())
                .strafeRight(24.5)
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
            drive.followTrajectory(moveStrafeLeft);
            drive.followTrajectory(moveBackLeft);
            //leftServo.setPosition(0.33);
            drive.followTrajectory(moveForwardLeft2);
            //clawServo.setPosition(0.4);
            drive.turn(Math.toRadians(-90));
            leftServo.setPosition(0.7275);
            sleep(9500);
            //sleep(1000);
            //clawServo.setPosition(1.0);
            drive.followTrajectory(moveForwardLeft4);
            drive.followTrajectory(moveStrafeRightLeft2);
            drive.followTrajectory(moveBackLeft3);
            leftClawServo.setPosition(0.32);
            sleep(350);
            leftClawServo.setPosition(0.5);
            sleep(750);
            leftServo.setPosition(0.5);
            sleep(100);
            leftClawServo.setPosition(0.32);
            sleep(550);
            drive.followTrajectory(moveForwardLeft5);
            leftClawServo.setPosition(0.32);
            sleep(100);
            leftServo.setPosition(0.4);

        } else if (location == PropDetectionProcessor.Location.Center) {
            leftClawServo.setPosition(0.32);
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardCenter);
            drive.followTrajectory(moveBackCenter);
            //leftServo.setPosition(0.33);
            drive.followTrajectory(moveForwardCenter2);
            //clawServo.setPosition(0.4);
            drive.turn(Math.toRadians(-90));
            leftServo.setPosition(0.7275);
            sleep(9500);
            //sleep(1000);
            //clawServo.setPosition(1.0);
            drive.followTrajectory(moveForwardCenter4);
            drive.followTrajectory(moveStrafeRightCenter2);
            drive.followTrajectory(moveBackCenter3);
            drive.followTrajectory(moveStrafeLeftCenter4);
            drive.followTrajectory(moveBackCenter5);
            sleep(750);
            leftClawServo.setPosition(0.32);
            sleep(350);
            leftClawServo.setPosition(0.5);
            sleep(750);
            leftServo.setPosition(0.5);
            sleep(100);
            leftClawServo.setPosition(0.32);
            sleep(550);
            drive.followTrajectory(moveForwardCenter5);
            leftClawServo.setPosition(0.32);
            sleep(750);
            leftServo.setPosition(0.4);
            //drive.followTrajectory(moveStrafeRightCenter3);
            //drive.followTrajectory(moveBackCenter4);

        } else if (location == PropDetectionProcessor.Location.Right) {
            leftClawServo.setPosition(0.32);
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardRight);
            drive.followTrajectory(moveStrafeRight);
            drive.followTrajectory(moveBackRight);
            drive.followTrajectory(moveStrafeLeftRight);
            drive.followTrajectory(moveForwardRight4);
            drive.turn(Math.toRadians(-90));
            leftServo.setPosition(0.7275);
            sleep(9500);
            drive.followTrajectory(moveForwardRight5);
            drive.followTrajectory(moveStrafeRight2);
            drive.followTrajectory(moveBackRight3);
            sleep(75);
            drive.followTrajectory(moveBackRight5);
            leftClawServo.setPosition(0.32);
            sleep(350);
            leftClawServo.setPosition(0.5);
            sleep(750);
            leftServo.setPosition(0.5);
            sleep(100);
            leftClawServo.setPosition(0.32);
            sleep(550);
            drive.followTrajectory(moveForwardRight6);
            leftClawServo.setPosition(0.32);
            sleep(100);
            leftServo.setPosition(0.4);
            //drive.followTrajectory(moveStrafeRight3);
        }

        telemetry.addData("Location: ",location);
        telemetry.update();

        // prepare for movement (set arm to down, lift slide)


    }
}
