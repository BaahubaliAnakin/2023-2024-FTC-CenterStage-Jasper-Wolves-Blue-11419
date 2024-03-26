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
public class AutoBlueFarCV50GATE extends LinearOpMode {

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
        Servo leftServo = hardwareMap.servo.get("leftarmservo");
        Servo clawServo = hardwareMap.servo.get("clawservo");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, 62, 199.491);
        drive.setPoseEstimate(startPose);

        //  CENTER //

        Trajectory moveForwardCenter = drive.trajectoryBuilder(startPose)
                .forward(31)
                .build();
        Trajectory moveBackCenter = drive.trajectoryBuilder(moveForwardCenter.end())
                .back(10)
                .build();
        Trajectory moveStrafeCenterRight2 = drive.trajectoryBuilder(moveBackCenter.end())
                .strafeRight(20)
                .build();
        Trajectory moveForwardCenter2 = drive.trajectoryBuilder(moveStrafeCenterRight2.end())
                .forward(30)
                .build();
        Trajectory moveForwardCenter4 = drive.trajectoryBuilder(new Pose2d(moveForwardCenter2.end().getX(), moveForwardCenter2.end().getY(), Math.toRadians(0)))
                .forward(90)
                .build();
        Trajectory moveStrafeLeftCenter2 = drive.trajectoryBuilder(moveForwardCenter4.end())
                .strafeLeft(13.25)
                .build();
        Trajectory moveBackCenter3 = drive.trajectoryBuilder(new Pose2d(moveStrafeLeftCenter2.end().getX(), moveStrafeLeftCenter2.end().getY(), Math.toRadians(180)))
                .back(16)
                .build();
        Trajectory moveStrafeRightCenter4 = drive.trajectoryBuilder(moveBackCenter3.end())
                .strafeRight(18.95)
                .build();
        Trajectory moveBackCenter5 = drive.trajectoryBuilder(moveStrafeRightCenter4.end())
                .back(3.2)
                .build();
        Trajectory moveForwardCenter5 = drive.trajectoryBuilder(moveBackCenter5.end())
                .forward(8)
                .build();
        Trajectory moveStrafeLeftCenter3 = drive.trajectoryBuilder(moveForwardCenter5.end())
                .strafeLeft(30)
                .build();
        Trajectory moveBackCenter4 = drive.trajectoryBuilder(moveStrafeLeftCenter3.end())
                .back(10)
                .build();

        //   LEFT   //
        Trajectory moveForwardLeft = drive.trajectoryBuilder(startPose, true)
                .forward(28.5)
                .build();
        Trajectory moveStrafeLeft = drive.trajectoryBuilder(moveForwardLeft.end())
                .strafeLeft(9)
                .build();
        Trajectory moveBackLeft = drive.trajectoryBuilder(moveStrafeLeft.end())
                .back(4.5)
                .build();
        Trajectory moveStrafeRightLeft = drive.trajectoryBuilder(moveBackLeft.end())
                .strafeRight(20)
                .build();
        Trajectory moveForwardLeft4 = drive.trajectoryBuilder(moveStrafeRightLeft.end())
                .forward(26)
                .build();
        Trajectory moveForwardLeft5 = drive.trajectoryBuilder(new Pose2d(moveForwardLeft4.end().getX(), moveForwardLeft4.end().getY(), Math.toRadians(0)))
                .forward(80)
                .build();
        Trajectory moveStrafeLeft2 = drive.trajectoryBuilder(moveForwardLeft5.end())
                .strafeLeft(39)
                .build();
        Trajectory moveBackLeft3 = drive.trajectoryBuilder(new Pose2d(moveStrafeLeft2.end().getX(), moveStrafeLeft2.end().getY(), Math.toRadians(180)))
                .back(18)
                .build();
        Trajectory moveBackLeft5 = drive.trajectoryBuilder(moveBackLeft3.end())
                .back(2)
                .build();
        Trajectory moveForwardLeft6 = drive.trajectoryBuilder(moveBackLeft3.end())
                .forward(4)
                .build();
        Trajectory moveStrafeLeft3 = drive.trajectoryBuilder(moveForwardLeft6.end())
                .strafeLeft(24.5)
                .build();
        Trajectory moveBackLeft4 = drive.trajectoryBuilder(moveStrafeLeft3.end())
                .back(10)
                .build();

        //   RIGHT  //
        Trajectory moveForwardRight = drive.trajectoryBuilder(startPose)
                .forward(26.5)
                .build();
        Trajectory moveStrafeRight = drive.trajectoryBuilder(moveForwardRight.end())
                .strafeRight(13.5)
                .build();
        Trajectory moveBackRight = drive.trajectoryBuilder(moveStrafeRight.end())
                .back(7)
                .build();
        Trajectory moveStrafeLeftRight = drive.trajectoryBuilder(moveBackRight.end())
                .strafeLeft(14.5)
                .build();
        Trajectory moveForwardRight2 = drive.trajectoryBuilder(moveStrafeLeftRight.end())
                .forward(31)
                .build();
        Trajectory moveForwardRight4 = drive.trajectoryBuilder(new Pose2d(moveForwardRight2.end().getX(), moveForwardRight2.end().getY(), Math.toRadians(0)))
                .forward(80)
                .build();
        Trajectory moveStrafeLeftRight2 = drive.trajectoryBuilder(moveForwardRight4.end())
                .strafeLeft(25)
                .build();
        Trajectory moveBackRight3 = drive.trajectoryBuilder(new Pose2d(moveStrafeLeftRight2.end().getX(), moveStrafeLeftRight2.end().getY(), Math.toRadians(180)))
                .back(8)
                .build();
        Trajectory moveBackRight5 = drive.trajectoryBuilder(moveBackRight3.end())
                .back(1)
                .build();
        Trajectory moveForwardRight5 = drive.trajectoryBuilder(moveBackRight3.end())
                .forward(4)
                .build();
        Trajectory moveStrafeLeftRight3 = drive.trajectoryBuilder(moveForwardRight5.end())
                .strafeLeft(10)
                .build();
        Trajectory moveBackRight4 = drive.trajectoryBuilder(moveStrafeLeftRight3.end())
                .back(10)
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
            drive.followTrajectory(moveStrafeRightLeft);
            //leftServo.setPosition(0.33);
           // drive.followTrajectory(moveForwardLeft2);
           // clawServo.setPosition(0.4);
           // drive.turn(Math.toRadians(-90));
           // drive.followTrajectory(moveForwardLeft3);
           // sleep(1000);
            //clawServo.setPosition(1.0);
           // drive.followTrajectory(moveBackLeft2);
            //drive.turn(Math.toRadians(90));
            drive.followTrajectory(moveForwardLeft4);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(moveForwardLeft5);
            drive.followTrajectory(moveStrafeLeft2);
            drive.turn(Math.toRadians(-180));
            drive.followTrajectory(moveBackLeft3);
            sleep(75);
            drive.followTrajectory(moveBackLeft5);
            sleep(750);
            as1.setPosition(0.5);
            sleep(750);
            drive.followTrajectory(moveForwardLeft6);
            as1.setPosition(0);
            sleep(950);
            //drive.followTrajectory(moveStrafeLeft3);

        } else if (location == PropDetectionProcessor.Location.Center) {
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardCenter);
            drive.followTrajectory(moveBackCenter);
            drive.followTrajectory(moveStrafeCenterRight2);
            //leftServo.setPosition(0.33);
            drive.followTrajectory(moveForwardCenter2);
            //clawServo.setPosition(0.4);
            drive.turn(Math.toRadians(90));
           // sleep(1000);
           // clawServo.setPosition(1.0);
            drive.followTrajectory(moveForwardCenter4);
            drive.followTrajectory(moveStrafeLeftCenter2);
            drive.turn(Math.toRadians(-180));
            drive.followTrajectory(moveBackCenter3);
            drive.followTrajectory(moveStrafeRightCenter4);
            drive.followTrajectory(moveBackCenter5);
            sleep(750);
            as1.setPosition(0.5);
            sleep(850);
            drive.followTrajectory(moveForwardCenter5);
            as1.setPosition(0);
            sleep(950);
            //drive.followTrajectory(moveStrafeLeftCenter3);
            //drive.followTrajectory(moveBackCenter4);

        } else if (location == PropDetectionProcessor.Location.Right) {
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardRight);
            drive.followTrajectory(moveStrafeRight);
            drive.followTrajectory(moveBackRight);
            drive.followTrajectory(moveStrafeLeftRight);
            //leftServo.setPosition(0.33);
            drive.followTrajectory(moveForwardRight2);
            //clawServo.setPosition(0.4);
            drive.turn(Math.toRadians(90));
            //sleep(1000);
            //clawServo.setPosition(1.0);
            drive.followTrajectory(moveForwardRight4);
            drive.followTrajectory(moveStrafeLeftRight2);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(moveBackRight3);
            sleep(75);
            drive.followTrajectory(moveBackRight5);
            sleep(750);
            as1.setPosition(0.5);
            sleep(750);
            drive.followTrajectory(moveForwardRight5);
            as1.setPosition(0);
            sleep(950);
            //drive.followTrajectory(moveStrafeLeftRight3);

        }



    }
}