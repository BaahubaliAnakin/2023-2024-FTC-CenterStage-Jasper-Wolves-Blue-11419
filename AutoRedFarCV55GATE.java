package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous
public class AutoRedFarCV55GATE extends LinearOpMode {

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
        CRServo rightServo = hardwareMap.crservo.get("rightarmservo");
        Servo clawServo = hardwareMap.servo.get("clawservo");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -62, -199.491);
        drive.setPoseEstimate(startPose);

        //  CENTER //

        Trajectory moveForwardCenter = drive.trajectoryBuilder(startPose)
                .forward(30.5)
                .build();
        Trajectory moveBackCenter = drive.trajectoryBuilder(moveForwardCenter.end())
                .back(10)
                .build();
        Trajectory moveStrafeLeft2 = drive.trajectoryBuilder(moveBackCenter.end())
                .strafeLeft(20)
                .build();
        Trajectory moveForwardCenter2 = drive.trajectoryBuilder(moveStrafeLeft2.end())
                .forward(30)
                .build();
        Trajectory moveStrafeRightCenter = drive.trajectoryBuilder(moveForwardCenter2.end())
                .strafeRight(5)
                .build();
        Trajectory moveForwardCenter3 = drive.trajectoryBuilder(new Pose2d(moveStrafeRightCenter.end().getX(), moveStrafeRightCenter.end().getY(), Math.toRadians(180)))
                .forward(2)
                .build();
        Trajectory moveBackCenter2 = drive.trajectoryBuilder(moveForwardCenter3.end())
                .back(5)
                .build();
        Trajectory moveForwardCenter4 = drive.trajectoryBuilder(new Pose2d(moveBackCenter2.end().getX(), moveBackCenter2.end().getY(), Math.toRadians(-360)))
                .forward(80)
                .build();
        Trajectory moveStrafeRightCenter2 = drive.trajectoryBuilder(moveForwardCenter4.end())
                .strafeRight(13.25)
                .build();
        Trajectory moveBackCenter3 = drive.trajectoryBuilder(new Pose2d(moveStrafeRightCenter2.end().getX(), moveStrafeRightCenter2.end().getY(), Math.toRadians(180)))
                .back(19)
                .build();
        Trajectory moveStrafeLeftCenter4 = drive.trajectoryBuilder(moveBackCenter3.end())
                .strafeLeft(4.5)
                .build();
        Trajectory moveBackCenter5 = drive.trajectoryBuilder(moveStrafeLeftCenter4.end())
                .back(3.5)
                .build();
        Trajectory moveForwardCenter5 = drive.trajectoryBuilder(moveBackCenter3.end())
                .forward(4)
                .build();
        Trajectory moveStrafeRightCenter3 = drive.trajectoryBuilder(moveForwardCenter5.end())
                .strafeRight(13)
                .build();
        Trajectory moveBackCenter4 = drive.trajectoryBuilder(moveStrafeRightCenter3.end())
                .back(10)
                .build();


        //   LEFT   //
        Trajectory moveForwardLeft = drive.trajectoryBuilder(startPose)
                .forward(26.5)
                .build();
        Trajectory moveStrafeLeft = drive.trajectoryBuilder(moveForwardLeft.end())
                .strafeLeft(9)
                .build();
        Trajectory moveBackLeft = drive.trajectoryBuilder(moveStrafeLeft.end())
                .back(7)
                .build();
        Trajectory moveStrafeRightLeft = drive.trajectoryBuilder(moveBackLeft.end())
                .strafeRight(11)
                .build();
        Trajectory moveForwardLeft2 = drive.trajectoryBuilder(moveStrafeRightLeft.end())
                .forward(31)
                .build();
        Trajectory moveStrafeLeftLeft = drive.trajectoryBuilder(moveForwardLeft2.end())
                .strafeLeft(17)
                .build();
        Trajectory moveForwardLeft3 = drive.trajectoryBuilder(new Pose2d(moveStrafeLeftLeft.end().getX(), moveStrafeLeftLeft.end().getY(), Math.toRadians(180)))
                .forward(2)
                .build();
        Trajectory moveBackLeft2 = drive.trajectoryBuilder(moveForwardLeft3.end())
                .back(5)
                .build();
        Trajectory moveForwardLeft4 = drive.trajectoryBuilder(new Pose2d(moveBackLeft2.end().getX(), moveBackLeft2.end().getY(), Math.toRadians(-360)))
                .forward(80)
                .build();
        Trajectory moveStrafeRightLeft2 = drive.trajectoryBuilder(moveForwardLeft4.end())
                .strafeRight(9)
                .build();
        Trajectory moveBackLeft3 = drive.trajectoryBuilder(new Pose2d(moveStrafeRightLeft2.end().getX(), moveStrafeRightLeft2.end().getY(), Math.toRadians(180)))
                .back(19)
                .build();
        Trajectory moveForwardLeft5 = drive.trajectoryBuilder(moveBackLeft3.end())
                .forward(4)
                .build();
        Trajectory moveStrafeRightLeft3 = drive.trajectoryBuilder(moveForwardLeft5.end())
                .strafeRight(10)
                .build();
        Trajectory moveBackLeft4 = drive.trajectoryBuilder(moveStrafeRightLeft3.end())
                .back(10)
                .build();

        //   RIGHT  //
        Trajectory moveForwardRight = drive.trajectoryBuilder(startPose, true)
                .forward(28.5)
                .build();

        Trajectory moveStrafeRight = drive.trajectoryBuilder(moveForwardRight.end())
                .strafeRight(13.5)
                .build();

        Trajectory moveBackRight = drive.trajectoryBuilder(moveStrafeRight.end())
                .back(4.5)
                .build();
        Trajectory moveStrafeLeftRight = drive.trajectoryBuilder(moveBackRight.end())
                .strafeLeft(20)
                .build();
        Trajectory moveForwardRight2 = drive.trajectoryBuilder(moveStrafeLeftRight.end())
                .forward(2.5)
                .build();
        Trajectory moveForwardRight3 = drive.trajectoryBuilder(new Pose2d(moveForwardRight2.end().getX(), moveForwardRight2.end().getY(), Math.toRadians(180)))
                .forward(12)
                .build();
        Trajectory moveBackRight2 = drive.trajectoryBuilder(moveForwardRight3.end())
                .back(12)
                .build();
        Trajectory moveForwardRight4 = drive.trajectoryBuilder(new Pose2d(moveBackRight2.end().getX(), moveBackRight2.end().getY(), Math.toRadians(-180)))
                .forward(30)
                .build();
        Trajectory moveForwardRight5 = drive.trajectoryBuilder(new Pose2d(moveForwardRight4.end().getX(), moveForwardRight4.end().getY(), Math.toRadians(-180)))
                .forward(75)
                .build();
        Trajectory moveStrafeRight2 = drive.trajectoryBuilder(moveForwardRight5.end())
                .strafeRight(27)
                .build();
        Trajectory moveBackRight3 = drive.trajectoryBuilder(new Pose2d(moveStrafeRight2.end().getX(), moveStrafeRight2.end().getY(), Math.toRadians(180)))
                .back(19)
                .build();
        Trajectory moveForwardRight6 = drive.trajectoryBuilder(moveBackRight3.end())
                .forward(4)
                .build();
        Trajectory moveStrafeRight3 = drive.trajectoryBuilder(moveForwardRight6.end())
                .strafeRight(26.5)
                .build();
        Trajectory moveBackRight4 = drive.trajectoryBuilder(moveStrafeRight3.end())
                .back(10)
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
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardLeft);
            drive.followTrajectory(moveStrafeLeft);
            drive.followTrajectory(moveBackLeft);
            drive.followTrajectory(moveStrafeRightLeft);
            rightServo.setPower(0.55);
            drive.followTrajectory(moveForwardLeft2);
            drive.followTrajectory(moveStrafeLeftLeft);
            clawServo.setPosition(1.0);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(moveForwardLeft3);
            sleep(1000);
            clawServo.setPosition(0.4);
            drive.followTrajectory(moveBackLeft2);
            drive.turn(Math.toRadians(-180));
            drive.followTrajectory(moveForwardLeft4);
            drive.followTrajectory(moveStrafeRightLeft2);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(moveBackLeft3);
            sleep(750);
            as1.setPosition(0.5);
            sleep(500);
            drive.followTrajectory(moveForwardLeft5);
            drive.followTrajectory(moveStrafeRightLeft3);
            drive.followTrajectory(moveBackLeft4);

        } else if (location == PropDetectionProcessor.Location.Center) {
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardCenter);
            drive.followTrajectory(moveBackCenter);
            drive.followTrajectory(moveStrafeLeft2);
            rightServo.setPower(0.55);
            drive.followTrajectory(moveForwardCenter2);
            drive.followTrajectory(moveStrafeRightCenter);
            clawServo.setPosition(1.0);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(moveForwardCenter3);
            sleep(1000);
            clawServo.setPosition(0.4);
            drive.followTrajectory(moveBackCenter2);
            drive.turn(Math.toRadians(-180));
            drive.followTrajectory(moveForwardCenter4);
            drive.followTrajectory(moveStrafeRightCenter2);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(moveBackCenter3);
            drive.followTrajectory(moveStrafeLeftCenter4);
            drive.followTrajectory(moveBackCenter5);
            sleep(750);
            as1.setPosition(0.5);
            sleep(500);
            drive.followTrajectory(moveForwardCenter5);
            drive.followTrajectory(moveStrafeRightCenter3);
            drive.followTrajectory(moveBackCenter4);

        } else if (location == PropDetectionProcessor.Location.Right) {
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardRight);
            drive.followTrajectory(moveStrafeRight);
            drive.followTrajectory(moveBackRight);
            drive.followTrajectory(moveStrafeLeftRight);
            rightServo.setPower(0.55);
            drive.followTrajectory(moveForwardRight2);
            clawServo.setPosition(1.0);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(moveForwardRight3);
            sleep(1000);
            clawServo.setPosition(0.4);
            drive.followTrajectory(moveBackRight2);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(moveForwardRight4);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(moveForwardRight5);
            drive.followTrajectory(moveStrafeRight2);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(moveBackRight3);
            sleep(750);
            as1.setPosition(0.5);
            sleep(500);
            drive.followTrajectory(moveForwardRight6);
            drive.followTrajectory(moveStrafeRight3);
            drive.followTrajectory(moveBackRight4);
        }

        telemetry.addData("Location: ",location);
        telemetry.update();

        // prepare for movement (set arm to down, lift slide)


    }
}
