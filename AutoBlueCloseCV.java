package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class AutoBlueCloseCV extends LinearOpMode {

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


        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(11.6, 62, 199.491);
        drive.setPoseEstimate(startPose);

        //  CENTER //

        Trajectory moveForwardCenter = drive.trajectoryBuilder(startPose)
                .forward(30.5)
                .build();

        Trajectory moveBackCenter = drive.trajectoryBuilder(moveForwardCenter.end())
                .back(14.5)
                .build();

        Trajectory moveStrafeCenter = drive.trajectoryBuilder(moveBackCenter.end())
                .strafeLeft(10)
                .build();

        Trajectory moveBackCenter2 = drive.trajectoryBuilder(new Pose2d(moveStrafeCenter.end().getX(), moveStrafeCenter.end().getY(), Math.toRadians(180)))
                .back(28)
                .build();

        Trajectory moveStrafeCenter2 = drive.trajectoryBuilder(moveBackCenter2.end())
                .strafeLeft(4.5)
                .build();

        Trajectory moveBackCenter3 = drive.trajectoryBuilder(moveStrafeCenter2.end())
                .back(2.5)
                .build();

        Trajectory moveForwardCenter2 = drive.trajectoryBuilder(moveBackCenter3.end())
                .forward(4)
                .build();

        Trajectory moveStrafeCenter3 = drive.trajectoryBuilder(moveForwardCenter2.end())
                .strafeRight(20)
                .build();

        Trajectory moveBackCenter4 = drive.trajectoryBuilder(moveStrafeCenter3.end())
                .back(9)
                .build();



        //   LEFT   //
        Trajectory moveForwardLeft = drive.trajectoryBuilder(startPose)
                .forward(25)
                .build();

        Trajectory moveStrafeLeft = drive.trajectoryBuilder(moveForwardLeft.end())
                .strafeLeft(10)
                .build();

        Trajectory moveBackLeft = drive.trajectoryBuilder(moveStrafeLeft.end())
                .back(13)
                .build();

        Trajectory moveBackLeft2 = drive.trajectoryBuilder(new Pose2d(moveBackLeft.end().getX(), moveBackLeft.end().getY(), Math.toRadians(180)))
                .back(28)
                .build();

        Trajectory moveStrafeLeft2 = drive.trajectoryBuilder(moveBackLeft2.end())
                .strafeLeft(3)
                .build();

        Trajectory moveBackLeft3 = drive.trajectoryBuilder(moveStrafeLeft2.end())
                .back(3.75)
                .build();

        Trajectory moveForwardLeft2 = drive.trajectoryBuilder(moveBackLeft2.end())
                .forward(4)
                .build();

        Trajectory moveStrafeLeft3 = drive.trajectoryBuilder(moveForwardLeft2.end())
                .strafeRight(11.25)
                .build();

        Trajectory moveBackLeft4 = drive.trajectoryBuilder(moveStrafeLeft3.end())
                .back(9)
                .build();


        //   RIGHT  //
        Trajectory moveForwardRight = drive.trajectoryBuilder(startPose)
                .forward(28.5)
                .build();

        Trajectory moveStrafeRight = drive.trajectoryBuilder(moveForwardRight.end())
                .strafeRight(13.5)
                .build();

        Trajectory moveBackRight = drive.trajectoryBuilder(moveStrafeRight.end())
                .back(3.95)
                .build();

        Trajectory moveStrafeLeftRight = drive.trajectoryBuilder(moveBackRight.end())
                .strafeLeft(18)
                .build();

        Trajectory moveForwardRight2 = drive.trajectoryBuilder(moveStrafeLeftRight.end())
                .forward(1.5)
                .build();

        Trajectory moveBackRight2 = drive.trajectoryBuilder(new Pose2d(moveForwardRight2.end().getX(), moveForwardRight2.end().getY(), Math.toRadians(180)))
                .back(35)
                .build();

        Trajectory moveStrafeRight2 = drive.trajectoryBuilder(moveBackRight2.end())
                .strafeLeft(1.25)
                .build();

        Trajectory moveBackRight3 = drive.trajectoryBuilder(moveStrafeRight2.end())
                .back(2)
                .build();

        Trajectory moveForwardRight3 = drive.trajectoryBuilder(moveBackRight2.end())
                .forward(4)
                .build();

        Trajectory moveStrafeRight3 = drive.trajectoryBuilder(moveForwardRight3.end())
                .strafeRight(25)
                .build();

        Trajectory moveBackRight4 = drive.trajectoryBuilder(moveStrafeRight3.end())
                .back(9)
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
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardLeft);
            drive.followTrajectory(moveStrafeLeft);
            drive.followTrajectory(moveBackLeft);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(moveBackLeft2);
            drive.followTrajectory(moveStrafeLeft2);
            drive.followTrajectory(moveBackLeft3);
            sleep(500);
            as1.setPosition(0.5);
            sleep(500);
            drive.followTrajectory(moveForwardLeft2);
            as1.setPosition(0);
            drive.followTrajectory(moveStrafeLeft3);
            drive.followTrajectory(moveBackLeft4);


        } else if (location == PropDetectionProcessor.Location.Center) {
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardCenter);
            drive.followTrajectory(moveBackCenter);
            drive.followTrajectory(moveStrafeCenter);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(moveBackCenter2);
            drive.followTrajectory(moveStrafeCenter2);
            drive.followTrajectory(moveBackCenter3);
            sleep(500);
            as1.setPosition(0.5);
            sleep(500);
            drive.followTrajectory(moveForwardCenter2);
            as1.setPosition(0);
            drive.followTrajectory(moveStrafeCenter3);
            drive.followTrajectory(moveBackCenter4);

        } else if (location == PropDetectionProcessor.Location.Right) {
            drive.setPoseEstimate(startPose);
            drive.followTrajectory(moveForwardRight);
            drive.followTrajectory(moveStrafeRight);
            drive.followTrajectory(moveBackRight);
            drive.followTrajectory(moveStrafeLeftRight);
            drive.followTrajectory(moveForwardRight2);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(moveBackRight2);
            drive.followTrajectory(moveStrafeRight2);
            drive.followTrajectory(moveBackRight3);
            sleep(500);
            as1.setPosition(0.5);
            sleep(500);
            drive.followTrajectory(moveForwardRight3);
            as1.setPosition(0);
            drive.followTrajectory(moveStrafeRight3);
            drive.followTrajectory(moveBackRight4);
        }
        telemetry.addData("Location: ",location);
        telemetry.update();

        // prepare for movement (set arm to down, lift slide)
        visionPortal.close();


    }
}
