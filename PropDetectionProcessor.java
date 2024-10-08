package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

//import com.acmerobotics.dashboard.config.Config;
//@Config
public class PropDetectionProcessor implements VisionProcessor {

    // Enumeration that helps set what color the game team prop is
    public enum Prop {
        BLUE, RED
    }

    // Enumeration that holds the location that the prop is detected at
    public enum Location {
        Left,
        Center,
        Right
    }
    private Location location;
    public Prop propColor = Prop.RED;

    // Scalars that provide the lowHSV and highHSV for image processing
    Scalar lowHSV;
    Scalar highHSV;

    // Rects define the areas that the prop must be on to be considered left, right, or center
    // remove the multipliers when building on robot. x multipliers -3, y multipliers -2.25
    static final Rect LEFT_ROI = new Rect(
            new Point(5, 5),
            new Point(210, 445));
    static final Rect RIGHT_ROI = new Rect(
            new Point(420, 5),
            new Point(635,445));

    static final Rect CENTER_ROI = new Rect(
            new Point(230, 5),
            new Point(400,445));

    // Sets how much percent of the rect an object must take up to be considered to be on the corresponding location
    static double PERCENT_COLOR_THRESHOLD = 0.2;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Not useful
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV); // Converts image to HSV color

        if (propColor == Prop.RED) {
            lowHSV = new Scalar(0, 138.8, 94.9);
            highHSV = new Scalar(18.4, 255, 255);
        } else {
            lowHSV = new Scalar(106.3, 46.8, 87.8);
            highHSV = new Scalar(145.9, 255, 177.1);
        }

        Core.inRange(frame, lowHSV, highHSV, frame);

        Mat left = frame.submat(LEFT_ROI);
        Mat right = frame.submat(RIGHT_ROI);
        Mat center = frame.submat(CENTER_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;

        left.release();
        right.release();
        center.release();

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneCenter = centerValue > PERCENT_COLOR_THRESHOLD;
        /*
        if (stoneCenter) {
            location = Location.Center;
        }
        else if (stoneLeft) {
            location = Location.Left;
        }
        else {
            location = Location.Right;
        }
        */

        double max = Math.max(leftValue, Math.max(rightValue, centerValue));

        if (leftValue == max){
            location = Location.Left;
        } else if (rightValue== max) {
            location = Location.Right;
        } else {
            location = Location.Center;
        }

        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_GRAY2RGB);

        Scalar red = new Scalar(255, 0, 0);
        Scalar green = new Scalar(0, 255, 0);

        Imgproc.rectangle(frame, LEFT_ROI, location == Location.Left? green: red);
        Imgproc.rectangle(frame, RIGHT_ROI, location == Location.Right? green: red);
        Imgproc.rectangle(frame, CENTER_ROI, location == Location.Center? green: red);
        Imgproc.putText(frame, location.name(), new Point(220*3,60*2.25),1,5, red);

        return null; // No context object
    }

    public Location getLocation() {
        return location;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not useful either
    }

}
