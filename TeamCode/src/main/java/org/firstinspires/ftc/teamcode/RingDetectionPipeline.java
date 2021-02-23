package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.AutoBase.allianceColor;

class RingDetectionPipeline extends OpenCvPipeline
{

    private Point Region1CentralAnchorPoint;
    private static Point Region1PointA;
    private static Point Region1PointB;

    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    protected int avg1;

    private volatile RingNumber ringNumber = null;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame)
    {
        if(allianceColor == AllianceColor.RED) {
            Region1CentralAnchorPoint = new Point(10,245);
        } else {
            Region1CentralAnchorPoint = new Point(RESOLUTION_WIDTH - 10,245);
        }

        Region1PointA = new Point(
                Region1CentralAnchorPoint.x - REGION_WIDTH/2,
                Region1CentralAnchorPoint.y - REGION_WIDTH/2);
        Region1PointB = new Point(
                Region1CentralAnchorPoint.x + REGION_WIDTH/2,
                Region1CentralAnchorPoint.y + REGION_HEIGHT/2);

        inputToCb(firstFrame);

        region1_Cb = Cb.submat(new Rect(Region1PointA, Region1PointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];

        if(avg1 > 120) {
            ringNumber = RingNumber.NONE;
        } else if (avg1 > 110) {
            ringNumber = RingNumber.ONE;
        } else {
            ringNumber = RingNumber.FOUR;
        }

        Imgproc.rectangle(
                input, // Buffer to draw on
                Region1PointA, // First point which defines the rectangle
                Region1PointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        return input;
    }

    public RingNumber getRingNumber()
    {
        return ringNumber;
    }
}