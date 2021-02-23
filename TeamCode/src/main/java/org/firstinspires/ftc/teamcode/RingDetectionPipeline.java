package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Constants.*;

class RingDetectionPipeline extends OpenCvPipeline
{
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
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCb(firstFrame);

        region1_Cb = Cb.submat(new Rect(REGION1_POINTA, REGION1_POINTB));
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
                REGION1_POINTA, // First point which defines the rectangle
                REGION1_POINTB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        return input;
    }

    public RingNumber getRingNumber()
    {
        return ringNumber;
    }
}