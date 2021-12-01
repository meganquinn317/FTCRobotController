package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    boolean viewportPaused;
    Mat mat = new Mat();
    public int lowX = 110;
    public int lowY = 20;
    public int lowZ = 250;
    public int highX = 140;
    public int highY = 240;
    public int highZ = 300;
    public int elementPos = 0;
    Rect leftROI = new Rect(new Point(0, 140), new Point(100, 100));
    Rect middleROI = new Rect(new Point(140, 140), new Point(180, 100));
    Rect rightROI = new Rect(new Point(220, 140), new Point(320, 100));
    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HLS);
        Mat leftMat = mat.submat(leftROI);
        Mat middleMat = mat.submat(middleROI);
        Mat rightMat = mat.submat(rightROI);

        Scalar lowColorvalue = new Scalar(lowX, lowY, lowZ);
        Scalar highColorValue = new Scalar(highX, highY, highZ);
        Core.inRange(mat, lowColorvalue, highColorValue, mat);

        return mat;
    }
}
