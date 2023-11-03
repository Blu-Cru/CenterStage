package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BluePropDetectionPipeline extends OpenCvPipeline {
    ArrayList<double[]> frameList;
    public static double strictLowS;
    public static double strictHighS;

    public BluePropDetectionPipeline() {
        frameList = new ArrayList<double[]>();
        strictLowS = 0;
        strictHighS = 255;
    }
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if(mat.empty()) {
            return input;
        }

        Scalar lower = new Scalar(80,60,40);
        Scalar upper = new Scalar(255, 255, 255);

        Mat thresh = new Mat();
        Core.inRange(mat, lower, upper, thresh);

        Mat masked = new Mat();
        Core.bitwise_and(mat, mat, masked, thresh);

        /*
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);
        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);

        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        Mat edges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (frameList.size() > 5) {
            frameList.remove(0);
        }
*/

        // input.release();
        //scaledThresh.release();
        //scaledMask.release();
        mat.release();
        Imgproc.cvtColor(masked, masked, Imgproc.COLOR_HSV2RGB);
        // masked.release();
        //edges.release();
        thresh.release();
        //finalMask.release();
        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
        return masked;
    }
}
