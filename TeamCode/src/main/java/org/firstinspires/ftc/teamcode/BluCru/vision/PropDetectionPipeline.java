package org.firstinspires.ftc.teamcode.BluCru.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.BluCru.Alliance;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class PropDetectionPipeline extends OpenCvPipeline {
    private static double blueStrictLowS = 130;
    private static double blueStrictHighS = 255;
    private static double redStrictLowS = 130;
    private static double redStrictHighS = 255;
    private static Rect rect0 = getRect(0.3, 0.5, 100, 100, 1280, 720);
    private static Rect rect1 = getRect(0.5, 0.5, 100, 100, 1280, 720);
    private static Rect rect2 = getRect(0.7, 0.5, 100, 100, 1280, 720);
    public static Mat subMat0, subMat1, subMat2;
    public static double average0, average1, average2;
    ArrayList<double[]> frameList;
    public static double strictLowS;
    public static double strictHighS;
    public static int position = 1;

    public PropDetectionPipeline(int camWidth, int camHeight, Alliance alliance) {
        // initialize frameList
        frameList = new ArrayList<double[]>();

        // set strict HSV values based on alliance
        if (alliance == Alliance.BLUE) {
            strictLowS = blueStrictLowS;
            strictHighS = blueStrictHighS;
        } else {
            strictLowS = redStrictLowS;
            strictHighS = redStrictHighS;
        }
    }
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if(mat.empty()) {
            return input;
        }

        //apply HSV filter
        Scalar lower = new Scalar(80,40,20);
        Scalar upper = new Scalar(150, 255, 255);

        Mat thresh = new Mat();
        Core.inRange(mat, lower, upper, thresh);

        Mat masked = new Mat();
        Core.bitwise_and(mat, mat, masked, thresh);


        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);
        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);



        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for blue
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for blue
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Imgproc.rectangle(scaledThresh, rect0, new Scalar(255, 255, 255), 2);
        Imgproc.rectangle(scaledThresh, rect1, new Scalar(255, 255, 255), 2);
        Imgproc.rectangle(scaledThresh, rect2, new Scalar(255, 255, 255), 2);

        // create submats for 3 detection areas
        subMat0 = scaledThresh.submat(rect0);
        subMat1 = scaledThresh.submat(rect1);
        subMat2 = scaledThresh.submat(rect2);

        // get average values of each submat
        average0 = Core.mean(subMat0).val[0];
        average1 = Core.mean(subMat1).val[0];
        average2 = Core.mean(subMat2).val[0];

        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        Imgproc.cvtColor(masked, masked, Imgproc.COLOR_HSV2RGB);

        // input.release();
        //scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        thresh.release();
        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
        return scaledThresh;
    }

    public static Rect getRect(double centerx, double centery, int width, int height, int camWidth, int camHeight) {
        int subMatRectX = (int)(camWidth * centerx) - (width / 2);
        int subMatRectY = (int)(camHeight * centery) - (height / 2);
        int subMatRectWidth = width;
        int subMatRectHeight = height;

        Rect subMatRect = new Rect(subMatRectX, subMatRectY, subMatRectWidth, subMatRectHeight);
        return subMatRect;
    }
}