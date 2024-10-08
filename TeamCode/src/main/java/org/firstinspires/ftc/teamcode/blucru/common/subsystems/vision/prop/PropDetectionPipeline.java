package org.firstinspires.ftc.teamcode.blucru.common.subsystems.vision.prop;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class PropDetectionPipeline extends OpenCvPipeline {
    private Alliance alliance;


    /*
    0    -----------------------------------------
         |                                       |
         |                                       |
         |                                       |
         |                                       |
 y       |                                       |
         |                                       |
         |                                       |
         |                                       |
     1   -----------------------------------------
         0                                       1
                             x
     rectangle positions */
    public static double rect0x = 0.15;
    public static double rect0y = 0.7;
    public static double rect1x = 0.55;
    public static double rect1y = 0.7;
    public static double rect2x = 0.95;
    public static double rect2y = 0.7;

    // hues for blue
    public static double blueLowH = 80;
    public static double blueHighH = 140;

    // hues for red
    // since red wraps around 0, we need to use 2 ranges
    public static double redLowH1 = 0;
    public static double redHighH1 = 30;
    public static double redLowH2 = 150;
    public static double redHighH2 = 180;

    // make rectangles
    private static Rect rect0 = getRect(rect0x, rect0y, 100, 100, 1280, 720); // rectangle over left prop
    private static Rect rect1 = getRect(rect1x, rect1y, 100, 100, 1280, 720); // rectangle over middle prop
    private static Rect rect2 = getRect(rect2x, rect2y, 100, 100, 1280, 720); // rectangle over right prop

    private Mat subMat0, subMat1, subMat2;

    // average value in each rectangle
    public double average0, average1, average2;
    private double max;
    ArrayList<double[]> frameList;
    public static double strictLowS = 50; // low saturation value for strict HSV filter
    public static double strictHighS = 255; // high saturation value for strict HSV filter
    public int position = 1;

    public PropDetectionPipeline(int camWidth, int camHeight, Alliance alliance) {
        // initialize frameList (not used)
        // frameList = new ArrayList<double[]>();

        // set strict HSV values based on alliance
        this.alliance = alliance;
    }
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if(mat.empty()) {
            return input;
        }

        Mat thresh = new Mat();


        //apply HSV filter
        if(alliance == Alliance.BLUE) {
            Scalar lower = new Scalar(blueLowH,40,20);
            Scalar upper = new Scalar(blueHighH, 255, 255);

            Core.inRange(mat, lower, upper, thresh);
        } else {
            Scalar lower1 = new Scalar(redLowH1, 40, 20);
            Scalar upper1 = new Scalar(redHighH1, 255, 255);

            Scalar lower2 = new Scalar(redLowH2, 40, 20);
            Scalar upper2 = new Scalar(redHighH2, 255, 255);

            Mat thresh1 = new Mat();
            Mat thresh2 = new Mat();

            Core.inRange(mat, lower2, upper2, thresh2);
            Core.inRange(mat, lower1, upper1, thresh1);

            // combine the 2 red threshes
            Core.bitwise_or(thresh1, thresh2, thresh);

            // release thresh1 and thresh2 to save processing power
            thresh1.release();
            thresh2.release();
        }



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
        //apply strict HSV filter onto scaledMask to get rid of any red/blue other than the prop
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        // draw rectangles on scaledThresh
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

        // find the max average value
        max = Math.max(Math.max(average0, average1), average2);
        // set position based on max average value
        if(max == average0) {
            position = 0;
        } else if(max == average1) {
            position = 1;
        } else {
            position = 2;
        }

        // arraylist for mean filter (not used)
//        if (frameList.size() > 5) {
//            frameList.remove(0);
//        }

        Imgproc.cvtColor(masked, masked, Imgproc.COLOR_HSV2RGB);

        // input.release();
        //scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        thresh.release();
        //change the return to whatever mat you want in camera stream
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