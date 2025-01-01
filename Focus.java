package Guidance;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.util.CircularBuffer;

public class Focus {
     /**
     * Image Sharpness detemination for focus adjusment.
     * Larger values are sharper.
     */
    static int bufferSize = 20;
    static CircularBuffer<Double> sharpnessBuffer = new CircularBuffer<>(bufferSize); // image frame history

    /**
     * Compute an image sharpness metric
     * 
     * Tuned for its special version of the Siemens Star
     * 
     * @param image on which to focus
     * @return sharpness metric
     */
    public static int sharpnessMetric(Mat img) {
        Mat gray = new Mat();
        Mat lap = new Mat();
        MatOfDouble mean = new MatOfDouble();
        MatOfDouble standardDeviation = new MatOfDouble();
        Mat var = new Mat();
        double[] variance = new double[1];

        Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);

        boolean removeStarBackground = false; // optional remove background around a Siemens Star to reduce sharpness detection jitter
        if (removeStarBackground) {
        // find a circle mask to get rid of the background around the star
        Mat circles = new Mat();
        Mat circleMask = Mat.zeros(gray.rows(), gray.cols(), gray.type());
        Mat grayBlurry = new Mat();
        Imgproc.blur(gray, grayBlurry, new Size(3, 3), new Point(-1, -1)); // trashing the input is okay now
        Core.inRange(grayBlurry, new Scalar(100), new Scalar(255), grayBlurry);
        Imgproc.HoughCircles(
			grayBlurry, //Input image (gray-scale).
			circles, //A vector that stores sets of 3 values: xc,yc,r for each detected circle.
			Imgproc.HOUGH_GRADIENT_ALT, //Define the detection method.
			1.5, //The inverse ratio of resolution.
			(double)img.rows()/8., //Minimum distance between detected centers.
			100., //param_1: Upper threshold for the internal Canny edge detector.
				// param1: sensitivity of strength of edge
					//	too high - no edges detected
					//	too low - too much clutter
			0.7, //param_2: Threshold for center detection.
					// param2: how many edge points needed to find a circle. It's related to circumference. Accumulator Threshold
					// param2 for not _ALT too low and everything is a circle.
                    // param2 for _ALT 1 is perfect circle and 0 is not a circle

			80, //Minimum radius to be detected. If unknown, put zero as default.
			350 //Maximum radius to be detected. If unknown, put zero as default.
			);

        if(!circles.empty()) {
            var extractBoundry = new float[3];
            circles.get(0, 0, extractBoundry);
            Imgproc.circle(circleMask, new Point(extractBoundry[0], extractBoundry[1]), (int)extractBoundry[2], new Scalar(255), Imgproc.FILLED);
            Core.bitwise_and(gray, circleMask, gray); 
            // HighGui.imshow("gray", gray);    
            // HighGui.waitKey(1);
        }
        }

        // The use of second derivatives is one technique for passing the high spatial frequencies,
        // which are associated with sharp edges. As a second derivative operator we use the Laplacian
        // operator.
        Imgproc.Laplacian(gray, lap, 3); // highlighting distinct edges
        Core.meanStdDev(lap, mean, standardDeviation); // pooling all the individual points into one value
        Core.multiply(standardDeviation, standardDeviation, var); // square the standard deviation to get variance
        var.get(0, 0, variance); // Mat extraction to double
        sharpnessBuffer.addFirst(variance[0]);
        var maxSharpness = 0.;
        for (int i = 0; i < sharpnessBuffer.size(); i++) {
            maxSharpness = Math.max( sharpnessBuffer.get(i), maxSharpness);
        }
        return (int)(maxSharpness + 0.51/*rounding*/); // max value in the circular buffer            
    }

/**
 * Siemens Star focusing target.
 * 
 * Distinct circle drawn around star so Hough circle detection can be used
 * to create a mask to remove the background noise.
 * 
 * @param img
 * @return focusing target image
 */
    public static Mat drawSiemensStar() {
        int radius = 350;
        int margin = 10;
        Mat star = Mat.zeros(2*radius+margin, 2*radius+margin, CvType.CV_8UC1);
        int xCenter = star.cols()/2;
        int yCenter = star.rows()/2;
        for (double angle = 0.; angle < 2.*Math.PI; angle += 2.*Math.PI/360.*10.) {
            int x1Circumference = (int)(radius*Math.sin(angle)) + xCenter;
            int y1Circumference = (int)(radius*Math.cos(angle)) + yCenter;
            int x2Circumference = (int)(radius*Math.sin(angle+2.*Math.PI/360.*5.)) + xCenter;
            int y2Circumference = (int)(radius*Math.cos(angle+2.*Math.PI/360.*5.)) + yCenter;
            MatOfPoint triangle = new MatOfPoint(new Point(xCenter, yCenter),
                    new Point(x1Circumference, y1Circumference), new Point(x2Circumference, y2Circumference));
            List<MatOfPoint> triangles = new ArrayList<>();
            triangles.add(triangle);
            Imgproc.polylines(star, triangles, true, new Scalar(210),1,Imgproc.LINE_AA);
            Imgproc.fillPoly(star, triangles, new Scalar(210), Imgproc.LINE_AA);
            Imgproc.circle(star, new Point(xCenter, yCenter), radius, new Scalar(255), 4, Imgproc.LINE_AA);
            Imgproc.circle(star, new Point(xCenter, yCenter), radius-4, new Scalar(0), 4, Imgproc.LINE_AA);
        }
        return star;
    }
}
