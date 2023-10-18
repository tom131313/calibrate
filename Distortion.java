package calibrator;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import static calibrator.ArrayUtils.brief;
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     Distortion class                                                       */
/*                                     Distortion class                                                       */
/*                                     Distortion class                                                       */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
public class Distortion
{
    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     Distortion constructor                                                 */
/*                                     Distortion constructor                                                 */
/*                                     Distortion constructor                                                 */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    private Distortion(){}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     get_bounds                                                             */
/*                                     get_bounds                                                             */
/*                                     get_bounds                                                             */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public static Rect get_bounds(Mat thresh, Mat mask)
    {
        // seems like a better strategy would be to see what contour actually contributes the most and not just check the largest ones
        // and use true area of contour and not just the number of points in the contour
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        Main.LOGGER.log(Level.WARNING, "thresh " + thresh);        
        Main.LOGGER.log(Level.WARNING, "mask " + mask);

        List<MatOfPoint> contours = new ArrayList<>(20); // arbitrary initial size - what is a better guess?
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
       
        Main.LOGGER.log(Level.WARNING, contours.size() + " contours");
        // look for the largest object that is not masked
        // This is essentially a Sort and Filter. It's not very efficient but that makes it easier
        // by not having to reorder the contours. The list is expected to be very short so it's not
        // a problem.

        // outer loop to allow the contours list to be trimmed by the filter
        // since Java disallows changing a list within an iterator
        while(contours.size() > 0)
        {
            // look for contour with largest area
            double areaContourMax = -1.; // indicate no max at the start
            int mx = -1; // indicate no max at the start
            for(int i = 0; i < contours.size(); i++)
            {
                long areaContour = contours.get(i).total();

                if( areaContour >= areaContourMax)
                {
                    // new max and its location
                    areaContourMax = areaContour;
                    mx = i;
                }
                Main.LOGGER.log(Level.WARNING, "Contour " + (mx+1) + " of " + contours.size() + ", area max so far " + areaContourMax
                    + ", contour size " + contours.get(mx).size(mx) + "\n" + contours.get(mx).dump());
            }
            // Now have contour with largest area so check that area not already covered,
            // that is, it's not masked.
            // If already mostly masked then this contour doesn't contribute
            // enough so delete it. Else it's good and return it.
            Rect aabb = Imgproc.boundingRect(contours.get(mx));
            int x = aabb.x;
            int y = aabb.y;
            int w = aabb.width;
            int h = aabb.height;
            Main.LOGGER.log(Level.WARNING, "processing Rect aabb " + aabb);

            if(!mask.empty() // amount of mask already filled where this contour would fill
                && (double)Core.countNonZero(mask.submat(y, y+h, x, x+w)) / (double)(w*h) > Cfg.MAX_OVERLAP)
            {
                contours.remove(mx); // largest contour wouldn't contribute enough in the right places so skip it
                continue;
            }
            Main.LOGGER.log(Level.WARNING, "returning aabb " + aabb); // best contributing contour for the pose
            return aabb; // best contour in list so return it
        }

        Main.LOGGER.log(Level.WARNING, "returning null aabb"); // pose doesn't contribute enough

        return null; // no contours met the criteria
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     make_distort_map                                                       */
/*                                     make_distort_map                                                       */
/*                                     make_distort_map                                                       */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    /**
     * creates a map for distorting an image as a opposed to the default behaviour of undistorting
     * 
     * @param K
     * @param sz width, height
     * @param dist
     * @param Knew
     * @return
     */
    public static Mat make_distort_map(Mat K, Size sz, Mat dist, Mat Knew)
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        Main.LOGGER.log(Level.WARNING, "camera matrix K " + K + "\n" + K.dump());
        Main.LOGGER.log(Level.WARNING, "sz " + sz);
        Main.LOGGER.log(Level.WARNING, "distortion coefficients dist " + dist.dump() + dist);
        Main.LOGGER.log(Level.WARNING, "Knew " + Knew.dump()); // null pointer (or empty?) Knew

        //     pts = np.array(np.meshgrid(range(sz[0]), range(sz[1]))).T.reshape(-1, 1, 2)
        // inclusive 0, to not included final, step; fils one column down the rows then the next column and down the rows
        int w = (int)sz.width; // columns
        int h = (int)sz.height; // rows
        int c = 2; // channels, x and y axes
        // make smaller 2-D Mat of x,y points from full size image Mat
        // the values in the smaller Mat are the original x, y coordinates from the larger Mat

        MatOfPoint2f pts = new MatOfPoint2f();
        pts.alloc(h*w);
        // make 2d meshgrid but flattened to 1 dimension
        // created mesh grid with flipped rows/cols so the transpose isn't needed like the np.meshgrid
        float[] ptsMeshGrid = new float[w*c]; // intermediate 1d version of 2d points for a column; X, Y pair
        int indexRowChannel;
        int indexRow = 0;
        for(int y = 0; y < h; y++) // traverse all rows
        {
            indexRowChannel = 0;
            for(int x = 0; x < w; x++) // traverse all columns
            {
                ptsMeshGrid[indexRowChannel++] = x;
                ptsMeshGrid[indexRowChannel++] = y;
            }
            pts.put(indexRow, 0, ptsMeshGrid);
            indexRow += indexRowChannel/c;
        }

        //     dpts = cv2.undistortPoints(pts.astype(np.float32), K, dist, P=Knew)

        MatOfPoint2f dpts = new MatOfPoint2f();

        Calib3d.undistortPoints(pts, dpts, K, dist, new Mat(), Knew);

        Mat dpts2D = dpts.reshape(2, h)/*.convertTo(dpts2D, CvType.CV_32FC2 )*/; //FIXME convert or not??????

        Main.LOGGER.log(Level.WARNING, "pts " + pts + "\n" + brief(pts));
        Main.LOGGER.log(Level.WARNING, "dpts " + dpts + "\n" + brief(dpts));
        Main.LOGGER.log(Level.WARNING, "returning dpts2D " + dpts2D + brief(dpts2D));
        Main.LOGGER.log(Level.WARNING, "maybe returning Knew\n" + Knew.dump());

        pts.release();
        dpts.release();

//     return dpts.reshape(sz[0], sz[1], 2).T
        return dpts2D;
    }
// def make_distort_map(K, sz, dist, Knew):
//     """
//     creates a map for distorting an image as a opposed to the default
//     behaviour of undistorting
//     @param sz: width, height
//     """
//     pts = np.array(np.meshgrid(range(sz[0]), range(sz[1]))).T.reshape(-1, 1, 2)
//     dpts = cv2.undistortPoints(pts.astype(np.float32), K, dist, P=Knew)
//     return dpts.reshape(sz[0], sz[1], 2).T
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     sparse_undistort_map                                                   */
/*                                     sparse_undistort_map                                                   */
/*                                     sparse_undistort_map                                                   */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    //     same output as initUndistortRectifyMap, but sparse
    //     @param sz: width, height
    //     @return: distorted points, original points
    public static List<Mat> sparse_undistort_map(Mat K, Size sz, Mat dist, Mat Knew, int step)
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // best I can tell step is always 20 (subsample) and never 1 so this should not be executed
        if(step == 1) throw new IllegalArgumentException("step = 1 full image sampling not converted and tested");
        //     zero = np.zeros(3)
        //     pts = np.array(np.meshgrid(range(0, sz[0], step), range(0, sz[1], step))).T.reshape(-1, 1, 2)
        // inclusive 0 to step
        int w = (int)sz.width/step; // columns
        int h = (int)sz.height/step; // rows
        int c = 2; // x and y axes
        int index; // make smaller 2-D Mat of x,y points from full size image Mat
        // the values in the smaller Mat are the original x, y coordinates from the larger Mat

        MatOfPoint2f pts = new MatOfPoint2f();
        pts.alloc(h*w);
        float[] ptsMeshGrid = new float[w*h*c]; // intermediate 2d points

        index = 0;
        for(float y = 0.f; y<h; y+=step)
        for(float x = 0.f; x<w; x+=step)
        {
            ptsMeshGrid[index++] = x;
            ptsMeshGrid[index++] = y;
        }
        pts.put(0, 0, ptsMeshGrid);

        MatOfPoint2f ptsUndistorted = new MatOfPoint2f(); // intermediate 2d points
        MatOfPoint3f pts3d = new MatOfPoint3f(); // 3d points
        Mat noTransforms = Mat.zeros(3, 1, CvType.CV_32FC1);

        Calib3d.undistortPoints(pts, ptsUndistorted, Knew, new Mat()); // undistort the 2d points

        Calib3d.convertPointsToHomogeneous(ptsUndistorted, pts3d); // now convert 2d to 3d homogeneous
        // n by 2 or 3 dimensions in or 2 or 3 dimensions by n in; always nx1x 3 or 4 channels out
        // a dimension is either a Mat row or column or 1 row or column and 2 or 3 channels
        // 1xnx2, 1xnx3, nx1x2, nx1x3, nx2x1, nx3x1 in; always nx1x3 or nx1x4 out

        MatOfDouble distOfDouble = new MatOfDouble(dist); // convert as required for the projectPoints()

        MatOfPoint2f dpts = new MatOfPoint2f();

        Calib3d.projectPoints(pts3d, noTransforms, noTransforms, Knew, distOfDouble, dpts); // project points in 3d back to a 2d screen
        int[] shape = {w, h};
        Mat dpts2D = dpts.reshape(c, shape);
        Mat pts2D = pts.reshape(c, shape);
        // reformat sparse flat MatOfPoint2f to 2-D 2-channel rectangular subsample map
        List<Mat> maps = new ArrayList<Mat>(2);

        pts.release();
        ptsUndistorted.release();
        pts3d.release();
        noTransforms.release();
        distOfDouble.release();
        dpts.release();

        maps.add(dpts2D);
        maps.add(pts2D);

        return maps;
    }
    // def sparse_undistort_map(K, sz, dist, Knew, step=1):
    // """
    // same output as initUndistortRectifyMap, but sparse
    // @param sz: width, height
    // @return: distorted points, original points
    // """
    // print("sparse_undistort_map", sz, step, sep="><")
 
    // zero = np.zeros(3)
    // pts = np.array(np.meshgrid(range(0, sz[0], step), range(0, sz[1], step))).T.reshape(-1, 1, 2) #long list of (x, y) points out
    // print("pts", np.shape(pts), type(pts), pts, sep="><")
    // """
    // sparse_undistort_map><(1280, 720)><20
    // pts><(2304, 1, 2)><<class 'numpy.ndarray'>><[[[   0    0]]
    // [[   0   20]]
    // [[   0   40]]
    // ...
    // [[1260  660]]
    // [[1260  680]]
    // [[1260  700]]]
    // """
    // if step == 1:
    //     dpts = cv2.initUndistortRectifyMap(K, dist, None, Knew, sz, cv2.CV_32FC2)[0].transpose(1, 0, 2)
    // else:
    //     pts3d = cv2.undistortPoints(pts.astype(np.float32), Knew, None)
    //     pts3d = cv2.convertPointsToHomogeneous(pts3d).reshape(-1, 3) # (x, y, 1) points out
    //     dpts = cv2.projectPoints(pts3d, zero, zero, K, dist)[0]

    // shape = (sz[0] // step, sz[1] // step, 2) # size of rectangular array of samples

    // return dpts.reshape(-1, 2).reshape(shape), pts.reshape(shape) # rectangular arrays of samples returned

/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     loc_from_dist                                                          */
/*                                     loc_from_dist                                                          */
/*                                     loc_from_dist                                                          */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    /**
     *     compute location based on distortion strength
     * @param pts: sampling locations
     * @param dpts: distorted points
     * @param mask: mask for ignoring locations
     * @param lower: find location with minimal distortion instead
     * @param thres: distortion strength to use as threshold [%]
     * @return
     */
    public static Rect loc_from_dist(Mat pts, Mat dpts, Mat mask, boolean lower, double thres) // force specifying all parameters
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        Main.LOGGER.log(Level.WARNING, "pts " + pts);
        Main.LOGGER.log(Level.WARNING, "dpts " + dpts);
        Main.LOGGER.log(Level.WARNING, "mask " + mask);
        Main.LOGGER.log(Level.WARNING, "lower " + lower);
        Main.LOGGER.log(Level.WARNING, "thres " + thres);
/*
pts =  np.array([[[   0,    0], [   0,   20],  [   0,   40]],
[[   0,    0], [   0,   20],  [   0,   40]]])
dpts = np.array([[[-3.3855473e+01, -1.7103071e+01], [-3.0816502e+01,  5.4372540e+00], [-2.8238459e+01,  2.7548717e+01] ],
[[-3.3855473e+01, -1.7103071e+01], [-3.0816502e+01,  5.4372540e+00], [-2.8238459e+01,  2.7548717e+01] ]])
        
diff [[[33.855473 17.103071]
[30.816502 14.562746]     
[28.238459 12.451283]]    

[[33.855473 17.103071]
[30.816502 14.562746]
[28.238459 12.451283]]]
diff1 [37.93030569 34.08416592 30.86170791 37.93030569 34.08416592 30.86170791]
diff2 [[37.93030569 37.93030569]
[34.08416592 34.08416592]
[30.86170791 30.86170791]]
diff3 [[255 255]
[116 116]
[  0   0]]
*/
        // pts><(64, 36, 2) dpts><(64, 36, 2)
        // pairs of points p - dp
        // want an array of norms of element by element not one total for the whole input arrays.
        // sqr root of the sum of the sqrs
        // points on row/col locations are a 2D mat with 2 channels - channel x, channel y
        // the norm of each point x,y is norm = sqrt(x^2 + y^2) is computed and put in the same row/col location
        // the pair of points in row=0 col=0 of pts and dpts have their norm computed and put in output row=0 col=0
        // x is in channel 0 and y is in channel 1
        // norm(0,0) = sqrt( (p(0,0)x-dp(0,0)x)^2 + (p(0,0)y-dp(0,0)z)^2 )
        Mat diffpts = new Mat();
        Core.subtract(pts, dpts, diffpts);
        Main.LOGGER.log(Level.WARNING, "diffpts " + diffpts);

        Mat normMat = new Mat(pts.rows(), pts.cols(), CvType.CV_32FC1);
        Main.LOGGER.log(Level.WARNING, "normMat empty " + normMat);

        for(int row = 0; row < pts.rows(); row++)
        for(int col = 0; col < pts.cols(); col++)
        {
            float[] point = new float[2]; // get the 2 channels of data x in 0 and y in 1
            diffpts.get(row, col, point);
            float norm = (float)Math.sqrt(Math.pow(point[0], 2) + Math.pow(point[1], 2)); // L2 norm (Frobenious)
            normMat.put(row, col, norm);
        }
        Main.LOGGER.log(Level.WARNING, "normMat filled " + normMat);

        normMat = normMat.reshape(0,mask.rows())/*.t()*/;
        Main.LOGGER.log(Level.WARNING, "normMat reshaped " + normMat);

        Mat diff = new Mat();
        Core.normalize(normMat, diff, 0, 255, Core.NORM_MINMAX, CvType.CV_8U);
        // Main.LOGGER.log(Level.WARNING, "diff " + diff.dump());
        Main.LOGGER.log(Level.WARNING, "normMat normalized=diff " + diff);

        Rect bounds = null;

        Mat thres_img = new Mat();
        while ((bounds == null) && (thres >= 0.) && (thres <= 1.) )
        {
            if (lower)
            {
                thres += 0.05;
                Imgproc.threshold(diff, thres_img, thres * 255., 255., Imgproc.THRESH_BINARY_INV);
            }
            else
            {
                thres -= 0.05;
                Imgproc.threshold(diff, thres_img, thres * 255., 255., Imgproc.THRESH_BINARY);
            }
            Main.LOGGER.log(Level.WARNING, "thres_img " + thres_img /*+ "\n" + brief(thres_img.dump())*/);

            bounds = get_bounds(thres_img, mask);

            if (bounds == null)
            {
                continue;
            }

            if (bounds.width*bounds.height == 0) // ensure area is not 0
            {
                bounds = null;                
            }
        }

        normMat.release();
        diff.release();
        diffpts.release();

        Main.LOGGER.log(Level.WARNING, "bounds " + (bounds == null ? "is null" : bounds));

        return bounds;
    }
}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     End Distortion class                                                                       */
/*                                     End Distortion class                                                                       */
/*                                     End Distortion class                                                                       */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*
 Calib3d.convertPointsToHomogeneous()
        n by 2 or 3 dimensions in or 2 or 3 dimensions by n in; always nx1x 3 or 4 channels out
        a dimension is either a Mat row or column or 1 row or column and 2 or 3 channels
        1xnx2, 1xnx3, nx1x2, nx1x3, nx2x1, nx3x1 in; always nx1x3 or nx1x4 out
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// import cv2

// import numpy as np
// import numpy.linalg as la

// def get_bounds(thresh, mask):
//     MAX_OVERLAP = 0.9
//     contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
//     contours = list(contours) # revert to the old way that contours is a list and not a tuple as in the new way
    
//     # look for the largest object that is not masked
//     while contours:
//         mx = np.argmax([len(c) for c in contours])
//         contour = contours[mx]
//         aabb = cv2.boundingRect(contour)

//         x, y, w, h = aabb
//         if mask is not None and (cv2.countNonZero(mask[y:y + h, x:x + w]) / (w * h) > MAX_OVERLAP):
//             del contours[mx]  # remove from candidates the old way from the list
//             #contours = contours[:mx] + contours[mx+1:] # the new way from the tuple
//             continue

//         return (aabb, contour)

//     return None

// def make_distort_map(K, sz, dist, Knew):
//     """
//     creates a map for distorting an image as a opposed to the default
//     behaviour of undistorting
//     @param sz: width, height
//     """
//     pts = np.array(np.meshgrid(range(sz[0]), range(sz[1]))).T.reshape(-1, 1, 2)
//     dpts = cv2.undistortPoints(pts.astype(np.float32), K, dist, P=Knew)

//     return dpts.reshape(sz[0], sz[1], 2).T

// def sparse_undistort_map(K, sz, dist, Knew, step=1):
//     """
//     same output as initUndistortRectifyMap, but sparse
//     @param sz: width, height
//     @return: distorted points, original points
//     """
//     zero = np.zeros(3)
//     pts = np.array(np.meshgrid(range(0, sz[0], step), range(0, sz[1], step))).T.reshape(-1, 1, 2)

//     if step == 1:
//         dpts = cv2.initUndistortRectifyMap(K, dist, None, Knew, sz, cv2.CV_32FC2)[0].transpose(1, 0, 2)
//     else:
//         pts3d = cv2.undistortPoints(pts.astype(np.float32), Knew, None)
//         pts3d = cv2.convertPointsToHomogeneous(pts3d).reshape(-1, 3)
//         dpts = cv2.projectPoints(pts3d, zero, zero, K, dist)[0]

//     shape = (sz[0] // step, sz[1] // step, 2)

//     return dpts.reshape(-1, 2).reshape(shape), pts.reshape(shape)

// def loc_from_dist(pts, dpts, mask=None, lower=False, thres=1.0):
//     """
//     compute location based on distortion strength
//     @param pts: sampling locations
//     @param dpts: distorted points
//     @param mask: mask for ignoring locations
//     @param lower: find location with minimal distortion instead
//     @param thres: distortion strength to use as threshold [%]
//     """
//     diff = la.norm((pts - dpts).reshape(-1, 2), axis=1)
//     diff = diff.reshape(pts.shape[0:2]).T
//     diff = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

//     bounds = None

//     while not bounds and thres >= 0 and thres <= 1:
//         if lower:
//             thres += 0.05
//             thres_img = cv2.threshold(diff, thres * 255, 255, cv2.THRESH_BINARY_INV)[1]
//         else:
//             thres -= 0.05
//             thres_img = cv2.threshold(diff, thres * 255, 255, cv2.THRESH_BINARY)[1]

//         bounds = get_bounds(thres_img, mask)

//         if bounds is None:
//             continue

//         # ensure area is not 0
//         if bounds[0][2] * bounds[0][3] == 0:
//             bounds = None
    
//     if bounds is None:
//         return None, None

//     return np.array(bounds[0]), thres_img

//////////////////////////////////
            // for get_bounds this is likely a start of better but something to consider later:
            // use true area instead of counting the perimeter segments
            // var areaContour = Imgproc.contourArea(contours.get(i)); // this might be better area but watch for area = 0.0 if only 2 points in contour

            // use true contour instead of bounding rect
            // if(!mask.empty())
            // {
            //     Mat tempContour = Mat.zeros(thresh.rows(), thresh.cols(), thresh.type());
            //     Imgproc.drawContours(tempContour, contours, locMax, new Scalar(1.), 0, Imgproc.FILLED);
            //     Core.bitwise_and(tempContour, mask, tempContour); // overlap of thresh's contour and mask
            //     var countOverlapPixels =Core.countNonZero(tempContour);
            //     if (countOverlapPixels/areaContourMax > MAX_OVERLAP)
            //         {
            //             contours.remove(locMax);
            //             continue;
            //         }
            // }
/////////////////////////////////