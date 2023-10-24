package calibrator;

import static calibrator.ArrayUtils.brief;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import java.util.logging.Level;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     PoseGeneratorDist class                                                */
/*                                     PoseGeneratorDist class                                                */
/*                                     PoseGeneratorDist class                                                */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
public class PoseGeneratorDist {
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     Gen_Bin class                                                          */
/*                                     Gen_Bin class                                                          */
/*                                     Gen_Bin class                                                          */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    // generate values in range by binary subdivision
    // note that the list of values grows exponentially with each call for a value
    // seems like a poor algorithm to me but it is used sparsely in this application
    class Gen_Bin
    {
        private List<Double> lst = new ArrayList<>(40); // always manipulate 2 items at a time instead of making a new datatype for a pair - not so safe, though
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     Gen_Bin constructor                                                    */
/*                                     Gen_Bin constructor                                                    */
/*                                     Gen_Bin constructor                                                    */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
        private Gen_Bin(double s, double e)
        {
            double t = (s + e) / 2.;
            this.lst.add(s);
            this.lst.add(t);
            this.lst.add(t);
            this.lst.add(e);
        }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     next                                                                   */
/*                                     next                                                                   */
/*                                     next                                                                   */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
        private double next()
        {
            double s = this.lst.get(0); // pop the top
            this.lst.remove(0);

            double e = this.lst.get(0); // pop the new top
            this.lst.remove(0);

            double t = (s + e) / 2.;
            this.lst.add(s);
            this.lst.add(t);
            this.lst.add(t);
            this.lst.add(e);

            return t;
        }
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     unproject                                                              */
/*                                     unproject                                                              */
/*                                     unproject                                                              */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/**
 *     project pixel back to a 3D coordinate at depth Z
 * @param p
 * @param K
 * @param cdist
 * @param Z
 * @return
 */
    private static Mat unproject(MatOfPoint2f p, Mat K, Mat cdist, double Z)
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // returns a translation (t) Mat
        Main.LOGGER.log(Level.WARNING, "p in " + p.dump());
        Main.LOGGER.log(Level.WARNING, "camera matrix K " + K + "\n" + K.dump());
        Main.LOGGER.log(Level.WARNING, "cdist " + cdist.dump());
        Main.LOGGER.log(Level.WARNING, "Z " + Z);

        Main.Kcsv(Id.__LINE__(), K);

        Calib3d.undistortPoints(p, p, K, cdist);

        Main.LOGGER.log(Level.WARNING, "p out " + p.dump());

        double[] pXY = p.get(0, 0); // get X and Y channels for the point (ravel)
 
        Mat p3D = new Mat(1, 3, CvType.CV_64FC1);
        p3D.put(0, 0, pXY[0], pXY[1], 1.); // x, y, 1

        Core.multiply(p3D, new Scalar(Z, Z, Z), p3D);

        Main.LOGGER.log(Level.WARNING, "return p3D Z scaled " + p3D.dump());

        return p3D;
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     orbital_pose                                                           */
/*                                     orbital_pose                                                           */
/*                                     orbital_pose                                                           */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/**
 * 
 * @param bbox object bounding box. note: assumes planar object with virtual Z dimension.
 * @param rx rotation around x axis in rad
 * @param ry rotation around y axis in rad
 * @param Z distance to camera in board lengths
 * @param rz
 * @return rvec, tvec
 */
    private static List<Mat> orbital_pose(Mat bbox, double rx, double ry, double Z, double rz) // force caller to use rz=0 if defaulting
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        Main.LOGGER.log(Level.WARNING, "bbox " + bbox + "\n" + bbox.dump());
        Main.LOGGER.log(Level.WARNING, "rx " + rx);
        Main.LOGGER.log(Level.WARNING, "ry " + ry);
        Main.LOGGER.log(Level.WARNING, "Z " + Z);
        Main.LOGGER.log(Level.WARNING, "rz " + rz);
        
        // compute rotation matrix from rotation vector
        double[] angleZ = {0., 0., rz};
        double[] angleX = {Math.PI + rx, 0., 0.}; // flip by 180° so Z is up
        double[] angleY = {0., ry, 0.};
        Mat angleZVector = new Mat(3, 1, CvType.CV_64FC1);
        Mat angleXVector = new Mat(3, 1, CvType.CV_64FC1);
        Mat angleYVector = new Mat(3, 1, CvType.CV_64FC1);
        angleZVector.put(0, 0, angleZ);       
        angleXVector.put(0, 0, angleX);
        angleYVector.put(0, 0, angleY);
        Mat Rz = new Mat();
        Mat Rx = new Mat();
        Mat Ry = new Mat();

        /********************************************************************************************************* */
        Calib3d.Rodrigues(angleZVector, Rz);
        Calib3d.Rodrigues(angleXVector, Rx);
        Calib3d.Rodrigues(angleYVector, Ry);
        /********************************************************************************************************* */

        Main.LOGGER.log(Level.WARNING, "Rz\n" + Rz.dump());
        Main.LOGGER.log(Level.WARNING, "Rx\n" + Rx.dump());
        Main.LOGGER.log(Level.WARNING, "Ry\n" + Ry.dump());

        // in Python (Ry).dot(Rx).dot(Rz) messed up nomenclature - it's often really matrix multiply Ry times Rx times Rz
        Mat R = Mat.eye(4, 4, CvType.CV_64FC1);
        Mat R3x3 = R.submat(0, 3, 0, 3);

        /********************************************************************************************************* */
        Core.gemm(Ry, Rx, 1., new Mat(), 0, R3x3);
        Core.gemm(R3x3, Rz, 1., new Mat(), 0., R3x3); // rotation matrix of the input Euler Angles [radians]
        /********************************************************************************************************* */
        
        angleZVector.release();
        angleXVector.release();
        angleYVector.release();
        Rz.release();
        Rx.release();
        Ry.release();

        // translate board to its center
        Mat Tc = Mat.eye(4, 4, CvType.CV_64FC1);
        Mat Tc1x3 = Tc.submat(3, 4, 0, 3);
        Mat Tc3x1 = new Mat(); // temp for Rodrigues output
        Mat translateToBoardCenter = new Mat(bbox.rows(), bbox.cols(), bbox.type()); // matching bbox for element by element multiply
        translateToBoardCenter.put(0, 0, -0.5, -0.5, 0.);
        translateToBoardCenter = bbox.mul(translateToBoardCenter);
        Main.LOGGER.log(Level.WARNING, "translateToBoardCenter\n" + translateToBoardCenter.dump());

        // Main.LOGGER.log(Level.WARNING, "R " + R.dump());
        // Main.LOGGER.log(Level.WARNING, "R3x3 " + R3x3.dump());
        // Main.LOGGER.log(Level.WARNING, "Tc " + Tc.dump());
        // Main.LOGGER.log(Level.WARNING, "Tc1x3 " + Tc1x3.dump());

        /********************************************************************************************************* */
        Core.gemm(R3x3, translateToBoardCenter, 1., new Mat(), 0.,Tc3x1);
        Tc3x1.t().copyTo(Tc1x3); // update Tc
        /********************************************************************************************************* */
        // Main.LOGGER.log(Level.WARNING, "Tc " + Tc.dump());
        // Main.LOGGER.log(Level.WARNING, "Tc1x3 " + Tc1x3.dump());

        // translate board to center of image

        Mat T = Mat.eye(4, 4, CvType.CV_64FC1);
        Mat T1x3 = T.submat(3, 4, 0, 3);     
        /********************************************************************************************************* */
        Mat translateToImageCenter = new Mat(bbox.rows(), bbox.cols(), bbox.type()); // matching bbox for element by element multiply
        translateToImageCenter.put(0, 0, -0.5, -0.5, Z);
        bbox.mul(translateToImageCenter).t().copyTo(T1x3);   
        /********************************************************************************************************* */
        // Main.LOGGER.log(Level.WARNING, "translateToImageCenter " + translateToImageCenter.dump());
        // Main.LOGGER.log(Level.WARNING, "T1x3 " + T1x3.dump());
        // Main.LOGGER.log(Level.WARNING, "T " + T.dump());

        // rotate center of board
        Mat Rf = new Mat();
        
        /********************************************************************************************************* */
        Core.gemm(Tc.inv(), R, 1., new Mat(), 0.,Rf);
        Core.gemm(Rf, Tc, 1., new Mat(), 0.,Rf);
        Core.gemm(Rf, T, 1., new Mat(), 0.,Rf);
        /********************************************************************************************************* */
        // Main.LOGGER.log(Level.WARNING, "Rf " + Rf.dump());

        // return cv2.Rodrigues(Rf[:3, :3])[0].ravel(), Rf[3, :3]
        Mat Rf3x3 = Rf.submat(0, 3, 0, 3);
        Mat RfVector = new Mat(1, 3, CvType.CV_64FC1);

        /********************************************************************************************************* */
        Calib3d.Rodrigues(Rf3x3,RfVector);
        Core.transpose(RfVector, RfVector);
        /********************************************************************************************************* */
        Main.LOGGER.log(Level.WARNING, "RfVector returned " + RfVector.dump());

        Mat t = Rf.submat(3, 4, 0, 3);
        Mat tVector = new Mat();
        t.copyTo(tVector);
        Main.LOGGER.log(Level.WARNING, "tVector returned " + tVector.dump());

        Tc.release();
        Tc1x3.release();
        translateToBoardCenter.release();
        translateToImageCenter.release();
        T.release();
        T1x3.release();
        Rf.release();
        Rf3x3.release();
        t.release();
        // OpenCV uses reference counting.
        // submat adds another reference to the data memory.
        // .release() does not deallocate the memory, unless the last reference was removed/decremented.

        List<Mat> rt = new ArrayList<Mat>(2);
        rt.add(RfVector);
        rt.add(tVector);

        return rt;
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     pose_planar_fullscreen                                                 */
/*                                     pose_planar_fullscreen                                                 */
/*                                     pose_planar_fullscreen                                                 */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    private static List<Mat> pose_planar_fullscreen(Mat K, Mat cdist, Size img_size, Mat bbox)
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // don't use the principal point throughout just have X and Y no Z until it's calculated in the middle
        // compute a new Z
        Main.LOGGER.log(Level.WARNING, "camera matrix K " + K + "\n" + K.dump());
        Main.LOGGER.log(Level.WARNING, "cdist " + cdist.dump());
        Main.LOGGER.log(Level.WARNING, "img_size " + img_size.toString());
        Main.LOGGER.log(Level.WARNING, "bbox " + bbox + "\n" + bbox.dump());

        Mat KB = new Mat(); // ignore principal point
        Mat bboxZeroZ = new Mat(3, 1, CvType.CV_64FC1);
        bboxZeroZ.put(0, 0, bbox.get(0, 0)[0], bbox.get(1, 0)[0], 0.);
        Main.Kcsv(Id.__LINE__(), K);
        Core.gemm(K, bboxZeroZ,1., new Mat(), 0., KB);
        Main.Kcsv(Id.__LINE__(), KB);
        double KBx = KB.get(0, 0)[0];
        double KBy = KB.get(1, 0)[0];
        double Z = Math.min(KBx/img_size.width, KBy/img_size.height);
        double[] pB = {KBx/Z, KBy/Z};

        Mat r = new Mat(1, 3, CvType.CV_64FC1);
        r.put(0, 0, Math.PI, 0., 0.); // flip image

        // move board to center, org = bl
        MatOfPoint2f p = new MatOfPoint2f(new Point(img_size.width/2. - pB[0]/2., img_size.height/2. + pB[1]/2.));
        
        Mat t = unproject(p, K, cdist, Z);

        Main.LOGGER.log(Level.WARNING, "KBnoPrinciplePoint " + KB + KB.dump());
        Main.LOGGER.log(Level.WARNING, "Z, pB(x, y) " + Z + ", " + Arrays.toString(pB));
        Main.LOGGER.log(Level.WARNING, "p " + p + p.dump());
        Main.LOGGER.log(Level.WARNING, "returning r " + r + r.dump());
        Main.LOGGER.log(Level.WARNING, "returning t " + t + t.dump());

        bboxZeroZ.release();
        KB.release();
        p.release();

        List<Mat> rt = new ArrayList<>(2);
        rt.add(r); // don't release r; this is the object that is returned
        rt.add(t); // don't release t; this is the object that is returned
        return rt;
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     pose_from_bounds                                                       */
/*                                     pose_from_bounds                                                       */
/*                                     pose_from_bounds                                                       */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    /**
    * 
    * @param src_extParm
    * @param tgt_rect
    * @param K
    * @param cdist
    * @param img_sz
    * @return
    */
    private static List<Object> pose_from_bounds(Mat src_extParm, Rect tgt_rect, Mat K, Mat cdist, Size img_sz)
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        Main.LOGGER.log(Level.WARNING, "src_extParm " + src_extParm + "\n" + src_extParm.dump()); // full ChArUcoBoard size + Z
        Main.LOGGER.log(Level.WARNING, "tgt_rect " + tgt_rect); // guidance board posed
        Main.LOGGER.log(Level.WARNING, "camera matrix K " + K + "\n" + K.dump());
        Main.LOGGER.log(Level.WARNING, "cdist " + cdist.dump());
        Main.LOGGER.log(Level.WARNING, "img_sz " + img_sz.toString()); // camera screen image size
        Main.Kcsv(Id.__LINE__(), K);

        double[] src_ext = new double[(int)src_extParm.total()];
        src_extParm.get(0, 0, src_ext);

        boolean rot90 = tgt_rect.height > tgt_rect.width;

        int MIN_WIDTH = (int)Math.floor(img_sz.width/3.333); // posed guidance board must be about a third or more of the camera, screen size

        Main.LOGGER.log(Level.WARNING, "rot90 " + rot90);

        if(rot90)
        {
            // flip width and height
            // src_ext = src_ext.clone(); // cloning not needed in this implementation since already hiding parameter src_extParm
            double swapWH = src_ext[0];
            src_ext[0] = src_ext[1];
            src_ext[1] = swapWH;

            if (tgt_rect.height < MIN_WIDTH)
            {
                // double scale = MIN_WIDTH / tgt_rect.width; //FIXME is this wrong in original? tgt_rect.width => tgt_rect.height? - rkt
                double scale = MIN_WIDTH / tgt_rect.height;
                tgt_rect.height = MIN_WIDTH;
                tgt_rect.width *= scale;
            }
        }
        else
        {
            if (tgt_rect.width < MIN_WIDTH)
            {
                double scale = MIN_WIDTH / tgt_rect.width;
                tgt_rect.width = MIN_WIDTH;
                tgt_rect.height *= scale;
            }
        }
        double aspect = src_ext[0] / src_ext[1]; // w/h of the full ChArUcoBoard // [2520; 1680; 2520] => 1.5

        // match aspect ratio of tgt to src, but keep tl
        if (!rot90)
        {
            tgt_rect.height = (int)(tgt_rect.width / aspect); // adapt height
        }
        else
        {
            tgt_rect.width = (int)(tgt_rect.height * aspect); // adapt width
        }
        // logic error here (missing check), I'm sure, so fix it - rkt
        // if target too wide reduce to image size; if target too high reduce to image size
        if (tgt_rect.width > img_sz.width)
        {
            aspect = img_sz.width/tgt_rect.width;
            tgt_rect.width = (int)(tgt_rect.height * aspect); // adapt width            
            tgt_rect.height = (int)(tgt_rect.width * aspect); // adapt height
        }
        if (tgt_rect.height > img_sz.height)
        {
            aspect = img_sz.height/tgt_rect.height;
            tgt_rect.width = (int)(tgt_rect.height * aspect); // adapt width            
            tgt_rect.height = (int)(tgt_rect.width * aspect); // adapt height
        }

        Mat r = new Mat(1, 3, CvType.CV_64FC1);
        r.put(0, 0, Math.PI, 0., 0.);
        
        // org is bl (bottom left)
        if (rot90)
        {
            Mat R = new Mat();

            Calib3d.Rodrigues(r, R);

            Main.LOGGER.log(Level.WARNING, "R " + R.dump());
            Mat rz = new Mat(1, 3, CvType.CV_64FC1);
            rz.put(0, 0, 0., 0., -Math.PI/2.);
            Mat Rz = new Mat();

            Calib3d.Rodrigues(rz, Rz);

            Main.LOGGER.log(Level.WARNING, "Rz " + Rz.dump());

            Core.gemm(R, Rz, 1., new Mat(), 0., R); // rotation matrix of the input Euler Angles

            Calib3d.Rodrigues(R, r);

            r = r.t(); // (ravel) Rodrigues out is 3x1 and (most of) the rest of program is 1x3

            R.release();
            Rz.release();
            rz.release();
            // org is tl (top left)
        }

        double Z = (K.get(0, 0)[0] * src_ext[0]) / tgt_rect.width;
        Main.LOGGER.log(Level.WARNING, "before clip tgt_rect " + tgt_rect);

        //  clip to image region
        int[] min_off = {0, 0};
        int[] max_off = {(int)(img_sz.width - tgt_rect.width), (int)(img_sz.height - tgt_rect.height)};
        tgt_rect.x = Math.min(max_off[0], Math.max(tgt_rect.x, min_off[0]));
        tgt_rect.y = Math.min(max_off[1], Math.max(tgt_rect.y, min_off[1]));
        Main.LOGGER.log(Level.WARNING, "after clip tgt_rect " + tgt_rect);

        if ( ! rot90)
        {
            tgt_rect.y += tgt_rect.height;
        }

        MatOfPoint2f p = new MatOfPoint2f(new Point(tgt_rect.x, tgt_rect.y));

        Mat t = unproject(p, K, cdist, Z);

        if ( ! rot90)
        {
            tgt_rect.y -= tgt_rect.height;
        }

        Main.LOGGER.log(Level.WARNING, "returning r " + r.dump());
        Main.LOGGER.log(Level.WARNING, "returning t " + t.dump());
        Main.LOGGER.log(Level.WARNING, "returning tgt_rect " + tgt_rect);

        List<Object> rtb = new ArrayList<>(3);
        rtb.add(r); // position 0
        rtb.add(t); // position 1
        rtb.add(tgt_rect); // position 2

        return rtb;
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     PoseGeneratorDist Constructor                                          */
/*                                     PoseGeneratorDist Constructor                                          */
/*                                     PoseGeneratorDist Constructor                                          */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    // two angle bins - one for the x axis [0] and one for the y axis [1]
    private Gen_Bin[] gb = {new Gen_Bin(Math.toRadians(-70.), Math.toRadians(70.)),  // valid poses: r_x -> -70° .. 70°
                            new Gen_Bin(Math.toRadians(-70.), Math.toRadians(70.))}; // valid poses: r_y -> -70° .. 70°
    /**
    * getter for the angle bins per axis
    * 
    * @param axis x = 0; y = 1
    * @return next angle iteration for the selected axis
    */
    private Function<Integer, double[]> orbital = (axis) ->
    {
        // !!!! NOT a normal function - it returns a different value each time
        // from the angle binning "iterator"
        /*
        * generate next angles
        *   {next x ,      0} for the x axis [0]
        * OR 
        *   {     0 , next y} for the y axis [1]
        */
        double[] angle = { 0., 0.};
        angle[axis] = gb[axis].next();
        return angle;
    };

    private static final int SUBSAMPLE = 20; // there is not a good conversion in general for this type of Python variable. Assuming value doesn't change nor instantiated more than once and changed elsewhere this conversion works.
    private Size img_size;
    // self.stats = [1, 1]  # number of (intrinsic, distortion) poses -- NOT USED
    private double orbitalZ = 1.6;
    private static final double rz = Math.PI / 8.;

    private Mat mask;
    private double sgn = 1.;

    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}

/**
 *     generate poses based on min/ max distortion
 * @param img_size
 */
    PoseGeneratorDist(Size img_size)
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        this.img_size = img_size;
        mask = Mat.zeros(
            new Size(Math.floor(img_size.width/this.SUBSAMPLE), Math.floor(img_size.height/this.SUBSAMPLE)),
            CvType.CV_8UC1); // t() transpose not needed in Java since those bindings account for w-h reversal
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     compute_distortion                                                     */
/*                                     compute_distortion                                                     */
/*                                     compute_distortion                                                     */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    private List<Mat> compute_distortion(Mat K, Mat cdist, int subsample)
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        return Distortion.sparse_undistort_map(K, img_size, cdist, K, subsample);
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     get_pose                                                               */
/*                                     get_pose                                                               */
/*                                     get_pose                                                               */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*
 * Determines (returns) the rotation and translation vectors to apply to the guidance board to effect
 * the desired pose that was determined to collect information to converge on the correct intrinsics.
 * 
 * @param bbox bounding box size of the calibration pattern
 * @param nk number of keyframes captured so far
 * @param tgt_param intrinic number of parameter that should be optimized by the pose
 * @param K camera matrix current calibration estimate
 * @param cdist distortion coefficients current calibration estimate
 * @return rotation vector and translation vector
 * @throws Exception
 */
    List<Mat> get_pose(Mat bbox, int nk, int tgt_param, Mat K, Mat cdist) throws Exception
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        Main.LOGGER.log(Level.WARNING, "bbox " + bbox + "\n" + bbox.dump());
        Main.LOGGER.log(Level.WARNING, "nk " + nk);
        Main.LOGGER.log(Level.WARNING, "tgt_param " + tgt_param);
        Main.LOGGER.log(Level.WARNING, "camera matrix K " + K + "\n" + K.dump());
        Main.LOGGER.log(Level.WARNING, "cdist " + cdist.dump());
        Main.Kcsv(Id.__LINE__(), K);
        if (nk == 0)
        {
            // x is camera pointing ahead up/down; y is camera pointing left/right; z is camera rotated (Z is axis from camera to target)
            // init sequence: first keyframe  0° flat - not up or down, 45° pointing left, 22.5° rotated CW

            /********************************************************************************************************* */
            return orbital_pose(bbox, 0., Math.PI/4., this.orbitalZ, Math.PI/8./*probably this.rz*/);
            /********************************************************************************************************* */
        }
        if (nk == 1)
        {
            // init sequence: second keyframe

            /********************************************************************************************************* */
            return pose_planar_fullscreen(K, cdist, this.img_size, bbox);
            /********************************************************************************************************* */
        }
        if (tgt_param < 4)
        {
            // orbital pose is used for focal length
            int axis = (tgt_param + 1) % 2;  // f_y -> r_x
            
            // r, t = orbital_pose(bbox, *next(self.orbital[axis]));

            double[] angleIteration = this.orbital.apply(axis); // get the next iteration for x & y pair of angles for the given axis
            Main.LOGGER.log(Level.WARNING, "angleIteration " + Arrays.toString(angleIteration));

            /********************************************************************************************************* */
            List<Mat> rt = orbital_pose(bbox, angleIteration[0], angleIteration[1], this.orbitalZ, this.rz);
            /********************************************************************************************************* */
            
            Main.LOGGER.log(Level.WARNING, "rt " + rt.get(0).dump() + rt.get(1).dump());
            
            Mat t = rt.get(1);

            if (tgt_param > 1)
            {
                // nudge the principal point in the axis in process and unproject it
                // "off" is Principal Point Cx, Cy from the intrinsics camera matrix
                double[] offPPoint = {K.get(0, 2)[0], K.get(1, 2)[0]};
                offPPoint[tgt_param - 2] += ((tgt_param - 2) == 0 ? this.img_size.width : this.img_size.height) * 0.05 * this.sgn;
                MatOfPoint2f off = new MatOfPoint2f(new Point(offPPoint));

                /********************************************************************************************************* */
                Mat off3d = unproject(off, K, cdist, t.get(0, 2)[0]);
                /********************************************************************************************************* */

                off3d.put(0, 2, 0.); // zero out the Z
                Core.add(t, off3d, t); // pretty big offset being nearly the principal point
                this.sgn = -this.sgn;
                off.release();
                off3d.release();
            }

            rt.set(1, t); // update the nudged t and return r and t

            Main.LOGGER.log(Level.WARNING, "returning rt " + rt.get(0).dump() + rt.get(1).dump());

            return rt;
        }

        /********************************************************************************************************* */
        List<Mat> res = compute_distortion(K, cdist, this.SUBSAMPLE);
        /********************************************************************************************************* */

        Mat dpts = res.get(0);
        Mat pts = res.get(1);

        /********************************************************************************************************* */
        Rect bounds = Distortion.loc_from_dist(pts, dpts, this.mask, false, 1.); // ignore previously used masked off areas
        /********************************************************************************************************* */

        if(bounds == null)
        {
            Main.LOGGER.log(Level.SEVERE, "bounds is null, pose not contributing; guessing what to do");
            return get_pose(bbox, nk, 3, K, cdist); // best guess of what the author meant to do (drop the axis)
        }

        dpts.release();
        pts.release();

        Rect tgt_rect = new Rect(bounds.x*this.SUBSAMPLE, bounds.y*this.SUBSAMPLE, bounds.width*this.SUBSAMPLE, bounds.height*this.SUBSAMPLE);
        /********************************************************************************************************* */
        List<Object> rtb = pose_from_bounds(bbox, tgt_rect, K, cdist, this.img_size);
        /********************************************************************************************************* */
        Mat r = (Mat)rtb.get(0);
        Mat t = (Mat)rtb.get(1);
        Rect nbounds = (Rect)rtb.get(2);

        Main.LOGGER.log(Level.WARNING, "returning r " + r.dump());
        Main.LOGGER.log(Level.WARNING, "returning t " + t.dump());
        Main.LOGGER.log(Level.WARNING, "nbounds " + nbounds);
   
        nbounds.x = (int)Math.ceil((double)nbounds.x / this.SUBSAMPLE);
        nbounds.y = (int)Math.ceil((double)nbounds.y / this.SUBSAMPLE);
        nbounds.width = (int)Math.ceil((double)nbounds.width / this.SUBSAMPLE);
        nbounds.height = (int)Math.ceil((double)nbounds.height / this.SUBSAMPLE);

        // The np.ceil of the scalar x is the smallest integer i, such that i >= x
        // mask off y through y+h(-1) and x through x+w(-1)
        Mat.ones(nbounds.height, nbounds.width, this.mask.type())
            .copyTo(this.mask.submat(nbounds.y, nbounds.y+nbounds.height, nbounds.x, nbounds.x+nbounds.width));

        Main.LOGGER.log(Level.WARNING, "mask count non-zeros = " + Core.countNonZero(this.mask) + "\n" + brief(this.mask));

        List<Mat> rt = new ArrayList<>(2);
        rt.add(r);
        rt.add(t);

        return rt;
    }
}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     End PoseGenerator class                                                */
/*                                     End PoseGenerator class                                                */
/*                                     End PoseGenerator class                                                */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */

// Parking lot
      
    // http://euclideanspace.com/maths/geometry/rotations/axisAngle/index.htm

    /* https://stackoverflow.com/questions/12933284/rodrigues-into-eulerangles-and-vice-versa
Rodrigues parameters are also called axis-angle rotation. They are formed by 4 numbers
 [theta, x, y, z], which means that you have to rotate an angle "theta" around the axis described
  by unit vector v=[x, y, z]. Looking at cv::Rodrigues function reference, it seems that OpenCV
   uses a "compact" representation of Rodrigues notation as vector with 3 elements rod2=[a, b, c],
    where:
Angle to rotate theta is the module of input vector theta = sqrt(a^2 + b^2 + c^2)
Rotation axis v is the normalized input vector: v = rod2/theta = [a/theta, b/theta, c/theta]
     */
    // public static void tester()
    // {
    //     Mat angleVector = new Mat(3, 1, CvType.CV_64FC1);
    //     double[] EulerAnglesInput = {Math.PI + 0., 0.7853981633974483, 0.39269908169872414};

    //     {// WPILib rotation vector + OpenCV Rodrigues has same numbers but some different signs than the Python program
    //     // WPILib appears to be YZX order and the Python program is XYZ order
    //     Transform3d TR = new Transform3d(
    //         new Translation3d(),
    //         new Rotation3d(EulerAnglesInput[0], EulerAnglesInput[1], EulerAnglesInput[2]));
    //     System.out.println("WPILib rotation vector\n" + TR.getRotation().getQuaternion().toRotationVector());

    //     // this rotation matrix matches the Python program except some negative signs
    //     double[] angle = TR.getRotation().getQuaternion().toRotationVector().getData();
    //     angleVector.put(0, 0, angle);
    //     Mat R = new Mat();
    //     Calib3d.Rodrigues(angleVector, R);
    //     System.out.println("Rodrigues transpose\n" + R.t().dump()); // same output as the one in orbital_pose below except some negative signs!
    //     }

    //     { // matches the results of Python program and below orbital_pose method
    //     double[][] M = new double[3][3];
    //     EulerAnglesToRotationMatrix.calculate(
    //         EulerAnglesInput[0], EulerAnglesInput[1], EulerAnglesInput[2],
    //         EulerAnglesToRotationMatrix.EulerOrder.ORDER_XYZ,
    //         M );
    //     System.out.println("no Rodrigues XYZ\n");     
    //     for(int i = 0; i < M[0].length; i++)
    //     {
    //         System.out.print(i + "| ");
    //         for(int j = 0; j < M[1].length; j++)
    //             System.out.print(M[i][j] + " "); // if XYZ order, same output exactly as Java orbital_pose below

    //         System.out.println("|");
    //     }
    //     }
    // }

/*
CV_8U - 8-bit unsigned integers ( 0..255 )
CV_8S - 8-bit signed integers ( -128..127 )
CV_16U - 16-bit unsigned integers ( 0..65535 )
CV_16S - 16-bit signed integers ( -32768..32767 )
CV_32S - 32-bit signed integers ( -2147483648..2147483647 )
CV_32F - 32-bit floating-point numbers ( -FLT_MAX..FLT_MAX, INF, NAN )
CV_64F - 64-bit floating-point numbers ( -DBL_MAX..DBL_MAX, INF, NAN )
 */
/*
Defining the principal point
The principal point is the point on the image plane onto which the perspective center is projected. It is also the point from which the focal length of the lens is measured.

Near the principal point is the principal point of autocollimation (PPA). This is defined as the image position where the optical axis intersects the image plane.

The principal point of symmetry (POS), also known as the calibrated principal point, is the point on the image where a ray of light travelling perpendicular to the image plane passes through the focal point of the lens and intersects the film. In a perfectly assembled camera, the principal point of symmetry would be where the lines of opposing fiducial marks on an image intersect, also known as the indicated principal point (IPP). However, in most cameras a slight offset occurs. The perspective effects in the image are radial about this point.
 */

//  public void testBin()
//  {
//      int axis;
//      axis = 0;
//      for(int i = 0; i<20; i++)
//      {
//          Main.LOGGER.log(Level.WARNING, String.valueOf(gb[axis].gen_bin()));
//      }
//      axis = 1;
//      for(int i = 0; i<20; i++)
//      {
//          Main.LOGGER.log(Level.WARNING, String.valueOf(gb[axis].gen_bin()));
//      }
//  }