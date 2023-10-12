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
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                                                                                            */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */

public class PoseGeneratorDist {
    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}

/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                                                                                            */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    // generate values in range by binary subdivision
    // note that the list of values grows exponentially with each call for a value
    // seems like a poor algorithm to me but it is used sparsely in this application
    class Gen_Bin
    {
        double value;
        double s;
        double e;
        double t;
        public List<Double> lst = new ArrayList<Double>(40);
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                                                                                            */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
        Gen_Bin(double s, double e)
        {
            this.s = s;
            this.t = (s + e) / 2.;
            this.e = e;
            lst.add(s);
            lst.add(t);
            lst.add(t);
            lst.add(e);
        }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                                                                                            */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
        double next()
        {
            s = lst.get(0);
            e = lst.get(1);
            t = (s + e) / 2.;

            lst.remove(0);
            lst.remove(0);

            lst.add(s);
            lst.add(t);
            lst.add(t);
            lst.add(e);

            return t;
        }
    }
    // two angle bins - one for the x axis [0] and one for the y axis [1]
    Gen_Bin[] gb = {new Gen_Bin(Math.toRadians(-70.), Math.toRadians(70.)), // valid poses: r_x -> -70° .. 70°
                    new Gen_Bin(Math.toRadians(-70.), Math.toRadians(70.))}; // valid poses: r_y -> -70° .. 70°
    Function<Integer, double[]> orbital = (axis) ->
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

    int SUBSAMPLE = 20;
    Size img_size = new Size();
    // self.stats = [1, 1]  # number of (intrinsic, distortion) poses -- NOT USED?
    double orbitalZ = 1.6;
    double rz = Math.PI / 8.;

    Mat mask;
    double sgn = 1.;
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     PoseGeneratorDist Constructor                                          */
/*                                     PoseGeneratorDist Constructor                                          */
/*                                     PoseGeneratorDist Constructor                                          */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public PoseGeneratorDist(Size img_size)
    {
        Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

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
    public List<Mat> compute_distortion(Mat K, Mat cdist, int subsample)
    {
        Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

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
    public List<Mat> get_pose(Mat bbox, int nk, int tgt_param, Mat K, Mat cdist) throws Exception
    {
        Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        Main.LOGGER.log(Level.SEVERE, "bbox " + bbox + "\n" + bbox.dump());
        Main.LOGGER.log(Level.SEVERE, "nk " + nk);
        Main.LOGGER.log(Level.SEVERE, "tgt_param " + tgt_param);
        Main.LOGGER.log(Level.SEVERE, "camera matrix K " + K + "\n" + K.dump());
        Main.LOGGER.log(Level.SEVERE, "cdist " + cdist.dump());
 
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
            Main.LOGGER.log(Level.SEVERE, "angleIteration " + Arrays.toString(angleIteration));

            /********************************************************************************************************* */
            List<Mat> rt = orbital_pose(bbox, angleIteration[0], angleIteration[1], this.orbitalZ, this.rz);
            /********************************************************************************************************* */
            
            Main.LOGGER.log(Level.SEVERE, "rt " + rt.get(0).dump() + rt.get(1).dump());
            
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
//FIXME off3d and t apparently have different depths so check this and fix if possible. for now specify the depth to get past the error
                off3d.put(0, 2, 0.); // zero out the Z //FIXME 3 channel Mat trying to 0 the 3rd channel at 0,0 - not this way
                off3d.convertTo(off3d, CvType.CV_32SC1); // patch for error- different depths in add
                  Core.add(t, off3d, t); // pretty big offset being nearly the principal point
                this.sgn = -this.sgn;
                off.release();
                off3d.release();
            }

            rt.set(1, t); // update the nudged t and return

            Main.LOGGER.log(Level.SEVERE, "rt " + rt.get(0).dump() + rt.get(1).dump());

            return rt;
        }

        // dpts, pts = self.compute_distortion(K, cdist, self.SUBSAMPLE)

        /********************************************************************************************************* */
        List<Mat> res = compute_distortion(K, cdist, this.SUBSAMPLE);
        /********************************************************************************************************* */

        Mat dpts = res.get(0);
        Mat pts = res.get(1);
        // bounds = loc_from_dist(pts, dpts, mask=self.mask, boolean False, double 1.0) // ignore previously used masked off areas

        /********************************************************************************************************* */
        Rect bounds = Distortion.loc_from_dist(pts, dpts, this.mask, false, 1.); // ignore previously used masked off areas
        /********************************************************************************************************* */

        if(bounds == null)
        {
            Main.LOGGER.log(Level.SEVERE, "bounds is null, guessing what to do");
            return get_pose(bbox, nk, 3, K, cdist); // best guess of what the author meant to do
        }

    // r, t, nbounds = pose_from_bounds(bbox, bounds * self.SUBSAMPLE, K, cdist, self.img_size)
    // x, y, w, h = np.ceil(np.array(nbounds) / self.SUBSAMPLE).astype(int)
    // self.mask[y:y + h, x:x + w] = 1
    // return r, t
    
    // aabb><(63, 35, 1, 1)   x, y, w, h = cv2.boundingRect(points)
        Rect tgt_rect = new Rect(bounds.x*this.SUBSAMPLE, bounds.y*SUBSAMPLE, bounds.width*SUBSAMPLE, bounds.height*SUBSAMPLE);
        /********************************************************************************************************* */
        List<Object> rtb = pose_from_bounds(bbox, tgt_rect, K, cdist, this.img_size);
        /********************************************************************************************************* */
        Mat r = (Mat)rtb.get(0);
        Mat t = (Mat)rtb.get(1);
        Rect nbounds = (Rect)rtb.get(2);

        Main.LOGGER.log(Level.SEVERE, "returning r " + r.dump());
        Main.LOGGER.log(Level.SEVERE, "returning t " + t.dump());
        Main.LOGGER.log(Level.SEVERE, "nbounds " + nbounds + nbounds.toString());
        Main.LOGGER.log(Level.SEVERE, "mask\n" + brief(mask));

        nbounds.x = (int)Math.ceil((double)nbounds.x / this.SUBSAMPLE);
        nbounds.y = (int)Math.ceil((double)nbounds.y / this.SUBSAMPLE);
        nbounds.width = (int)Math.ceil((double)nbounds.width / this.SUBSAMPLE);
        nbounds.height = (int)Math.ceil((double)nbounds.height / this.SUBSAMPLE);

        // The np.ceil of the scalar x is the smallest integer i, such that i >= x
        // x, y, w, h = np.ceil(np.array(nbounds) / this.SUBSAMPLE).astype(int)

        // mask off y through y+h (-1) and x through x+w (-1)
        Mat.ones(nbounds.height, nbounds.width, this.mask.type())
            .copyTo(this.mask.submat(nbounds.y, nbounds.y+nbounds.height, nbounds.x, nbounds.x+nbounds.width));

        List<Mat> rt = new ArrayList<Mat>(2);
        rt.add(r);
        rt.add(t);

        return rt;
    }

    // def get_pose(self, bbox, nk, tgt_param, K, cdist):
    // """
    // @param bbox: bounding box of the calibration pattern
    // @param nk: number of keyframes captured so far
    // @param tgt_param: parameter that should be optimized by the pose
    // @param K, cdist: current calibration estimate
    // """
    // if nk == 0:
    //     # init sequence: first keyframe 45° tilted to camera
    //     return oribital_pose(bbox, 0, np.pi / 4, self.orbitalZ, np.pi / 8)
    // if nk == 1:
    //     # init sequence: second keyframe
    //     return pose_planar_fullscreen(K, cdist, self.img_size, bbox)
    // if tgt_param < 4:
    //     # orbital pose is used for focal length
    //     axis = (tgt_param + 1) % 2  # f_y -> r_x
        
    //     self.stats[0] += 1
    //     r, t = oribital_pose(bbox, *next(self.orbital[axis]))

    //     if tgt_param > 1:
    //         off = K[:2, 2].copy()
    //         off[tgt_param - 2] += self.img_size[tgt_param - 2] * 0.05 * self.sgn
    //         off3d = unproject(off, K, cdist, t[2])
    //         off3d[2] = 0
    //         t += off3d
    //         self.sgn *= -1

    //     return r, t

    // dpts, pts = self.compute_distortion(K, cdist, self.SUBSAMPLE)

    // bounds = loc_from_dist(pts, dpts, mask=self.mask)[0]
    
    // if bounds is None:
    //     # FIXME: anything else?
    //     print("loc_from_dist failed. return orbital pose instead of crashing")
    //     return self.get_pose(bbox, nk, 3, axis, K, cdist)
    
    // self.stats[1] += 1
    // r, t, nbounds = pose_from_bounds(bbox, bounds * self.SUBSAMPLE, K, cdist, self.img_size)
    // x, y, w, h = np.ceil(np.array(nbounds) / self.SUBSAMPLE).astype(int)
    // self.mask[y:y + h, x:x + w] = 1

    // return r, t

/*
    def unproject(p, K, cdist, Z):
    project pixel back to a 3D coordinate at depth Z
    print("unproject args", p, K, cdist, Z, sep="><")
    p = cv2.undistortPoints(p.reshape(-1, 1, 2), K, cdist).ravel()
    print("p", p)
    print("returns", np.array([p[0], p[1], 1]) * Z)
    return np.array([p[0], p[1], 1]) * Z

    unproject args>
    <[  0.         786.66666667]>
    <[[1.08640282e+03 0.00000000e+00 6.39500000e+02]
    [0.00000000e+00 1.08640282e+03 3.59500000e+02]
    [0.00000000e+00 0.00000000e+00 1.00000000e+00]]>
    <[[0. 0. 0. 0. 0.]]>
    <2138.8555557272484
    p [-0.58863986  0.39319363]
    returns [-1259.015625     840.984375    2138.85555573]
    ---OR--- for example
    unproject args><[  0.         786.66666667]><[[1.14215101e+03 0.00000000e+00 6.39500000e+02]
    [0.00000000e+00 1.14215101e+03 3.59500000e+02]
    [0.00000000e+00 0.00000000e+00 1.00000000e+00]]><[[0. 0. 0. 0. 0.]]><2248.6098077466895
    p [-0.55990845  0.37400192]
    returns [-1259.015625     840.984375    2248.60980775]
*/
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                                                                                            */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public static Mat unproject(MatOfPoint2f p, Mat K, Mat cdist, double Z)
    {
        Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // returns a translation (t) Mat
        Main.LOGGER.log(Level.SEVERE, "p in " + p.dump());
        Main.LOGGER.log(Level.SEVERE, "camera matrix K " + K + "\n" + K.dump());
        Main.LOGGER.log(Level.SEVERE, "cdist " + cdist.dump());
        Main.LOGGER.log(Level.SEVERE, "Z " + Z);

        // p = cv2.undistortPoints(p.reshape(-1, 1, 2), K, cdist).ravel()
        Calib3d.undistortPoints(p, p, K, cdist);
        Main.LOGGER.log(Level.SEVERE, "p out " + p.dump());

        // MatOfPoint3f p3D = new MatOfPoint3f(new Point3(p.get(0, 0)[0], p.get(0, 0)[1], 1.));
        Mat p3D = new Mat(1, 3, CvType.CV_32FC1);
        p3D.put(0, 0, p.get(0, 0)[0], p.get(0, 0)[1], 1.);
        Core.multiply(p3D, new Scalar(Z, Z, Z), p3D);
        Main.LOGGER.log(Level.SEVERE, "return p3D Z scaled " + p3D.dump());

        return p3D;
        
        // print("p", p)
        // print("returns", np.array([p[0], p[1], 1]) * Z)
        // return np.array([p[0], p[1], 1]) * Z
        /*
        p in [0, 786.66669]
        K [1086.4028, 0, 639.5;
        0, 1086.4028, 359.5;
        0, 0, 1]
        cdist [0, 0, 0, 0, 0]
        Z 2138.8555557272484
        p out [-0.58863986, 0.39319363]
        p3D Z scaled [-1259.0156, 840.98438, 2138.8555]
        */
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
    // orbital_pose><[2520. 1680. 2520.]><0><0.7853981633974483><<1.6>"""
    //FIXME original misspelled this orbital
    public static List<Mat> orbital_pose(Mat bbox, double rx, double ry, double Z, double rz) // force caller to use rz=0 if defaulting
    {
/*
orbital_pose
bbox (3,) [2520.0000 1680.0000 2520.0000]
rx () 0
ry () 0.7853981633974483
Z 1.6
rz () 0.39269908169872414
Rx [[1.0000 0.0000 0.0000]
 [0.0000 -1.0000 -0.0000]
 [0.0000 0.0000 -1.0000]]
Ry [[0.7071 0.0000 0.7071]
 [0.0000 1.0000 0.0000]
 [-0.7071 0.0000 0.7071]]
Rz [[0.9239 -0.3827 0.0000]
 [0.3827 0.9239 0.0000]
 [0.0000 0.0000 1.0000]]
bbox><[2520.0000 1680.0000 2520.0000]><[-0.5, -0.5, 0]><[-1260.0000 -840.0000 0.0000]
Tc><[[1.0000 0.0000 0.0000 0.0000]
 [0.0000 1.0000 0.0000 0.0000]
 [0.0000 0.0000 1.0000 0.0000]
 [-595.8323 1258.2399 595.8323 1.0000]]><(3,)
T [[1.0000 0.0000 0.0000 0.0000]
 [0.0000 1.0000 0.0000 0.0000]
 [0.0000 0.0000 1.0000 0.0000]
 [-1260.0000 -840.0000 4032.0000 1.0000]]
Rf [[0.6533 -0.2706 -0.7071 0.0000]
 [-0.3827 -0.9239 -0.0000 0.0000]
 [-0.6533 0.2706 -0.7071 0.0000]
 [-595.8323 1258.2399 4627.8323 1.0000]]
Rod Rf r returned [2.7188 -0.5408 -1.1262]
Rod Rf t returned><[-595.8323 1258.2399 4627.8323]
end of orbital_pose function
*/
        Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        Main.LOGGER.log(Level.SEVERE, "bbox " + bbox + "\n" + bbox.dump());
        Main.LOGGER.log(Level.SEVERE, "rx " + rx);
        Main.LOGGER.log(Level.SEVERE, "ry " + ry);
        Main.LOGGER.log(Level.SEVERE, "Z " + Z);
        Main.LOGGER.log(Level.SEVERE, "rz " + rz);
        
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

        // Main.LOGGER.log(Level.FINE, "Rz\n" + Rz.dump());
        // Main.LOGGER.log(Level.FINE, "Rx\n" + Rx.dump());
        // Main.LOGGER.log(Level.FINE, "Ry\n" + Ry.dump());

        // in Python (Ry).dot(Rx).dot(Rz) messed up nomenclature - it's often really matrix multiply Ry times Rx times Rz
        Mat R = Mat.eye(4, 4, CvType.CV_64FC1);
        Mat R3x3 = R.submat(0, 3, 0, 3);

        /********************************************************************************************************* */
        Core.gemm(Ry, Rx, 1., new Mat(), 0, R3x3);
        Core.gemm(R3x3, Rz, 1., new Mat(), 0., R3x3); // rotation matrix of the input Euler Angles
        /********************************************************************************************************* */
        // translate board to its center
        Mat Tc = Mat.eye(4, 4, CvType.CV_64FC1);
        Mat Tc1x3 = Tc.submat(3, 4, 0, 3);
        Mat Tc3x1 = new Mat(); // temp for Rodrigues output
        Mat translateToBoardCenter = new Mat(bbox.rows(), bbox.cols(), bbox.type()); // matching bbox for element by element multiply
        translateToBoardCenter.put(0, 0, -0.5, -0.5, 0.);
        translateToBoardCenter = bbox.mul(translateToBoardCenter);
        Main.LOGGER.log(Level.SEVERE, "translateToBoardCenter\n" + translateToBoardCenter.dump());

        // Main.LOGGER.log(Level.SEVERE, "R " + R.dump());
        // Main.LOGGER.log(Level.SEVERE, "R3x3 " + R3x3.dump());
        // Main.LOGGER.log(Level.SEVERE, "Tc " + Tc.dump());
        // Main.LOGGER.log(Level.SEVERE, "Tc1x3 " + Tc1x3.dump());

        /********************************************************************************************************* */
        Core.gemm(R3x3, translateToBoardCenter, 1., new Mat(), 0.,Tc3x1);
        Tc3x1.t().copyTo(Tc1x3); // update Tc
        /********************************************************************************************************* */
        // Main.LOGGER.log(Level.SEVERE, "Tc " + Tc.dump());
        // Main.LOGGER.log(Level.SEVERE, "Tc1x3 " + Tc1x3.dump());

    // # translate board to center of image
    // T = np.eye(4)
    // T[3, :3] = bbox * [-0.5, -0.5, Z]

        Mat T = Mat.eye(4, 4, CvType.CV_64FC1);
        Mat T1x3 = T.submat(3, 4, 0, 3);     
        /********************************************************************************************************* */
        Mat translateToImageCenter = new Mat(bbox.rows(), bbox.cols(), bbox.type()); // matching bbox for element by element multiply
        translateToImageCenter.put(0, 0, -0.5, -0.5, Z);
        bbox.mul(translateToImageCenter).t().copyTo(T1x3);   
        /********************************************************************************************************* */
        // Main.LOGGER.log(Level.SEVERE, "translateToImageCenter " + translateToImageCenter.dump());
        // Main.LOGGER.log(Level.SEVERE, "T1x3 " + T1x3.dump());
        // Main.LOGGER.log(Level.SEVERE, "T " + T.dump());
    
    // rotate center of board
        Mat Rf = new Mat();
        
        /********************************************************************************************************* */
        Core.gemm(Tc.inv(), R, 1., new Mat(), 0.,Rf);
        Core.gemm(Rf, Tc, 1., new Mat(), 0.,Rf);
        Core.gemm(Rf, T, 1., new Mat(), 0.,Rf);
        /********************************************************************************************************* */
        // Main.LOGGER.log(Level.SEVERE, "Rf " + Rf.dump());

        // return cv2.Rodrigues(Rf[:3, :3])[0].ravel(), Rf[3, :3]
        Mat Rf3x3 = Rf.submat(0, 3, 0, 3);
        Mat RfVector = new Mat(1, 3, CvType.CV_64FC1);

        /********************************************************************************************************* */
        Calib3d.Rodrigues(Rf3x3,RfVector);
        Core.transpose(RfVector, RfVector);
        /********************************************************************************************************* */
        Main.LOGGER.log(Level.SEVERE, "RfVector returned " + RfVector.dump());

        Mat t = Rf.submat(3, 4, 0, 3);
        Mat tVector = new Mat();
        t.copyTo(tVector);
        Main.LOGGER.log(Level.SEVERE, "tVector returned " + tVector.dump());

        List<Mat> rt = new ArrayList<Mat>(2);
        rt.add(RfVector);
        rt.add(tVector);

        return rt;
    }
// Numpy Dot product of two arrays. Specifically,
// If both a and b are 1-D arrays, it is inner product of vectors (without complex conjugation).
// If both a and b are 2-D arrays, it is matrix multiplication, but using matmul or a @ b is preferred.
// If either a or b is 0-D (scalar), it is equivalent to multiply and using numpy.multiply(a, b) or a * b is preferred.
// If a is an N-D array and b is a 1-D array, it is a sum product over the last axis of a and b.
// If a is an N-D array and b is an M-D array (where M>=2), it is a sum product over the last axis of a and the second-to-last axis of b:
    // def oribital_pose(bbox, rx, ry, Z, rz=0):
    // Rz = cv2.Rodrigues(np.array([0., 0., rz]))[0] # rotation vector to rotation matrix
    // Rx = cv2.Rodrigues(np.array([np.pi + rx, 0., 0.]))[0]  # flip by 180° so Z is up
    // Ry = cv2.Rodrigues(np.array([0., ry, 0.]))[0]

    // R = np.eye(4)
    // R[:3, :3] = (Ry).dot(Rx).dot(Rz) # matrix multiply (not DOT) if it's numpy but not sure - looks like OpenCV mat.dot(mat)

    // Tc = np.eye(4)
    // Tc[3, :3] = R[:3, :3].dot(bbox * [-0.5, -0.5, 0])

    // # translate board to center of image
    // T = np.eye(4)
    // T[3, :3] = bbox * [-0.5, -0.5, Z]

    // # rotate center of board
    // Rf = la.inv(Tc).dot(R).dot(Tc).dot(T)

    // return cv2.Rodrigues(Rf[:3, :3])[0].ravel(), Rf[3, :3]
/*
java:
2023-10-04 11:46:59.305 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] method entered  . . . . . . . . . . . . . . . . . . . . . . . . 
2023-10-04 11:46:59.307 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] bbox [2520;
 1680;
 2520] 
2023-10-04 11:46:59.309 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] rx 0.0 
2023-10-04 11:46:59.310 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] ry 0.7853981633974483 
2023-10-04 11:46:59.312 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] Z 1.6 
2023-10-04 11:46:59.313 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] rz 0.39269908169872414 
2023-10-04 11:46:59.316 FINE    [ calibrator.PoseGeneratorDist oribital_pose] Rz
[0.9238795325112867, -0.3826834323650898, 0;
 0.3826834323650898, 0.9238795325112867, 0;
 0, 0, 1] 
2023-10-04 11:46:59.319 FINE    [ calibrator.PoseGeneratorDist oribital_pose] Rx
[1, 0, 0;
 0, -1, -1.224646799147353e-16;
 0, 1.224646799147353e-16, -1] 
2023-10-04 11:46:59.325 FINE    [ calibrator.PoseGeneratorDist oribital_pose] Ry
[0.7071067811865476, 0, 0.7071067811865476;
 0, 1, 0;
 -0.7071067811865476, 0, 0.7071067811865476] 
2023-10-04 11:46:59.328 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] translateTOBoardCenter [-1260;
 -840;
 0] 
2023-10-04 11:46:59.330 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] R [0.6532814824381883, -0.2705980500730985, -0.7071067811865476, 0;
 -0.3826834323650898, -0.9238795325112867, -1.224646799147353e-16, 0;
 -0.6532814824381883, 0.2705980500730986, -0.7071067811865476, 0;
 0, 0, 0, 1] 
2023-10-04 11:46:59.332 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] R3x3 [0.6532814824381883, -0.2705980500730985, -0.7071067811865476;
 -0.3826834323650898, -0.9238795325112867, -1.224646799147353e-16;
 -0.6532814824381883, 0.2705980500730986, -0.7071067811865476] 
2023-10-04 11:46:59.336 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] Tc [1, 0, 0, 0;
 0, 1, 0, 0;
 0, 0, 1, 0;
 0, 0, 0, 1] 
2023-10-04 11:46:59.338 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] Tc1x3 [0, 0, 0] 
2023-10-04 11:46:59.340 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] Tc [1, 0, 0, 0;
 0, 1, 0, 0;
 0, 0, 1, 0;
 -595.8323058107145, 1258.239932089494, 595.8323058107144, 1] 
2023-10-04 11:46:59.342 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] Tc1x3 [-595.8323058107145, 1258.239932089494, 595.8323058107144] 
2023-10-04 11:46:59.345 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] translateToImageCenter [-0.5;
 -0.5;
 1.6] 
2023-10-04 11:46:59.348 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] T1x3 [-1260, -840, 4032] 
2023-10-04 11:46:59.351 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose] T [1, 0, 0, 0;
 0, 1, 0, 0;
 0, 0, 1, 0;
 -1260, -840, 4032, 1] 
2023-10-04 11:46:59.354 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose]
 Rf [0.6532814824381883, -0.2705980500730985, -0.7071067811865476, 0;
 -0.3826834323650898, -0.9238795325112867, -1.224646799147353e-16, 0;
 -0.6532814824381883, 0.2705980500730986, -0.7071067811865476, 0;
 -595.8323058107145, 1258.239932089494, 4627.832305810714, 1] 
2023-10-04 12:55:06.145 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose]
RfVector [2.718846028485863, -0.5408121000669039, -1.12618289900307] 
2023-10-04 12:55:06.146 SEVERE  [ calibrator.PoseGeneratorDist oribital_pose]
 t [-595.8323058107145, 1258.239932089494, 4627.832305810714] 

python:
orbital_pose
Rx [[1.0000 0.0000 0.0000]
 [0.0000 -1.0000 -0.0000]
 [0.0000 0.0000 -1.0000]]
Ry [[0.7071 0.0000 0.7071]
 [0.0000 1.0000 0.0000]
 [-0.7071 0.0000 0.7071]]
Rz [[0.9239 -0.3827 0.0000]
 [0.3827 0.9239 0.0000]
 [0.0000 0.0000 1.0000]]
bbox><[2520.0000 1680.0000 2520.0000]><[-0.5, -0.5, 0]><[-1260.0000 -840.0000 0.0000]
Tc><[[1.0000 0.0000 0.0000 0.0000]
 [0.0000 1.0000 0.0000 0.0000]
 [0.0000 0.0000 1.0000 0.0000]
 matches ok [-595.8323 1258.2399 595.8323 1.0000]]><(3,)
T [[1.0000 0.0000 0.0000 0.0000]
 [0.0000 1.0000 0.0000 0.0000]
 [0.0000 0.0000 1.0000 0.0000]
 [-1260.0000 -840.0000 4032.0000 1.0000]]
matches ok Rf [[0.6533 -0.2706 -0.7071 0.0000]
 [-0.3827 -0.9239 -0.0000 0.0000]
 [-0.6533 0.2706 -0.7071 0.0000]
 [-595.8323 1258.2399 4627.8323 1.0000]]
Rod Rf><[2.7188 -0.5408 -1.1262]><[-595.8323 1258.2399 4627.8323]
end of orbital_pose function
*/
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     pose_planar_fullscreen                                                 */
/*                                     pose_planar_fullscreen                                                 */
/*                                     pose_planar_fullscreen                                                 */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public static List<Mat> pose_planar_fullscreen(Mat K, Mat cdist, Size img_size, Mat bbox)
    {
        Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // don't use the principal point throughout just have X and Y no Z until it's calculated in the middle
        // compute a new Z
        Main.LOGGER.log(Level.SEVERE, "camera matrix K " + K + "\n" + K.dump());
        Main.LOGGER.log(Level.SEVERE, "cdist " + cdist.dump());
        Main.LOGGER.log(Level.SEVERE, "img_size " + img_size.toString());
        Main.LOGGER.log(Level.SEVERE, "bbox " + bbox + "\n" + bbox.dump());
// K [1120.16, 0, 639.5;
//  0, 1120.16, 359.5;
//  0, 0, 1]
// cdist [0, 0, 0, 0, 0]
// img_size 1280x720
// bbox [2520, 1680, 2520]
        Mat KB = new Mat(); // ignore principal point

        Core.gemm(
            K.submat(0, 2, 0, 2), // K without PP
             bbox.submat(0, 2, 0, 1), // bbox without Z
             1., new Mat(), 0., KB);

        double KBx = KB.get(0, 0)[0];
        double KBy = KB.get(1, 0)[0];
        double Z = Math.min(KBx/img_size.width, KBy/img_size.height);
        double[] pB = {KBx/Z, KBy/Z};

        Mat r = new Mat(1, 3, CvType.CV_64FC1); //FIXME will 1x3 work okay?
        r.put(0, 0, Math.PI, 0., 0.);

        MatOfPoint2f p = new MatOfPoint2f(new Point(img_size.width/2. - pB[0]/2., img_size.height/2. + pB[1]/2.));
        Mat t = unproject(p, K, cdist, Z);

        Main.LOGGER.log(Level.FINE, "KBnoPrinciplePoint " + KB + KB.dump());
        Main.LOGGER.log(Level.FINE, "Z, pB x, y " + Z + ", " + Arrays.toString(pB));
        Main.LOGGER.log(Level.FINE, "p " + p + p.dump());
        Main.LOGGER.log(Level.FINE, "returning r " + r + r.dump());
        Main.LOGGER.log(Level.FINE, "returning t " + t + t.dump());

        // KB = K.dot([bbox[0], bbox[1], 0])  # ignore principal point
        // Z = (KB[0:2] / img_size).min()
        // pB = KB / Z
    
        // r = np.array([np.pi, 0, 0])  # flip image
        // # move board to center, org = bl
        // p = np.array([img_size[0] / 2 - pB[0] / 2, img_size[1] / 2 + pB[1] / 2])
        // t = unproject(p, K, cdist, Z)
        // return r, t

        KB.release();
        p.release();

        List<Mat> rt = new ArrayList<>(2);
        rt.add(r);
        rt.add(t);

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
    public static List<Object> pose_from_bounds(Mat src_extParm, Rect tgt_rect, Mat K, Mat cdist, Size img_sz)
    {
        Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        Main.LOGGER.log(Level.SEVERE, "src_extParm " + src_extParm + src_extParm.dump());
        Main.LOGGER.log(Level.SEVERE, "tgt_rect " + tgt_rect.toString()); //tgt 0,40,20,40
        Main.LOGGER.log(Level.SEVERE, "camera matrix K " + K + "\n" + K.dump());
        Main.LOGGER.log(Level.SEVERE, "cdist " + cdist.dump());
        Main.LOGGER.log(Level.SEVERE, "img_sz " + img_sz.toString());
        // pose_from_bounds
        // [2520. 1680. 2520.]> src_ext
        // <[1260    0   20   20]> tgt_rect x y w h
        // <[[1.19595149e+03 0.00000000e+00 6.12804738e+02] k
        //  [0.00000000e+00 1.18941114e+03 3.70394348e+02] k
        //  [0.00000000e+00 0.00000000e+00 1.00000000e+00]]> k
        //  <[[ 1.02232239e-01 -7.33276512e-01 -9.25042234e-05  3.04543199e-04 cdist
        //    8.93575471e-01]]> cdist
        //    <(1280, 720) img_sz
        double[] src_ext = new double[(int)src_extParm.total()]; // maybe pass in int or double
        src_extParm.get(0, 0, src_ext);

        boolean rot90 = tgt_rect.height > tgt_rect.width;
        int MIN_WIDTH = (int)Math.floor(img_sz.width/3.333); // guidance board must be about a third or more of the image size

        Main.LOGGER.log(Level.FINE, "rot90 " + rot90);

        if(rot90)
        {
            // flip x and y for the rotation
            src_ext = src_ext.clone(); // not needed in this implementation; copy hidden from method caller need only if passing not a Mat
            double swapXY = src_ext[0];
            src_ext[0] = src_ext[1];
            src_ext[1] = swapXY;

            if (tgt_rect.height < MIN_WIDTH)
            {
                // double scale = MIN_WIDTH / tgt_rect.width; //FIXME was this wrong? tgt_rect.width => tgt_rect.height?
                double scale = MIN_WIDTH / tgt_rect.height; //FIXME was this wrong? tgt_rect.width => tgt_rect.height?
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

        double aspect = src_ext[0] / src_ext[1];

        // match aspect ratio of tgt to src, but keep tl
        if (!rot90)
        {
            tgt_rect.height = (int)(tgt_rect.width / aspect); // adapt height
        }
        else
        {
            tgt_rect.width = (int)(tgt_rect.height * aspect); // adapt width
        }
        
        Mat r = new Mat(1, 3, CvType.CV_64FC1);
        r.put(0, 0, Math.PI, 0., 0.);
        
        // org is bl (bottom left?)
        if (rot90)
        {
            Mat R = new Mat();

            Calib3d.Rodrigues(r, R);

            Main.LOGGER.log(Level.SEVERE, "R " + R.dump());
            Mat rz = new Mat(1, 3, CvType.CV_64FC1);
            rz.put(0, 0, 0., 0., -Math.PI/2.);
            Mat Rz = new Mat();

            Calib3d.Rodrigues(rz, Rz);

            Main.LOGGER.log(Level.SEVERE, "Rz " + Rz.dump());

            Core.gemm(R, Rz, 1., new Mat(), 0., R); // rotation matrix of the input Euler Angles

            Calib3d.Rodrigues(R, r);

            r = r.t(); // Rodrigues out is 3x1 and rest of program is 1x3
            // org is tl (top left?)
        }

        double Z = (K.get(0, 0)[0] * src_ext[0]) / tgt_rect.width;

        //  clip to image region
        int[] min_off = {0, 0};
        int[] max_off = {(int)(img_sz.width - tgt_rect.width), (int)(img_sz.height - tgt_rect.height)};
        tgt_rect.x = Math.min(max_off[0], Math.max(tgt_rect.x, min_off[0]));
        tgt_rect.y = Math.min(max_off[1], Math.max(tgt_rect.y, min_off[1]));
    
        if (!rot90)
        {
            tgt_rect.y += tgt_rect.height;            
        }
    
        MatOfPoint2f p = new MatOfPoint2f(new Point(tgt_rect.x, tgt_rect.y));

        Mat t = unproject(p, K, cdist, Z);

        if (!rot90)
        {
            tgt_rect.y -= tgt_rect.height;
        }

        Main.LOGGER.log(Level.SEVERE, "returning r " + r.dump());
        Main.LOGGER.log(Level.SEVERE, "returning t " + t.dump());
        Main.LOGGER.log(Level.SEVERE, "returning tgt_rect " + tgt_rect.toString());

        List<Object> rtb = new ArrayList<>(3);
        rtb.add(r); // position 0
        rtb.add(t); // position 1
        rtb.add(tgt_rect); // position 2

        return rtb;
    }
    // def pose_from_bounds(src_ext, tgt_rect, K, cdist, img_sz):
    // rot90 = tgt_rect[3] > tgt_rect[2]

    // MIN_WIDTH = img_sz[0] // 3.333

    // if rot90:
    //     src_ext = src_ext.copy()
    //     src_ext[0], src_ext[1] = src_ext[1], src_ext[0]

    //     if tgt_rect[3] < MIN_WIDTH:
    //         scale = MIN_WIDTH / tgt_rect[2]
    //         tgt_rect[3] = MIN_WIDTH
    //         tgt_rect[2] *= scale
    // else:
    //     if tgt_rect[2] < MIN_WIDTH:
    //         scale = MIN_WIDTH / tgt_rect[2]
    //         tgt_rect[2] = MIN_WIDTH
    //         tgt_rect[3] *= scale

    // aspect = src_ext[0] / src_ext[1]

    // # match aspect ratio of tgt to src, but keep tl
    // if not rot90:
    //     # adapt height
    //     tgt_rect[3] = tgt_rect[2] / aspect
    // else:
    //     # adapt width
    //     tgt_rect[2] = tgt_rect[3] * aspect

    // r = np.array([np.pi, 0, 0])

    // # org is bl
    // if rot90:
    //     R = cv2.Rodrigues(r)[0]
    //     Rz = cv2.Rodrigues(np.array([0., 0., -np.pi / 2]))[0]
    //     R = R.dot(Rz)
    //     r = cv2.Rodrigues(R)[0].ravel()
    //     # org is tl

    // Z = (K[0, 0] * src_ext[0]) / tgt_rect[2]

    // # clip to image region
    // max_off = img_sz - tgt_rect[2:4]
    // tgt_rect[0:2] = tgt_rect[0:2].clip([0, 0], max_off)

    // if not rot90:
    //     tgt_rect[1] += tgt_rect[3]

    // t = unproject(np.array([tgt_rect[0], tgt_rect[1]], dtype=np.float32), K, cdist, Z)

    // if not rot90:
    //     tgt_rect[1] -= tgt_rect[3]

    // return r, t, tgt_rect

}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                                                                                            */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*
pose_from_bounds 
src [2520;
 1680;    
 2520]
tgt [1260;
 0;
 20;
 20]
K [1195.95149, 0, 612.804738;
 0, 1189.41114, 370.394348;
 0, 0, 1]
cdist [0.102232239, -0.733276512, -9.25042234e-05, 0.000304543199, 0.893575471]
img_sz 1280x720
rot90 false
r [3.141592653589793;
 0;
 0]
unproject
p in [896, 256]
K [1195.95149, 0, 612.804738;
 0, 1189.41114, 370.394348;
 0, 0, 1]
cdist [0.102232239, -0.733276512, -9.25042234e-05, 0.000304543199, 0.893575471]
Z 7848.431653125
p out [0.23584361, -0.095792912]
p3D Z scaled [1851.0024, -751.8241, 7848.4316]
[3.141592653589793;
 0;
 0]
[1851.0024, -751.8241, 7848.4316]
[896;
 0;
 384;
 256]

     */
    // def pose_from_bounds(src_ext, tgt_rect, K, cdist, img_sz):
    // print("pose_from_bounds")
    // print(src_ext, tgt_rect, K, cdist, img_sz, sep="><")
    // rot90 = tgt_rect[3] > tgt_rect[2]
    // print("rot90", rot90)
    // MIN_WIDTH = img_sz[0] // 3.333
    // print("min width", MIN_WIDTH)
    // if rot90:
    //     print("id src_ext", id(src_ext))
    //     src_ext = src_ext.copy()
    //     src_ext[0], src_ext[1] = src_ext[1], src_ext[0]
    //     print("rot90 src_ext", id(src_ext), src_ext)
    //     if tgt_rect[3] < MIN_WIDTH:
    //         scale = MIN_WIDTH / tgt_rect[2]
    //         tgt_rect[3] = MIN_WIDTH
    //         tgt_rect[2] *= scale
    // else:
    //     if tgt_rect[2] < MIN_WIDTH:
    //         scale = MIN_WIDTH / tgt_rect[2]
    //         tgt_rect[2] = MIN_WIDTH
    //         tgt_rect[3] *= scale

    // aspect = src_ext[0] / src_ext[1]

    // # match aspect ratio of tgt to src, but keep tl
    // if not rot90:
    //     # adapt height
    //     tgt_rect[3] = tgt_rect[2] / aspect
    // else:
    //     # adapt width
    //     tgt_rect[2] = tgt_rect[3] * aspect

    // r = np.array([np.pi, 0, 0])

    // # org is bl
    // if rot90:
    //     R = cv2.Rodrigues(r)[0]
    //     print("R", R)
    //     Rz = cv2.Rodrigues(np.array([0., 0., -np.pi / 2]))[0]
    //     print("Rz", Rz)
    //     R = R.dot(Rz)
    //     print("R", R)
    //     r = cv2.Rodrigues(R)[0].ravel()
    //     # org is tl

    // Z = (K[0, 0] * src_ext[0]) / tgt_rect[2]

    // # clip to image region
    // max_off = img_sz - tgt_rect[2:4]
    // tgt_rect[0:2] = tgt_rect[0:2].clip([0, 0], max_off)

    // if not rot90:
    //     tgt_rect[1] += tgt_rect[3]

    // t = unproject(np.array([tgt_rect[0], tgt_rect[1]], dtype=np.float32), K, cdist, Z)

    // if not rot90:
    //     tgt_rect[1] -= tgt_rect[3]

    // print("r t tgt_rect", r, t, tgt_rect, "end pose_from_bounds", sep="><")

    // return r, t, tgt_rect
// set_next_pose 2
// get_pose
// compute_distortion
// pose_from_bounds
// [2520. 1680. 2520.]>
// <[1260    0   20   20]>
// <[[1.19595149e+03 0.00000000e+00 6.12804738e+02]
//  [0.00000000e+00 1.18941114e+03 3.70394348e+02]
//  [0.00000000e+00 0.00000000e+00 1.00000000e+00]]>
//  <[[ 1.02232239e-01 -7.33276512e-01 -9.25042234e-05  3.04543199e-04
//    8.93575471e-01]]>
//    <(1280, 720)
// rot90 False
// min width 384.0
// r t tgt_rect>
// <[3.14159265 0.         0.        ]>
// <[1851.00247682 -751.82411964 7848.43162565]>
// <[896   0 384 256]>
// <end pose_from_bounds

/*
pose_planar_fullscreen
K [1120.16, 0, 639.5;
 0, 1120.16, 359.5;
 0, 0, 1]
cdist [0, 0, 0, 0, 0]
img_size 1280x720
bbox [2520;
 1680;
 2520]
unproject
p in [0, 786.66669]
K [1120.16, 0, 639.5;
 0, 1120.16, 359.5;
 0, 0, 1]
cdist [0, 0, 0, 0, 0]
Z 2205.3150390625
p out [-0.57090056, 0.38134435]
p3D Z scaled [-1259.0156, 840.98444, 2205.3149]
KBnoPrinciplePoint [2822803.3;
 1881868.9] Mat [ 2*1*CV_32FC1, isCont=true, isSubmat=false, nativeObj=0x23fc99aa4e0, dataAddr=0x23fc998d300 ]
Z pB 2205.3150390625 1280.0 853.3333522270814
p [-0.57090056, 0.38134435]
r [3.1415927;
 0;
 0]
t [-1259.0156, 840.98444, 2205.3149]
 */
/*    
    def pose_planar_fullscreen(K, cdist, img_size, bbox):
    print("pose_planar_fullscreen")
    print(K, cdist, img_size, bbox, sep="><")
    
    KB = K.dot([bbox[0], bbox[1], 0])  # ignore principal point
    Z = (KB[0:2] / img_size).min()
    pB = KB / Z
    print("full screen", KB, Z, pB, sep="><")

    r = np.array([np.pi, 0, 0])  # flip image
    # move board to center, org = bl
    p = np.array([img_size[0] / 2 - pB[0] / 2, img_size[1] / 2 + pB[1] / 2])
    t = unproject(p, K, cdist, Z)
    print("p", p)
    print("r t", r, t, "return",sep="><")

    return r, t
"""
set_next_pose 1
get_pose
pose_planar_fullscreen
K [[1.12016001e+03 0.00000000e+00 6.39500000e+02]
 [0.00000000e+00 1.12016001e+03 3.59500000e+02]
 [0.00000000e+00 0.00000000e+00 1.00000000e+00]]>
 cdist <[[0. 0. 0. 0. 0.]]>
 img_size <(1280, 720)>
 bbox <[2520. 1680. 2520.]
full screen>
KB <[2822803.23270081 1881868.82180054       0.        ]>
Z <2205.315025547511>
pB <[1280.          853.33333333    0.        ]
p [  0.         786.66666667]
r t>
r <[3.14159265 0.         0.        ]>
t <[-1259.015625     840.984375    2205.31502555]>
<return
"""
*/

/*
    Python results - goal
set_next_pose 0
bbox, rx, ry, rz, Z><[2520. 1680. 2520.]><0><0.7853981633974483><0.39269908169872414><1.6
Rx [[ 1.0000000e+00  0.0000000e+00  0.0000000e+00]
 [ 0.0000000e+00 -1.0000000e+00 -1.2246468e-16]
 [ 0.0000000e+00  1.2246468e-16 -1.0000000e+00]]
Ry [[ 0.70710678  0.          0.70710678]
 [ 0.          1.          0.        ]
 [-0.70710678  0.          0.70710678]]
Rz [[ 0.92387953 -0.38268343  0.        ]
 [ 0.38268343  0.92387953  0.        ]
 [ 0.          0.          1.        ]]
<class 'numpy.ndarray'> (3, 3)
R3 [[ 6.53281482e-01 -2.70598050e-01 -7.07106781e-01]
 [-3.82683432e-01 -9.23879533e-01 -1.22464680e-16]
 [-6.53281482e-01  2.70598050e-01 -7.07106781e-01]]
bbox><(3,)><(3,)
bbox><[2520. 1680. 2520.]><[-0.5, -0.5, 0]><[-1260.  -840.     0.]
Tc><[[ 1.00000000e+00  0.00000000e+00  0.00000000e+00  0.00000000e+00]
 [ 0.00000000e+00  1.00000000e+00  0.00000000e+00  0.00000000e+00]
 [ 0.00000000e+00  0.00000000e+00  1.00000000e+00  0.00000000e+00]
 [-5.95832306e+02  1.25823993e+03  5.95832306e+02  1.00000000e+00]]><(3,)
T [[ 1.000e+00  0.000e+00  0.000e+00  0.000e+00]
 [ 0.000e+00  1.000e+00  0.000e+00  0.000e+00]
 [ 0.000e+00  0.000e+00  1.000e+00  0.000e+00]
 [-1.260e+03 -8.400e+02  4.032e+03  1.000e+00]]
Rf [[ 6.53281482e-01 -2.70598050e-01 -7.07106781e-01  0.00000000e+00]
 [-3.82683432e-01 -9.23879533e-01 -1.22464680e-16  0.00000000e+00]
 [-6.53281482e-01  2.70598050e-01 -7.07106781e-01  0.00000000e+00]
 [-5.95832306e+02  1.25823993e+03  4.62783231e+03  1.00000000e+00]]
Rod Rf><[ 2.71884603 -0.5408121  -1.1261829 ]><[-595.83230581 1258.23993209 4627.83230581]
end of orbital_pose function
[ 2.71884603 -0.5408121  -1.1261829 ]
board warped 2764800 (720, 1280, 3)
*/
    // # translation defined in terms of board dimensions
    //     self.board_units = np.array([tracker.board_sz[0], tracker.board_sz[1], tracker.board_sz[0]]) * self.square_len
    //     x, y, x  (z set to same as x)
    //     board_x: 9
    //     board_y: 6
    //     square_len: 280
    //     9*280=2520
    //     6*280=1680


    // def oribital_pose(bbox, rx, ry, Z, rz=0):
    // """
    // orbital_pose><[2520. 1680. 2520.]><0><0.7853981633974483><1.6><0.39269908169872414"""
    // @param bbox: object bounding box. note: assumes planar object with virtual Z dimension. 
    // @param rx: rotation around x axis in rad
    // @param ry: rotation around y axis in rad
    // @param Z: distance to camera in board lengths
    // @return: rvec, tvec 
    // """
    // Rz = cv2.Rodrigues(np.array([0., 0., rz]))[0]
    // Rx = cv2.Rodrigues(np.array([np.pi + rx, 0., 0.]))[0]  # flip by 180° so Z is up
    // Ry = cv2.Rodrigues(np.array([0., ry, 0.]))[0]

    // R = np.eye(4)
    // R[:3, :3] = (Ry).dot(Rx).dot(Rz)

    // # translate board to its center
    // Tc = np.eye(4)
    // Tc[3, :3] = R[:3, :3].dot(bbox * [-0.5, -0.5, 0])

    // # translate board to center of image
    // T = np.eye(4)
    // T[3, :3] = bbox * [-0.5, -0.5, Z]

    // # rotate center of board
    // Rf = la.inv(Tc).dot(R).dot(Tc).dot(T)

    // return cv2.Rodrigues(Rf[:3, :3])[0].ravel(), Rf[3, :3]
    
    
    /////////////////////////////////////////////////////////////////
    // something from somewherw I have forgotten
    // rotate an image around its center in Python
    // def rotate(image, angle, center = None, scale = 1.0):
    // (h, w) = image.shape[:2]

    // if center is None:
    //     center = (w / 2, h / 2)

    // # Perform the rotation
    // M = cv2.getRotationMatrix2D(center, angle, scale)
    // rotated = cv2.warpAffine(image, M, (w, h))

    // return rotated

       
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
ChArUcoBoard to be printed is in file ChArUcoBoard.jpg
2
true
2000
2
true
3.0
Rx
[1, 0, 0;
 0, -1, -1.224646799147353e-16;
 0, 1.224646799147353e-16, -1]
Ry
[0.7071067811865476, 0, 0.7071067811865476;
 0, 1, 0;
 -0.7071067811865476, 0, 0.7071067811865476]
Rz
[0.9238795325112867, -0.3826834323650898, 0;
 0.3826834323650898, 0.9238795325112867, 0;
 0, 0, 1]
R
[0.6532814824381883, -0.2705980500730985, -0.7071067811865476, 0;
 -0.3826834323650898, -0.9238795325112867, -1.224646799147353e-16, 0;
 -0.6532814824381883, 0.2705980500730986, -0.7071067811865476, 0;
 0, 0, 0, 1]
box center [-1260;
 -840;
 0]
Tc [1, 0, 0, 0;
 0, 1, 0, 0;
 0, 0, 1, 0;
 -595.8323058107145, 1258.239932089494, 595.8323058107144, 1]falsetrue
T [1, 0, 0, 0;
 0, 1, 0, 0;
 0, 0, 1, 0;
 -1260, -840, 4032, 1]falsetrue
Rf [0.6532814824381883, -0.2705980500730985, -0.7071067811865476, 0;
 -0.3826834323650898, -0.9238795325112867, -1.224646799147353e-16, 0;
 -0.6532814824381883, 0.2705980500730986, -0.7071067811865476, 0;
 -595.8323058107145, 1258.239932089494, 4627.832305810714, 1]
RfVector [2.718846028485863, -0.5408121000669039, -1.12618289900307]
RfT [-595.8323058107145, 1258.239932089494, 4627.832305810714]
 */

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

///////////////////////////////////////////////////
 
//  public void testBin()
//  {

//      int axis;
//      axis = 0;
//      for(int i = 0; i<20; i++)
//      {
//          Main.LOGGER.log(Level.FINE, String.valueOf(gb[axis].gen_bin()));
//      }
//      axis = 1;
//      for(int i = 0; i<20; i++)
//      {
//          Main.LOGGER.log(Level.FINE, String.valueOf(gb[axis].gen_bin()));
//      }
//  }