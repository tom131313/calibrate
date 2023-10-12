package calibrator;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;

public class Calibrator {

    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}
    Size img_size;
    int nintr = 9;
    Mat Kin;
    Mat K = new Mat();

    Mat cdist = Mat.zeros(1,5, CvType.CV_64FC1);

    int flags = Calib3d.CALIB_USE_LU;

    // calibration data
    public List<keyframe> keyframes = new ArrayList<>(20);
    int N_pts = 0; // not used - only for Jacobian covariance that was removed
    double reperr = Double.NaN;
    double[] varIntrinsics =  {0., 0., 0., 0., 0., 0., 0., 0., 0.}; // nintr length
    double[] pose_var = {0., 0., 0., 0., 0., 0.}; // rotation x, y, z and translation x, y, z
    double[] disp_idx = // index of dispersion. One for each intrinsic this program handles (nintr=9)
      {Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    List<Mat> p2d = new ArrayList<>(40);
    List<Mat> p3d = new ArrayList<>(40);
    // public List<Mat> allImagePoints = new ArrayList<>(); part of keyframes
    // public List<Mat> allObjectPoints = new ArrayList<>(); part of keyframes

    Calibrator(Size img_size)
    {
      Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        this.img_size = img_size;
        // initial fake camera matrix just to get things started
        Kin = Mat.zeros(3, 3, CvType.CV_64FC1);
        Kin.put(0, 0, 1000.);
        Kin.put(1, 1, 1000.);
        Kin.put(2, 2, 1.);
        Kin = Calib3d.getDefaultNewCameraMatrix(Kin, img_size, true);
        Kin.copyTo(K); // kin needed after this? could just use k?
    }

    public double[] get_intrinsics()
    {
      Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
      double[] intrinsics = {
        this.K.get(0, 0)[0],
        this.K.get(1, 1)[0],
        this.K.get(0, 2)[0],
        this.K.get(1, 2)[0],
        this.cdist.get(0, 0)[0],
        this.cdist.get(0, 1)[0],
        this.cdist.get(0, 2)[0],
        this.cdist.get(0, 3)[0],
        this.cdist.get(0, 4)[0]
        };
      return intrinsics;
    }

    public double[] calibrate(List<keyframe> keyframes) throws Exception
    {
      Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        int flags = this.flags;

        if( keyframes.isEmpty())
        {
          keyframes.addAll(this.keyframes);
        }

        if(keyframes.isEmpty())
        {
          throw new Exception("keyframes is empty");
        }

        int nkeyframes = keyframes.size();

        if(nkeyframes <= 1)
        {
            // restrict first calibration to K matrix parameters
            flags |= Calib3d.CALIB_FIX_ASPECT_RATIO;
        }

        if(nkeyframes <= 1)
        {
          // with only one frame we just estimate the focal length
            flags |= Calib3d.CALIB_FIX_PRINCIPAL_POINT;

            flags |= Calib3d.CALIB_ZERO_TANGENT_DIST;
            flags |= Calib3d.CALIB_FIX_K1 | Calib3d.CALIB_FIX_K2 | Calib3d.CALIB_FIX_K3;
        }

        final Mat cameraMatrix;
        final Mat distCoeffs = new Mat();
        final List<Mat> rvecs = new ArrayList<>();
        final List<Mat> tvecs = new ArrayList<>();

        ArrayList<Object> res = calibrateCamera(keyframes, this.img_size, flags, this.Kin);

        // self.reperr, self.K, self.cdist, rvecs, tvecs, stdDeviationsIntrinsics, self.N_pts = res
        this.reperr = Double.class.cast(res.get(0)); // magic I called it double
        Mat.class.cast(res.get(1)).copyTo(this.K);
        Mat.class.cast(res.get(2)).copyTo(this.cdist);
        Main.LOGGER.log(Level.SEVERE, "res.get(2) " + res.get(2).toString() + " " + cdist.dump() );
        List<Mat> rvecsList = (List<Mat>)res.get(3);
        List<Mat> tvecsList = (List<Mat>)res.get(4);
        Mat variances= Mat.class.cast(res.get(5));// this.varIntrinsics replaces self.PCov in Python
        this.N_pts = Integer.class.cast(res.get(6)); // magic I called it int

        variances.get(0, 0, this.varIntrinsics); // convert from Mat to double array; this.varIntrinsics replaces self.PCov in Python

        this.pose_var = compute_pose_var(rvecsList, tvecsList);

        this.disp_idx = index_of_dispersion(this.get_intrinsics(), this.varIntrinsics);

        return this.disp_idx;
    }

//         self.reperr, self.K, self.cdist, rvecs, tvecs, self.PCov, self.N_pts = res
//         self.pose_var = compute_pose_var(rvecs, tvecs)
//         self.unknowns = self.nintr + 6 * nkeyframes

//         pvar = np.diag(self.PCov)
//         self.mean_extr_var = mean_extr_var(pvar[self.nintr:])

//         self.disp_idx = index_of_dispersion(self.get_intrinsics(), np.diag(self.PCov)[:self.nintr])
//         return self.disp_idx

    public double[] index_of_dispersion(double[] mean, double[] variance)
    {
      Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

      // computes index of dispersion:
      // https://en.wikipedia.org/wiki/Index_of_dispersion
      // compute the 9 VMR's
      if(mean.length != variance.length)
      {
        Main.LOGGER.log(Level.SEVERE, "mean and variance not the same size");
      }
      double[] VMR = new double[9];
      for(int i=0; i<mean.length; i++)
      {
        VMR[i] = variance[i] /  (Math.abs(mean[i]) > 0 ? Math.abs(mean[i]) : 1.);
      }

      return VMR;
    }

/**
 *    mean_extr_var  NOT USED -- NOT CONVERTED
 *    mean_extr_var  NOT USED -- NOT CONVERTED
 *    mean_extr_var  NOT USED -- NOT CONVERTED
*/

/**
 *    Jc2J and compute_state_cov replaced by OpenCV std dev -- NOT CONVERTED
 *    Jc2J and compute_state_cov replaced by OpenCV std dev -- NOT CONVERTED
 *    Jc2Ja nd compute_state_cov replaced by OpenCV std dev -- NOT CONVERTED
 * @throws Exception
*/

    public ArrayList<Object> calibrateCamera(List<keyframe> keyframes, Size img_size, int flags, Mat K) throws Exception
    {
      Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // split keyframes into its two separate components for OpenCV calibrateCamera
        // we put them together when detected then we take them apart for calibration. Maybe that's not a good idea.
        List<Mat> imagePoints = new ArrayList<Mat>(); // pts2d
        List<Mat> objectPoints = new ArrayList<Mat>(); // pts3d
        int N = 0; // count total number of points

        for (keyframe keyframe:keyframes)
        {
            imagePoints.add(keyframe.p2d);
            objectPoints.add(keyframe.p3d);
            N += keyframe.p2d.rows(); // rows or cols?
        }

        if(N <= 4) throw new Exception("not enough total points");

        Mat cdist = new Mat();
        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();
        Mat stdDeviationsIntrinsics = new Mat();
        double DBL_EPSILON = Math.ulp(1.);
        TermCriteria criteria = new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 30, DBL_EPSILON);
        double reperr = Double.NaN;
        try
        {
           reperr = Calib3d.calibrateCameraExtended(
              objectPoints, imagePoints, img_size, K, cdist, rvecs, tvecs, stdDeviationsIntrinsics, new Mat(), new Mat(), flags, criteria); // maybe null for the two empty Mats??

          Main.LOGGER.log(Level.SEVERE, "camera matrix K " + K + "\n" + K.dump());
          Main.LOGGER.log(Level.SEVERE, "distortion coefficients " + cdist + "\n" + cdist.dump());
          Main.LOGGER.log(Level.SEVERE, "repError " + reperr);
        }
        catch(CvException error)
        {
          Main.LOGGER.log(Level.SEVERE, error.toString());
        }

        Mat varianceIntrinsics = new Mat();
        Core.multiply(stdDeviationsIntrinsics, stdDeviationsIntrinsics, varianceIntrinsics); // variance = stddev squared

        Main.LOGGER.log(Level.SEVERE, "cdist " + cdist.dump() + ", N = " + N);
        // return is positional
        ArrayList<Object> res = new ArrayList<>(7);
        res.add(reperr); // 0
        res.add(K); // 1
        res.add(cdist); // 2
        res.add(rvecs); // 3
        res.add(tvecs); // 4
        res.add(varianceIntrinsics); // 5
        res.add(N); // 6
        return res;
    }
     
    public  double[] compute_pose_var(List<Mat> rvecs, List<Mat> tvecs)
    {
      Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

      // void meanStdDev(InputArray src, OutputArray mean, OutputArray stddev, InputArray mask=noArray());
    // def compute_pose_var(rvecs, tvecs):
    //     ret = np.empty(6)
    //     reuler = np.array([cv2.RQDecomp3x3(cv2.Rodrigues(r)[0])[0] for r in rvecs])

    //     # workaround for the given board so r_x does not oscilate between +-180°
    //     reuler[:, 0] = reuler[:, 0] % 360

    //     ret[0:3] = np.var(reuler, axis=0)
    //     ret[3:6] = np.var(np.array(tvecs) / 10, axis=0).ravel()  # [mm]
    //     return ret
      double[] ret = {0., 0., 0., 0., 0., 0.};

      List<double[]> reulers = new ArrayList<>(50);

      for(Mat r:rvecs)
      {
        Mat dst = new Mat();
        Mat mtxR = new Mat();
        Mat mtxQ = new Mat();
        double[] reuler;

        Calib3d.Rodrigues(r, dst);
        reuler = Calib3d.RQDecomp3x3(dst, mtxR, mtxQ);
        // workaround for the given board so r_x does not oscilate between +-180°
        reuler[0] = reuler[0] % 360.; //FIXME verify this works as wanted
        reuler[1] = reuler[1] % 360.;
        reuler[2] = reuler[2] % 360.;

        reulers.add(reuler);
      }
      double[] varReuler = stupidVariance(reulers);

      for(int i=0; i <3; i++)
      {
         ret[i] = varReuler[i];
      }

      List<double[]> translations = new ArrayList<>(50);
      for(Mat t:tvecs)
      {
        double[] tvec = new double[3];
        t.get(0, 0, tvec);

        for(int i=0; i <3; i++)
        {
          tvec[i] /= 10.; // [mm]      // divide by 245 to get inches?
        }
        translations.add(tvec);
      }
        
      double[] varTvecs = stupidVariance(translations); //TODO translation 
      for(int i = 3; i < 6; i++)
      {
         ret[i] = varTvecs[i-3];
      }

      return ret;
    }

    public double[] stupidVariance(List<double[]> data)
    {
      Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

      // always 3 components x, y, z
      double[] sum = {0., 0., 0.};
      double[] sumSqr = {0., 0., 0.};
      for(double[] datum:data)
      {
        for(int i=0; i<3; i++)
        {
          sum[i] += datum[i];
        }
      }

      double[] mean = {sum[0]/data.size(), sum[1]/data.size(), sum[2]/data.size()};

      for(double[] datum:data)
      {
        for(int i=0; i<3; i++)
        {
          sumSqr[i] += Math.pow(datum[i]-mean[i], 2);
        }
      }

      double[] variance = {sumSqr[0]/data.size(), sumSqr[1]/data.size(), sumSqr[2]/data.size()};

      return variance;
    }
}
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

// class Calibrator:
//     def __init__(self, img_size):
//         self.img_size = img_size

//         self.nintr = 9
//         self.unknowns = None  # number of unknowns in our equation system

//         # initial K matrix
//         # with aspect ratio of 1 and pp at center. Focal length is empirical.
//         self.Kin = cv2.getDefaultNewCameraMatrix(np.diag([1000, 1000, 1]), img_size, True)
//         self.K = self.Kin.copy()

//         self.cdist = None

//         self.flags = cv2.CALIB_USE_LU

//         # calibration data
//         self.keyframes = []
//         self.reperr = float("NaN")
//         self.PCov = np.zeros((self.nintr, self.nintr))  # parameter covariance
//         self.pose_var = np.zeros(6)

//         self.disp_idx = None  # index of dispersion

//     def get_intrinsics(self):
//         K = self.K
//         return [K[0, 0], K[1, 1], K[0, 2], K[1, 2]] + list(self.cdist.ravel())

//     def calibrate(self, keyframes=None):
//         flags = self.flags

//         if not keyframes:
//             keyframes = self.keyframes

//         assert(keyframes)

//         nkeyframes = len(keyframes)

//         if nkeyframes <= 1:
//             # restrict first calibration to K matrix parameters
//             flags |= cv2.CALIB_FIX_ASPECT_RATIO

//         if nkeyframes <= 1:
//             # with only one frame we just estimate the focal length
//             flags |= cv2.CALIB_FIX_PRINCIPAL_POINT

//             flags |= cv2.CALIB_ZERO_TANGENT_DIST
//             flags |= cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3

//         res = calibrateCamera(keyframes, self.img_size, flags, self.Kin)
//         #print("len keyframes", len(keyframes)) # rkt
//         self.reperr, self.K, self.cdist, rvecs, tvecs, self.PCov, self.N_pts = res
//         self.pose_var = compute_pose_var(rvecs, tvecs)
//         self.unknowns = self.nintr + 6 * nkeyframes

//         pvar = np.diag(self.PCov)
//         self.mean_extr_var = mean_extr_var(pvar[self.nintr:])

//         self.disp_idx = index_of_dispersion(self.get_intrinsics(), np.diag(self.PCov)[:self.nintr])
//         return self.disp_idx

// def index_of_dispersion(mean, var):
//     """
//     computes index of dispersion:
//     https://en.wikipedia.org/wiki/Index_of_dispersion
//     """
//     return var / [abs(v) if abs(v) > 0 else 1. for v in mean]

// def mean_extr_var(var):
//     """
//     computes the mean of the extrinsic variances
//     @param var: variance vector excluding the intrinsic parameters
//     """
//     assert(len(var) % 6 == 0)
//     nframes = len(var) // 6
//     my_var = var[:6].copy()

//     for i in range(1, nframes - 1):
//         my_var += var[6 * i:6 * (i + 1)]

//     return my_var / nframes

// def Jc2J(Jc, N_pts, nintr=9):
//     """
//     decompose a compact 'single view' jacobian into a sparse 'multi view' jacobian
//     @param Jc: compact single view jacobian 
//     @param N_pts: number of points per view
//     @param nintr: number of intrinsic parameters
//     """
//     total = np.sum(N_pts)

//     J = np.zeros((total * 2, nintr + 6 * len(N_pts)))
//     J[:, :nintr] = Jc[:, 6:]

//     i = 0

//     for j, n in enumerate(N_pts):
//         J[2 * i:2 * i + 2 * n, nintr + 6 * j:nintr + 6 * (j + 1)] = Jc[2 * i:2 * i + 2 * n, :6]
//         i += n

//     return J

// def compute_pose_var(rvecs, tvecs):
//     ret = np.empty(6)
//     reuler = np.array([cv2.RQDecomp3x3(cv2.Rodrigues(r)[0])[0] for r in rvecs])

//     # workaround for the given board so r_x does not oscilate between +-180°
//     reuler[:, 0] = reuler[:, 0] % 360

//     ret[0:3] = np.var(reuler, axis=0)
//     ret[3:6] = np.var(np.array(tvecs) / 10, axis=0).ravel()  # [mm]
//     return ret

// def compute_state_cov(pts3d, rvecs, tvecs, K, cdist, flags):
//     """
//     state covariance from current intrinsic and extrinsic estimate
//     """
//     P_cam = []
//     N_pts = [len(pts) for pts in pts3d]

//     # convert to camera coordinate system
//     for i in range(len(pts3d)):
//         R = cv2.Rodrigues(rvecs[i])[0]
//         P_cam.extend([R.dot(P) + tvecs[i].ravel() for P in pts3d[i]])

//     zero = np.array([0, 0, 0.], dtype=np.float32)

//     # get jacobian
//     Jc = cv2.projectPoints(np.array(P_cam), zero, zero, K, cdist)[1]
//     J = Jc2J(Jc, N_pts)
//     JtJ = J.T.dot(J)
    
//     if flags & (cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3):
//         # TODO: remove the according fixed rows so we can invert this 
//         return np.zeros_like(JtJ)
    
//     # convert la.inv to opencv likely JtJ.inv() rkt
//     JtJinverse = la.inv(JtJ)
//     return JtJinverse
// """
// Python output
// JtJ  [[ 1.51914368e+01  0.00000000e+00 -8.05418439e+00 ... -2.36699145e+00
//    2.82868310e-04 -1.07038621e+00]
//  [ 0.00000000e+00  4.91273784e+00  0.00000000e+00 ... -4.29856045e-04
//    1.19103024e+00 -2.76731990e-01]
//  [-8.05418439e+00  0.00000000e+00  1.99000000e+02 ...  5.48904383e+00
//   -2.85957413e-03  2.35021766e+00]
//  ...
//  [-2.36699145e+00 -4.29856045e-04  5.48904383e+00 ...  7.72624755e-01
//   -7.98138634e-04  3.31186892e-01]
//  [ 2.82868310e-04  1.19103024e+00 -2.85957413e-03 ... -7.98138634e-04
//    7.73419341e-01 -1.66189055e-01]
//  [-1.07038621e+00 -2.76731990e-01  2.35021766e+00 ...  3.31186892e-01
//   -1.66189055e-01  1.88358354e-01]]
// JtJi [[ 158.46083083  153.94073819  -69.51641845 ...   51.00328891
//    336.51579173  863.19901436]
//  [ 153.94073819  150.69750368  -63.63820278 ...   75.5292668
//    342.77282908  852.81041385]
//  [ -69.51641845  -63.63820278   98.99737608 ...  183.41000941
//   -228.12947062 -175.98008647]
//  ...
//  [  51.00328889   75.52926678  183.41000942 ... 6832.03441948
//    833.04235959 2783.63212969]
//  [ 336.51579171  342.77282906 -228.1294706  ...  833.04235962
//   5023.34631516  800.19167068]
//  [ 863.19901435  852.81041384 -175.98008646 ... 2783.63212981
//    800.1916708  6933.56387439]]
// """

// def calibrateCamera(keyframes, img_size, flags, K):
//     pts2d = []
//     pts3d = []
//     N = 0

//     for p2d, p3d in keyframes:
//         pts2d.append(p2d)
//         pts3d.append(p3d)
//         N += len(p2d)

//     objectPoints = np.array(pts3d) # fixed yet? rkt
//     # print("pts3d", pts3d)

//     imagePoints = np.array(pts2d)
//     reperr, K, cdist, rvecs, tvecs = cv2.calibrateCamera(objectPoints, imagePoints, img_size, K, None, flags=flags)
// #     c:\\Users\\RKT\\frc\\FRC2023\\Code\\pose_calib\\utils.py:304: VisibleDeprecationWarning: Creating an ndarray from ragged nested sequences (which is a list-or-tuple of lists-or-tuples-or ndarrays with different lengths or shapes) is deprecated. If you meant to do this, you must specify 'dtype=object' when creating the ndarray.
// #   reperr, K, cdist, rvecs, tvecs = cv2.calibrateCamera(np.array(pts3d), np.array(pts2d), img_size, K, None, flags=flags)

//     cov = compute_state_cov(pts3d, rvecs, tvecs, K, cdist, flags)

//     return reperr, K, cdist, rvecs, tvecs, cov, N
