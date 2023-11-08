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
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     Calibrator class                                                       */
/*                                     Calibrator class                                                       */
/*                                     Calibrator class                                                       */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
class Calibrator {
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     Calibrator constructor                                                 */
/*                                     Calibrator constructor                                                 */
/*                                     Calibrator constructor                                                 */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}
    private Size img_size;
    private int nintr = 9;
    // unknowns not used
    private Mat Kin;
    private Mat K = new Mat();
    private Mat cdist = Mat.zeros(1,5, CvType.CV_64FC1);
    private int flags = Calib3d.CALIB_USE_LU;

    // calibration data
    List<keyframe> keyframes = new ArrayList<>(20);
    // private int N_pts = 0; // not used - only for Jacobian covariance that was removed
    private double reperr = Double.NaN;
    private double[] varIntrinsics =  {0., 0., 0., 0., 0., 0., 0., 0., 0.}; // nintr length
    private double[] pose_var = {0., 0., 0., 0., 0., 0.}; // rotation x, y, z and translation x, y, z
    private double[] disp_idx = // index of dispersion. One for each intrinsic this program handles (nintr=9)
      {Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    // getters
    Size img_size()
    {
      return this.img_size;
    }
    int nintr()
    {
      return this.nintr;
    }
    double reperr()
    {
      return this.reperr;
    }
    double[] varIntrinsics()
    {
      return this.varIntrinsics;
    }
    double[] pose_var()
    {
      return this.pose_var;
    }
    int flags()
    {
      return this.flags;
    }
    Mat K()
    {
      return this.K;
    }
    Mat cdist()
    {
      return this.cdist;
    }

    Calibrator(Size img_size)
    {
      //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        this.img_size = img_size;
        // initial fake camera matrix to get things started
        // initial K matrix
        // with aspect ratio of 1 and pp at center. Focal length is empirical.
        this.Kin = Mat.zeros(3, 3, CvType.CV_64FC1);
        this.Kin.put(0, 0, 1000.);
        this.Kin.put(1, 1, 1000.);
        this.Kin.put(2, 2, 1.);
        this.Kin = Calib3d.getDefaultNewCameraMatrix(this.Kin, img_size, true);
        this.Kin.copyTo(this.K);
        //Main.LOGGER.log(Level.WARNING, "K/Kin\n" + this.K.dump() + "\n" + this.Kin.dump());
        //Main.Kcsv(Id.__LINE__(), K);
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     get_intrinsics                                                         */
/*                                     get_intrinsics                                                         */
/*                                     get_intrinsics                                                         */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    private double[] get_intrinsics()
    {
      //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
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
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     calibrate                                                              */
/*                                     calibrate                                                              */
/*                                     calibrate                                                              */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    double[] calibrate(List<keyframe> keyframes) throws Exception // force use of keyframes instead of default None
    {
      //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        int flags = this.flags;

        if ( keyframes.isEmpty())
        {
          keyframes.addAll(this.keyframes);
        }

        if (keyframes.isEmpty())
        {
          throw new Exception("keyframes is empty");
        }

        int nkeyframes = keyframes.size();

        if (nkeyframes <= 1)
        {
            // restrict first calibration to K matrix parameters
            flags |= Calib3d.CALIB_FIX_ASPECT_RATIO;
        }

        if (nkeyframes <= 1)
        {
            // with only one frame we just estimate the focal length
            flags |= Calib3d.CALIB_FIX_PRINCIPAL_POINT;

            flags |= Calib3d.CALIB_ZERO_TANGENT_DIST;
            flags |= Calib3d.CALIB_FIX_K1 | Calib3d.CALIB_FIX_K2 | Calib3d.CALIB_FIX_K3;
        }

        //Main.Kcsv(Id.__LINE__(), K);

        calibrateCameraReturn res = calibrateCamera(keyframes, this.img_size, flags, this.Kin);
 
        this.reperr = res.reperr;
        res.K.copyTo(this.K);
        res.cdist.copyTo(this.cdist);
        List<Mat> rvecsList = res.rvecsList;
        List<Mat> tvecsList = res.tvecsList;
        Mat variances = res.varianceIntrinsics;

        //Main.Kcsv(Id.__LINE__(), K);

        variances.get(0, 0, this.varIntrinsics); // convert from Mat to double array; this.varIntrinsics replaces self.PCov in Python

        this.pose_var = compute_pose_var(rvecsList, tvecsList);

        this.disp_idx = index_of_dispersion(this.get_intrinsics(), this.varIntrinsics);

        return this.disp_idx;
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     index_of_dispersion                                                    */
/*                                     index_of_dispersion                                                    */
/*                                     index_of_dispersion                                                    */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    private static double[] index_of_dispersion(double[] mean, double[] variance)
    {
      //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

      // computes index of dispersion:
      // https://en.wikipedia.org/wiki/Index_of_dispersion
      // compute the 9 VMR's
      if (mean.length != variance.length)
      {
        Main.LOGGER.log(Level.SEVERE, "mean and variance not the same size");
      }
      double[] VMR = new double[9];
      for (int i = 0; i < mean.length; i++)
      {
        VMR[i] = variance[i] / (Math.abs(mean[i]) > 0 ? Math.abs(mean[i]) : 1.);
      }

      return VMR;
    }

/**
 *    mean_extr_var and estimate_pt_std NOT USED -- NOT CONVERTED
 *    mean_extr_var and estimate_pt_std NOT USED -- NOT CONVERTED
 *    mean_extr_var and estimate_pt_std NOT USED -- NOT CONVERTED
*/

/**
 *    Jc2J and compute_state_cov  replaced by OpenCV std dev -- NOT CONVERTED
 *    Jc2J and compute_state_cov  replaced by OpenCV std dev -- NOT CONVERTED
 *    Jc2J and compute_state_cov  replaced by OpenCV std dev -- NOT CONVERTED
*/

/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     compute_pose_var                                                       */
/*                                     compute_pose_var                                                       */
/*                                     compute_pose_var                                                       */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    /**
     * Compute variances and prints the euler angles in degrees for debugging
     * @param rvecs data 1 - List of triplets
     * @param tvecs data 2 - List of triplets
     * @return variances of data 1 and data 2 - a sextet
     */
    private static double[] compute_pose_var(List<Mat> rvecs, List<Mat> tvecs)
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        double[] ret = new double[6]; // return 

        List<double[]> reulers = new ArrayList<>(50);

        // temporary Mats for inside loop but create them only once for efficiency
        Mat dst = new Mat();
        Mat mtxR = new Mat();
        Mat mtxQ = new Mat();

        for (Mat r : rvecs)
        {
          Calib3d.Rodrigues(r, dst);
          double[] reuler = Calib3d.RQDecomp3x3(dst, mtxR, mtxQ); // always returns reuler.length = 3
          //Main.LOGGER.log(Level.WARNING, "\nreuler degrees " + Arrays.toString(reuler) + "\nr " + r.t().dump());
          // workaround for the given board so r_x does not oscilate between +-180°
          reuler[0] = reuler[0] % 360.;
          reulers.add(reuler);
        }

        double[] varReuler = simpleVariance(reulers);

        for (int i = 0; i < 3; i++)
        {
          ret[i] = varReuler[i];
        }

        List<double[]> translations = new ArrayList<>(50);
        for (Mat t : tvecs)
        {
          double[] tvec = new double[3];
          t.get(0, 0, tvec);

          for (int i = 0; i < 3; i++)
          {
            tvec[i] /= 10.; // [mm]      // divide by 254 to get inches - rkt?
          }
          translations.add(tvec);
        }
          
        double[] varTvecs = simpleVariance(translations);
        for (int i = 3; i < 6; i++)
        {
          ret[i] = varTvecs[i - 3];
        }

        dst.release();
        mtxR.release();
        mtxQ.release();

        return ret;
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     simpleVariance                                                         */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    /**
     * Variance of each axis of a list of 3-D points
     * [Could be generalized to any number of dimensions with little change.]
     * @param data
     * @return variances in a 3 element array
     */
    private static double[] simpleVariance(List<double[]> data)
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // always 3 components x, y, z so do all 3 at once
        double[] sum = {0., 0., 0.};
        double[] sumSqr = {0., 0., 0.};
        for (double[] datum : data)
        {
          for (int i = 0; i < 3; i++)
          {
            sum[i] += datum[i];
          }
        }

        double[] mean = {sum[0]/data.size(), sum[1]/data.size(), sum[2]/data.size()};

        for (double[] datum : data)
        {
          for (int i = 0; i < 3; i++)
          {
            sumSqr[i] += Math.pow(datum[i] - mean[i], 2);
          }
        }

        double[] variance = {sumSqr[0]/data.size(), sumSqr[1]/data.size(), sumSqr[2]/data.size()};

        return variance;
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     calibrateCamera                                                        */
/*                                     calibrateCamera                                                        */
/*                                     calibrateCamera                                                        */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    /**
     * 
     * @param keyframes
     * @param img_size
     * @param flags
     * @param K
     * @return
     * @throws Exception
     */
    calibrateCameraReturn calibrateCamera(List<keyframe> keyframes, Size img_size, int flags, Mat K) throws Exception
    {
      //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // split keyframes into its two separate components, image points and object points, for OpenCV calibrateCamera
        // we put them together when detected then we take them apart for calibration.
        
        List<Mat> pts2dFrames = new ArrayList<>(40); // image points
        List<Mat> pts3dFrames = new ArrayList<>(40); // object points
        int N = 0; // count total number of points

        for (keyframe keyframe : keyframes)
        {
            pts2dFrames.add(keyframe.p2d());
            pts3dFrames.add(keyframe.p3d());
            N += keyframe.p2d().rows(); // total points - all frames (poses/views) and all points in those poses
        }

        if (N <= 4) throw new Exception("not enough total points");

        Mat cdist = new Mat();
        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();
        Mat stdDeviationsIntrinsics = new Mat();
        double reperr = Double.NaN;
        //Main.Kcsv(Id.__LINE__(), K);
        //Main.LOGGER.log(Level.WARNING, UserGuidance.formatFlags(flags));
        try
        {
           reperr = Calib3d.calibrateCameraExtended(
            pts3dFrames, pts2dFrames,
            img_size, K, cdist,
            rvecs, tvecs,
            stdDeviationsIntrinsics, new Mat(), new Mat(),
            flags, Cfg.calibrateCameraCriteria); //FIXME maybe null for the two empty Mats?

          //Main.LOGGER.log(Level.WARNING, "camera matrix K " + K + "\n" + K.dump());
          //Main.LOGGER.log(Level.WARNING, "distortion coefficients " + cdist.dump() + cdist);
          //Main.LOGGER.log(Level.WARNING, "repError " + reperr);
        }
        catch(CvException error)
        {
          Main.LOGGER.log(Level.SEVERE, Id.__LINE__() + " " + error.toString());
        }
        //Main.Kcsv(Id.__LINE__(), K);
        Mat varianceIntrinsics = new Mat();
        Core.multiply(stdDeviationsIntrinsics, stdDeviationsIntrinsics, varianceIntrinsics); // variance = stddev squared

        //Main.LOGGER.log(Level.WARNING, "cdist " + cdist.dump() + ", N = " + N);

        stdDeviationsIntrinsics.release();

        return new calibrateCameraReturn(reperr, K, cdist, rvecs, tvecs, varianceIntrinsics);
    }

    class calibrateCameraReturn
    {
        double reperr;
        Mat K;
        Mat cdist;
        List<Mat> rvecsList;
        List<Mat> tvecsList;
        Mat varianceIntrinsics; 
        calibrateCameraReturn(double reperr, Mat K, Mat cdist, List<Mat> rvecs, List<Mat> tvecs, Mat varianceIntrinsics)
        {
            this.reperr = reperr;
            this.K = K;
            this.cdist = cdist;
            this.rvecsList = rvecs;
            this.tvecsList = tvecs;
            this.varianceIntrinsics = varianceIntrinsics;
            // N_pts not used
        }         
    }
}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     End Calibrator class                                                   */
/*                                     End Calibrator class                                                   */
/*                                     End Calibrator class                                                   */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
