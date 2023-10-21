package calibrator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.logging.Level;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;

class Calibrator {

    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}
    private Size img_size;
    private int nintr = 9;
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
    Size img_size(){return this.img_size;}
    int nintr(){return this.nintr;}
    double reperr(){return reperr;}
    double[] varIntrinsics(){return varIntrinsics;}
    double[] pose_var(){return pose_var;}
    int flags(){return flags;}
    Mat K(){return K;}
    Mat cdist(){return cdist;}

    Calibrator(Size img_size)
    {
      Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        this.img_size = img_size;
        // initial fake camera matrix just to get things started
        this.Kin = Mat.zeros(3, 3, CvType.CV_64FC1);
        this.Kin.put(0, 0, 1000.);
        this.Kin.put(1, 1, 1000.);
        this.Kin.put(2, 2, 1.);
        this.Kin = Calib3d.getDefaultNewCameraMatrix(this.Kin, img_size, true);
        this.Kin.copyTo(this.K);
        Main.LOGGER.log(Level.WARNING, "K/Kin\n" + this.K.dump() + "\n" + this.Kin.dump());
        Main.Kcsv(Id.__LINE__(), K);
    }

    private double[] get_intrinsics()
    {
      Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
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

    double[] calibrate(List<keyframe> keyframes) throws Exception
    {
      Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

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

        Main.Kcsv(Id.__LINE__(), K);
        List<Object> res = calibrateCamera(keyframes, this.img_size, flags, this.Kin);

        this.reperr = Double.class.cast(res.get(0)); // magic, I called it double
        Mat.class.cast(res.get(1)).copyTo(this.K);
        Mat.class.cast(res.get(2)).copyTo(this.cdist);
        Main.LOGGER.log(Level.WARNING, "res.get(2) " + res.get(2).toString() + " " + cdist.dump() );
        List<Mat> rvecsList = (List<Mat>)res.get(3);
        List<Mat> tvecsList = (List<Mat>)res.get(4);
        Mat variances= Mat.class.cast(res.get(5));// this.varIntrinsics replaces self.PCov in Python
        // this.N_pts = Integer.class.cast(res.get(6)); // magic I called it int
        Main.Kcsv(Id.__LINE__(), K);

        variances.get(0, 0, this.varIntrinsics); // convert from Mat to double array; this.varIntrinsics replaces self.PCov in Python

        this.pose_var = compute_pose_var(rvecsList, tvecsList);

        this.disp_idx = index_of_dispersion(this.get_intrinsics(), this.varIntrinsics);

        return this.disp_idx;
    }

    private static double[] index_of_dispersion(double[] mean, double[] variance)
    {
      Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

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
 *    Jc2J and compute_state_cov  replaced by OpenCV std dev -- NOT CONVERTED
 *    Jc2J and compute_state_cov  replaced by OpenCV std dev -- NOT CONVERTED
 *    Jc2J and compute_state_cov  replaced by OpenCV std dev -- NOT CONVERTED
*/
    
    /**
     * Compute variances and prints the euler angles in degrees for debugging
     * @param rvecs data 1 - List of triplets
     * @param tvecs data 2 -List of triplets
     * @return variances of data 1 and data 2 - a sextet
     */
    private static double[] compute_pose_var(List<Mat> rvecs, List<Mat> tvecs)
    {
      Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

      double[] ret = new double[6];

      List<double[]> reulers = new ArrayList<>(50);

      for(Mat r:rvecs)
      {
        Mat dst = new Mat();
        Mat mtxR = new Mat();
        Mat mtxQ = new Mat();
        double[] reuler;

        Calib3d.Rodrigues(r, dst);
        reuler = Calib3d.RQDecomp3x3(dst, mtxR, mtxQ); // always returns reuler.length = 3
        Main.LOGGER.log(Level.WARNING, "\nreuler degrees " + Arrays.toString(reuler) + "\nr " + r.t().dump());
        // workaround for the given board so r_x does not oscilate between +-180°
        reuler[0] = reuler[0] % 360.;
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
          tvec[i] /= 10.; // [mm]      // divide by 254 to get inches - rkt?
        }
        translations.add(tvec);
      }
        
      double[] varTvecs = stupidVariance(translations);
      for(int i = 3; i < 6; i++)
      {
         ret[i] = varTvecs[i-3];
      }

      return ret;
    }

    private static double[] stupidVariance(List<double[]> data)
    {
      Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

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
    
    /**
     * 
     * @param keyframes
     * @param img_size
     * @param flags
     * @param K
     * @return
     * @throws Exception
     */
    static List<Object> calibrateCamera(List<keyframe> keyframes, Size img_size, int flags, Mat K) throws Exception
    {
      Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // split keyframes into its two separate components, image points and object points, for OpenCV calibrateCamera
        // we put them together when detected then we take them apart for calibration. Maybe that's not a good idea.
        
        List<Mat> pts2d = new ArrayList<>(40); // image points
        List<Mat> pts3d = new ArrayList<>(40); // object points
        int N = 0; // count total number of points

        for (keyframe keyframe:keyframes)
        {
            pts2d.add(keyframe.p2d());
            pts3d.add(keyframe.p3d());
            N += keyframe.p2d().rows(); // rows or cols?
        }

        if(N <= 4) throw new Exception("not enough total points");

        Mat cdist = new Mat();
        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();
        Mat stdDeviationsIntrinsics = new Mat();
        double DBL_EPSILON = Math.ulp(1.);
        TermCriteria criteria = new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 30, DBL_EPSILON);
        double reperr = Double.NaN;
        Main.Kcsv(Id.__LINE__(), K);
        Main.LOGGER.log(Level.WARNING, UserGuidance.formatFlags(flags));
        try
        {
           reperr = Calib3d.calibrateCameraExtended(
              pts3d, pts2d, img_size, K, cdist, rvecs, tvecs, stdDeviationsIntrinsics, new Mat(), new Mat(), flags, criteria); // maybe null for the two empty Mats??

          Main.LOGGER.log(Level.WARNING, "camera matrix K " + K + "\n" + K.dump());
          Main.LOGGER.log(Level.WARNING, "distortion coefficients " + cdist.dump() + cdist);
          Main.LOGGER.log(Level.WARNING, "repError " + reperr);
        }
        catch(CvException error)
        {
          Main.LOGGER.log(Level.SEVERE, error.toString());
        }
        Main.Kcsv(Id.__LINE__(), K);
        Mat varianceIntrinsics = new Mat();
        Core.multiply(stdDeviationsIntrinsics, stdDeviationsIntrinsics, varianceIntrinsics); // variance = stddev squared

        Main.LOGGER.log(Level.WARNING, "cdist " + cdist.dump() + ", N = " + N);

        stdDeviationsIntrinsics.release();

        // return is positional
        ArrayList<Object> res = new ArrayList<>(7);
        res.add(reperr); // 0
        res.add(K); // 1
        res.add(cdist); // 2
        res.add(rvecs); // 3
        res.add(tvecs); // 4
        res.add(varianceIntrinsics); // 5
        // res.add(N); // 6
        return res;
    }
}
