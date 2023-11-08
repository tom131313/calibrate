package calibrator;

import static calibrator.ArrayUtils.argmax;
import static calibrator.ArrayUtils.argmin;
import static calibrator.ArrayUtils.isAllTrue;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Level;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     UserGuidance class                                                     */
/*                                     UserGuidance class                                                     */
/*                                     UserGuidance class                                                     */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
public class UserGuidance {
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     UserGuidance constructor                                               */
/*                                     UserGuidance constructor                                               */
/*                                     UserGuidance constructor                                               */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}

    private Calibrator calib;

    private final String[] AX_NAMES = {"red", "green", "blue"};
    private final String[] INTRINSICS = {"fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3"};
    private final String[] POSE = {"fx", "ry", "rz", "tx", "ty", "tz"};

    // parameters that are optimized by the same board poses
    private final int PARAM_GROUPS[][] = {{0, 1, 2, 3}, {4, 5, 6, 7, 8}}; // grouping and numbering the INTRINSICS

    // get geometry from tracker
    private ChArucoDetector tracker;
    private int allpts;
    private int square_len;
    private int marker_len;
    private int SQUARE_LEN_PIX = 12;

    private Size img_size;
    private Mat overlap;
    private BoardPreview board;
    private Mat board_units;
    private Mat board_warped = new Mat();
    private double var_terminate;
    private boolean converged = false;
    private boolean[] pconverged;
    private double min_reperr_init = Double.POSITIVE_INFINITY;

    private int tgt_param = -999_999_999; // None in Python which throws error if accessed; this may throw an error if used as a subscript

    // actual user guidance
    private boolean pose_reached = false;
    private boolean capture = false;
    private boolean still = false;
    private String user_info_text = "";

    private PoseGeneratorDist posegen;
    private Mat tgt_r = new Mat();
    private Mat tgt_t = new Mat();

    // getters
    boolean converged()
    {
        return converged;
    }
    String user_info_text()
    {
        return user_info_text;
    }
    Mat tgt_r()
    {
        return tgt_r;
    }
    Mat tgt_t()
    {
        return tgt_t;
    }
    boolean[] pconverged()
    {
        return pconverged;
    }
    String[] INTRINSICS()
    {
        return INTRINSICS;
    }

    UserGuidance(ChArucoDetector tracker, double var_terminate) throws Exception // force use of var_terminate=0.1 instead of defaulting
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        this.tracker = tracker;
        this.var_terminate = var_terminate;
        this.calib = new Calibrator(tracker.img_size);
        this.pconverged = new boolean[this.calib.nintr()]; // initialized to false by Java
        this.allpts = (Cfg.board_x-1)*(Cfg.board_y-1); // board w = 9 h = 6 => 54 squares; 8x5 => 40 interior corners
        this.square_len = Cfg.square_len;
        this.marker_len = Cfg.marker_len;
        this.SQUARE_LEN_PIX = this.square_len;
        this.img_size = tracker.img_size;
        this.overlap = Mat.zeros((int)this.img_size.height, (int)this.img_size.width, CvType.CV_8UC1);

        // preview image
        this.board = new BoardPreview(this.tracker.boardImage);

        // desired pose of board for first frame
        // translation defined in terms of board dimensions
        this.board_units = new Mat(3, 1, CvType.CV_64FC1);
        this.board_units.put(0, 0, tracker.board_sz().width*this.square_len);
        this.board_units.put(1, 0, tracker.board_sz().height*this.square_len);
        this.board_units.put(2, 0, tracker.board_sz().width*this.square_len);
        this.posegen = new PoseGeneratorDist(this.img_size);

        // set first pose
        this.set_next_pose();
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
    private void calibrate() throws Exception
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        if (this.calib.keyframes.size() < 2) // need at least 2 keyframes
        {
            return;
        }

        double[] pvar_prev = this.calib.varIntrinsics().clone(); // calib still has the previous intrinsics
        boolean first = this.calib.keyframes.size() == 2;

        // compute the new intrinsics
        double[] index_of_dispersion = this.calib.calibrate(new ArrayList<>(1)); // dummy arg to avoid overloaded method

        double[] pvar = this.calib.varIntrinsics(); // save the new intrinsics - just a shorter name to match original

        double[] rel_pstd = new double[pvar.length];

        if ( ! first)
        {      
            double total_var_prev = Arrays.stream(pvar_prev).sum();
            double total_var = Arrays.stream(pvar).sum();

            if (total_var > total_var_prev)
            {
                //Main.LOGGER.log(Level.WARNING, "note: total var degraded");
            }
            // check for convergence
            for (int i = 0; i < pvar.length; i++)
            {
                rel_pstd[i] = 1 - Math.sqrt(pvar[i]) / Math.sqrt(pvar_prev[i]); //relative change to each std dev
            }

            //Main.LOGGER.log(Level.WARNING, "relative stddev " + Arrays.toString(rel_pstd));
            
            if (rel_pstd[this.tgt_param] < 0)
            {
                //Main.LOGGER.log(Level.WARNING, this.INTRINSICS[this.tgt_param] + " degraded");
            }

            // g0(p0 p1 p2 p3)  g1(p4 p5 p6 p7 p8)
            for (int[] g : this.PARAM_GROUPS) // loop through all groups (2 groups)
            {
                // check if tgt_parm in this group
                boolean inGroup = false; // first assume not in this group
                for (int p : g) // loop through whole group (4 or 5 items)
                {
                    if (this.tgt_param == p)
                    {
                        inGroup = true; // found it in this group
                        break; // no need to check further
                    }
                }

                if ( ! inGroup)
                {
                    continue; // not in this group so move on to next group            
                }

                StringBuilder converged = new StringBuilder();

                for (int p : g)
                {
                    if (rel_pstd[p] > 0 && rel_pstd[p] < this.var_terminate)
                    {    
                        if (! this.pconverged[p])
                          {
                            converged.append(this.INTRINSICS[p]);
                            this.pconverged[p] = true;
                          }
                    }
                }
                if ( ! converged.isEmpty())
                {
                    //Main.LOGGER.log(Level.WARNING, "{" + converged + "} converged");
                }
            }
        }
        // if an intrinsic has converged, then set it to 0 so it can't be selected (again) as the max 
        for (int i = 0; i < this.pconverged.length; i++)
        {
            if (this.pconverged[i])
            {
                index_of_dispersion[i] = 0.;
            }
        }
        
        this.tgt_param = argmax(index_of_dispersion);
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     set_next_pose                                                          */
/*                                     set_next_pose                                                          */
/*                                     set_next_pose                                                          */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    private void set_next_pose() throws Exception
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        int nk = this.calib.keyframes.size();
    
        List<Mat> rt = this.posegen.get_pose(this.board_units, // rotation and translation of the guidance board
                                            nk,
                                            this.tgt_param,
                                            this.calib.K(),
                                            this.calib.cdist());
        rt.get(0).copyTo(this.tgt_r);
        rt.get(1).copyTo(this.tgt_t);

        //Main.LOGGER.log(Level.WARNING, "rt1 " + tgt_r.dump() + " " + tgt_t.dump());
        rt.get(1).release();
        rt.remove(1);
        rt.get(0).release();
        rt.remove(0);
        
        this.board.create_maps(this.calib.K(), this.calib.cdist(), this.img_size);
        // make the guidance board warped and right size
        //board_warped_shape =  # Height Width Channels (720, 1280, 3)
        this.board_warped.release(); // rkt
        //Main.LOGGER.log(Level.WARNING, "rt2 " + this.tgt_r.dump() + " " + this.tgt_t.dump());

        this.board_warped = this.board.project(this.tgt_r, this.tgt_t, false, Imgproc.INTER_NEAREST);

        //Main.LOGGER.log(Level.WARNING, "board_warped created r/t " + this.tgt_r.dump() + this.tgt_t.dump()  + board_warped);
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     pose_close_to_tgt                                                      */
/*                                     pose_close_to_tgt                                                      */
/*                                     pose_close_to_tgt                                                      */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    /**
     * Jaccard similarity coefficient (Jaccard index) compares estimated image pose and target GuidanceBoard pose.
     * Change from original - Returning numerical index instead of the true/false decision of close to target.
     * Calling program can decide what to do with the number. This can put all the decisions
     * in the same place instead of dispersed.
     * @return Jaccard similarity coefficient of estimated image pose and desired (target) guidance pose
     */
    private double pose_close_to_tgt()
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        //Main.LOGGER.log(Level.WARNING, "pose_valid " + this.tracker.pose_valid() + ", tgt_r empty " + this.tgt_r.empty());
        
        double jaccard = 0.;
    
        if ( ! this.tracker.pose_valid())
            return jaccard;

        if (this.tgt_r.empty())
            return jaccard;
    
        byte[] board_warpedArray = new byte[this.board_warped.rows()*this.board_warped.cols()*this.board_warped.channels()];
        this.board_warped.get(0, 0, board_warpedArray); // efficient retrieval of complete board_warped

        byte[] overlapArray = new byte[this.overlap.rows()*this.overlap.cols()]; // 1 channel; java sets this to all zeros

        int indexBoard_warpedArray = 1; // start position; extracting channel 1 (of 0, 1, 2)
        for (int row = 0; row < overlapArray.length; row++)
        {
            if (board_warpedArray[indexBoard_warpedArray] != 0)
            {
                overlapArray[row] = 1;
            }
            indexBoard_warpedArray += 3; // bump to next pixel; incrementing by number of channels
        }
        this.overlap.put(0, 0, overlapArray);


        int Aa = Core.countNonZero(this.overlap); // number of on (1) pixels in the warped_board (from above)

        Mat tmp = this.board.project(this.tracker.rvec(), // create projected shadow same way as the guidance board but using the estimated pose of the camera image
                                    this.tracker.tvec(), 
                                    true,
                                    Imgproc.INTER_NEAREST);
        // debug display
        Mat tempImg = new Mat();
        tmp.copyTo(Main.testImg1); // test 1 has the board projected (warped) from where the detector thinks is the camera image pose
        this.overlap.copyTo(tempImg); // tempImg has the warped guidance board

        Core.multiply(Main.testImg1, new Scalar(220.), Main.testImg1); // brighten (to near white) so it can be seen by humans
        Core.multiply(tempImg, new Scalar(130.), tempImg); // brighten (to dark gray) so it can be seen by humans
        Core.add(Main.testImg1, tempImg, Main.testImg1); // where they overlap is bright white

        //Main.LOGGER.log(Level.WARNING, "shadow_warped created r/t " + this.tracker.rvec().dump() + this.tracker.tvec().dump()  + board_warped);

        int Ab = Core.countNonZero(tmp); // number of on (1) pixels in the warped shadow board
        Core.bitwise_and(this.overlap, tmp, this.overlap); // make the overlapped pixels on (1)
        int Aab = Core.countNonZero(this.overlap); // number of on (1) pixels that overlap on the 2 boards
        
        // circumvents instability during initialization and large variance in depth later on
        // Jaccard similarity index
        jaccard = (double)Aab / (double)(Aa + Ab - Aab);

        tmp.release();
        tempImg.release();

        //Main.LOGGER.log(Level.WARNING, "jaccard " + jaccard);
    
        return jaccard;
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     update                                                                 */
/*                                     update                                                                 */
/*                                     update                                                                 */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    /**
     * @param force
     * @return true if a new pose was captured
     * @throws Exception
     */ 
    boolean update(boolean force) throws Exception
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // first time need to see at least half of the interior corners or force
        if ((this.calib.keyframes.isEmpty() && this.tracker.N_pts() >= this.allpts/2))
        {Main.LOGGER.log(Level.WARNING, "initial calibrate");
            // try to estimate intrinsic params from single frame
            this.calib.calibrate(Arrays.asList(this.tracker.get_calib_pts()));

            if ( /*! np.isnan(this.calib.K()).any() &&*/ this.calib.reperr() < this.min_reperr_init) // assume K is all numeric - no way it couldn't be
            {Main.LOGGER.log(Level.WARNING, "initial set_next_pose and intrinsics");
                this.set_next_pose();  // update target pose
                this.tracker.set_intrinsics(this.calib);
                this.min_reperr_init = this.calib.reperr();
            }
        }

        this.pose_reached = force && this.tracker.N_pts() >= Cfg.minCorners; // original had > 4

        double pose_close_to_tgt = this.pose_close_to_tgt();

        if (pose_close_to_tgt > Cfg.pose_close_to_tgt_min)
        {
            this.pose_reached = true;
        }
        // we need at least 57.5 points after 2 frames # rkt - the calc yields 27 with init nintr of 9, not 57.5
        // and 15 points per frame from then on
        int n_required = ((this.calib.nintr() + 2 * 6) * 5 + 3) / (2 * 2); // 27

        if (this.calib.keyframes.size() >= 2)
        {
            n_required = 6 / 2 * 5; // yup - that's a 15 rkt
        }

        this.still = this.tracker.mean_flow() < Cfg.mean_flow_max;
        // use all points instead to ensure we have a stable pose
        this.pose_reached &= this.tracker.N_pts() >= n_required;

        this.capture = this.pose_reached && (this.still || force);

        //Main.LOGGER.log(Level.WARNING,
            // "corners " + this.tracker.N_pts() +
            // ", pose_close_to_tgt " + pose_close_to_tgt +
            // ", still " + this.still +
            // ", mean_flow " + this.tracker.mean_flow() +
            // ", pose_reached " + this.pose_reached +
            // ", force " + force);

        if ( ! this.capture)
        {
            return false;            
        }

        this.calib.keyframes.add(this.tracker.get_calib_pts());

        // update calibration with all keyframe
        this.calibrate();

        // use the updated calibration results for tracking
        this.tracker.set_intrinsics(this.calib);

        this.converged = isAllTrue(this.pconverged);

        if (this.converged)
        {
            this.tgt_r.release();
            this.tgt_r = new Mat(); // clear the rotation
        }
        else
        {
            this.set_next_pose();
        }

        this._update_user_info();

        return true;
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     _update_user_info                                                      */
/*                                     _update_user_info                                                      */
/*                                     _update_user_info                                                      */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    private void _update_user_info()
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        this.user_info_text = "";

        if (this.calib.keyframes.size() < 2)
        {
            this.user_info_text = "initialization";
        }
        else if ( ! this.converged)
        {
            String action = "";
            int axis;
            if (this.tgt_param < 2)
            {
                action = "rotate";
                // do not consider r_z as it does not add any information
                double[] temp = {this.calib.pose_var()[0], this.calib.pose_var()[1]};
                axis = argmin(temp);
            }
            else
            {
                action = "translate";
                // do not consider t_z
                //FIXME above comment doesn't seem to match the code below
                double[] temp = {this.calib.pose_var()[3], this.calib.pose_var()[4], this.calib.pose_var()[5]};
                axis = argmin(temp) + 3; // find min of t_x, t_y, t_z and add 3 to that index to skip the rotation locations
            }
            String param = this.INTRINSICS[this.tgt_param];
            this.user_info_text = String.format("{%s} {%s} to minimize {%s}", action, this.POSE[axis], param);
        }
        else
        {
            this.user_info_text = "converged at MSE: {" + this.calib.reperr() + "}";
        }

        if (this.pose_reached && ! this.still)
        {
            this.user_info_text += "\nhold camera steady";
        }
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     draw                                                                   */
/*                                     draw                                                                   */
/*                                     draw                                                                   */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    /**
     * add the guidance board to the camera image to make the new user display
     * @param img in/out; the composite of everything Mat that will be displayed to the user
     * @param mirror
     * @throws Exception
     */
    void draw(Mat img, boolean mirror) throws Exception // force users to specify mirror false instead of defaulting
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // assumes both img and board are 3 color channels BGR
        if ( ! this.tgt_r.empty())
        {
            // process complete Mats' temp buffers for efficient access
            byte[] imgBuff = new byte[img.rows()*img.cols()*img.channels()];
            byte[] board_warpedBuff = new byte[this.board_warped.rows()*this.board_warped.cols()*this.board_warped.channels()];

            if (imgBuff.length != board_warpedBuff.length) throw new Exception("major trouble here"); // debug statement

            img.get(0, 0, imgBuff); // get the Mat
            this.board_warped.get(0, 0,board_warpedBuff); // get the Mat

            for (int index = 0; index < imgBuff.length; index++)
            {
                // if there is a non-black pixel in the warped board then use it in img
                if (board_warpedBuff[index] != 0)
                {
                    imgBuff[index] = board_warpedBuff[index];
                }
            }
            img.put(0, 0, imgBuff); // update the Mat            
        }

        if (this.tracker.pose_valid())
        {
            this.tracker.draw_axis(img); // draw axes on the detected board from the camera image
        }

        if (mirror)
        {
            Core.flip(img, img, 1);
        }
    }

/**
 *    seed NOT USED -- NOT CONVERTED
 *    seed NOT USED -- NOT CONVERTED
 *    seed NOT USED -- NOT CONVERTED
*/

/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     write                                                                  */
/*                                     write                                                                  */
/*                                     write                                                                  */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    void write()
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        String pattern = "yyyy-MM-dd-HH-mm-ss";
        SimpleDateFormat simpleDateFormat = new SimpleDateFormat(pattern);
        String calibrationDataFile = "CameraCalibrationData_" + simpleDateFormat.format(new Date()) + ".txt";

        Main.LOGGER.log(Level.SEVERE, "calibration data file " + calibrationDataFile);

        try (PrintWriter pw = new PrintWriter(calibrationDataFile))
        {
            pw.println("calibration_time: " + LocalDateTime.now());
            pw.println("nr_of_frames: " + this.calib.keyframes.size());
            pw.println("image_width: " + this.calib.img_size().width);
            pw.println("image_height: " + this.calib.img_size().height);
            pw.println("board_width: " + this.tracker.board_sz().width);
            pw.println("board_height: " + this.tracker.board_sz().height);
            pw.println("square_size: " + this.square_len);
            pw.println("marker_size: " + this.marker_len);
            pw.println(formatFlags(calib.flags()));
            pw.println("fisheye_model: " + 0);
            pw.println("camera_matrix:\n" + this.calib.K().dump());
            pw.println("distortion_coefficients:\n" + this.calib.cdist().dump());
            pw.println("avg_reprojection_error: " + this.calib.reperr());
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     formatFlags                                                            */
/*                                     formatFlags                                                            */
/*                                     formatFlags                                                            */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    static String formatFlags(int flagsCalibration)
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        HashMap<Integer, String> flags = new HashMap<>(3);
        flags.put(Calib3d.CALIB_FIX_PRINCIPAL_POINT, "+fix_principal_point");
        flags.put(Calib3d.CALIB_ZERO_TANGENT_DIST, "+zero_tangent_dist");
        flags.put(Calib3d.CALIB_USE_LU, "+use_lu");
        flags.put(Calib3d.CALIB_FIX_ASPECT_RATIO, "+fix aspect ratio");
        flags.put(Calib3d.CALIB_FIX_PRINCIPAL_POINT, "+fix principal point");
        flags.put(Calib3d.CALIB_ZERO_TANGENT_DIST, "+zero tangent dist");
        flags.put(Calib3d.CALIB_FIX_K1, "+fix k1");
        flags.put(Calib3d.CALIB_FIX_K2, "+fix k2");
        flags.put(Calib3d.CALIB_FIX_K3, "+fix k3");
       
        StringBuilder flags_str = new StringBuilder("flags: ");
        int unknownFlags = flagsCalibration; // initially assume all flags are unknown to the hashmap

        for (Map.Entry<Integer, String> flag : flags.entrySet())
        {
            if ((flagsCalibration & flag.getKey()) == flag.getKey())
            {
                flags_str.append(flag.getValue());
                unknownFlags -= flag.getKey(); // this flag is known so un-mark unknown flags
            }                       
        }

        flags_str.append(String.format("\nflags: %08x", flagsCalibration));
        if (unknownFlags != 0)
        {
            flags_str.append(String.format("; unknown flag usage = %08x", unknownFlags));          
        }
        return flags_str.toString();
    }
}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     End UserGuidance class                                                 */
/*                                     End UserGuidance class                                                 */
/*                                     End UserGuidance class                                                 */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
