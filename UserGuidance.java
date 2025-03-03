// This project and file are derived in part from the "Pose Calib" project by
// @author Pavel Rojtberg
// It is subject to his license terms in the PoseCalibLICENSE file.

package Guidance;

import static Guidance.ArrayUtils.argmax;
import static Guidance.ArrayUtils.argmin;
import static Guidance.ArrayUtils.isAllTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.logging.Logger;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     UserGuidance class                                          */
/*                                     UserGuidance class                                          */
/*                                     UserGuidance class                                          */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
public class UserGuidance {
    private static Logger LOGGER;
    static {
      LOGGER = Logger.getLogger("");
      LOGGER.finer("Loading");     
    }

/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     UserGuidance constructor                                    */
/*                                     UserGuidance constructor                                    */
/*                                     UserGuidance constructor                                    */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    public Calibrator calib;

    // private final String[] AX_NAMES = {"red", "green", "blue"};
    private final String[] INTRINSICS = {"fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3"};
    private final String[] POSE = {"fx", "ry", "rz", "tx", "ty", "tz"};

    // parameters that are optimized by the same board poses
    private final int PARAM_GROUPS[][] = {{0, 1, 2, 3}, {4, 5, 6, 7, 8}}; // grouping and numbering the INTRINSICS

    // get geometry from tracker
    public ChArucoDetector tracker;
    private int minCornersInitially;
    private int square_len;
    private int marker_len;
    // private int SQUARE_LEN_PIX = 12;

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
    private double pose_close_to_tgt = 0.;
    private boolean pose_reached = false;
    private boolean capture = false;
    private boolean still = false;
    private String user_info_text = "initialization";

    private PoseGeneratorDist posegen;
    private Mat tgt_r = new Mat();
    private Mat tgt_t = new Mat();

    // getters
    public boolean converged()
    {
        return converged;
    }
    public String user_info_text()
    {
        return user_info_text;
    }
    public Mat tgt_r()
    {
        return tgt_r;
    }
    public Mat tgt_t()
    {
        return tgt_t;
    }
    public boolean[] pconverged()
    {
        return pconverged;
    }
    public String[] INTRINSICS()
    {
        return INTRINSICS;
    }
    public double pose_close_to_tgt_get()
    {
        return pose_close_to_tgt;
    }

    public UserGuidance(ChArucoDetector tracker, double var_terminate, Size img_size) // force use of var_terminate=0.1 instead of defaulting
    {
        LOGGER.finer("Instantiating");

        this.img_size = img_size;
        this.tracker = tracker;
        this.var_terminate = var_terminate;
        this.calib = new Calibrator(this.img_size);
        this.pconverged = new boolean[this.calib.nintr()]; // initialized to false by Java
        this.minCornersInitially = (int)(Cfg.board_x*Cfg.board_y*0.4); // intial pose detection requires most of the ArUcos in a board;
        this.square_len = Cfg.square_len;
        this.marker_len = Cfg.marker_len;
        // this.SQUARE_LEN_PIX = this.square_len;
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
        this.set_next_pose(); // set first pose
    }
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     calibrate                                                   */
/*                                     calibrate                                                   */
/*                                     calibrate                                                   */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    private void calibrate()
    {
        LOGGER.finer("method entered  . . . . . . . . . . . . . . . . . . . . . . . .");


        // need at least 2 keyframes to compute the variances, etc.
        if (this.calib.keyframes.size() < 2)
        {
            return;
        }

        double[] pvar_prev = this.calib.varIntrinsics().clone(); // calib still has the previous intrinsics to save here
        boolean first = this.calib.keyframes.size() == 2;

        // compute the new intrinsics in calibrate
        double[] index_of_dispersion = this.calib.calibrate(new ArrayList<>(1)); // dummy arg to use all captures

        double[] pvar = this.calib.varIntrinsics(); // save the new intrinsics (shorter name to match original code) to compare with the previous

        double[] rel_pstd = new double[pvar.length];

        if ( ! first)
        {      
            double total_var_prev = Arrays.stream(pvar_prev).sum();
            double total_var = Arrays.stream(pvar).sum();

            if (total_var > total_var_prev)
            {
                LOGGER.finest("note: total var degraded");
            }
            // check for convergence
            for (int i = 0; i < pvar.length; i++)
            {
                rel_pstd[i] = 1 - Math.sqrt(pvar[i]) / Math.sqrt(pvar_prev[i]); //relative change to each std dev
            }

            LOGGER.finest("relative stddev " + Arrays.toString(rel_pstd));
            
            if (rel_pstd[this.tgt_param] < 0)
            {
                LOGGER.finest(this.INTRINSICS[this.tgt_param] + " degraded");
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
                        if ( ! this.pconverged[p])
                          {
                            converged.append(this.INTRINSICS[p]);
                            this.pconverged[p] = true;
                          }
                    }
                }
                if (converged.length() > 0)
                {
                    LOGGER.finest("{" + converged + "} converged");
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
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     set_next_pose                                               */
/*                                     set_next_pose                                               */
/*                                     set_next_pose                                               */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    private void set_next_pose()
    {
        LOGGER.finer("method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        int nk = this.calib.keyframes.size();
    
        List<Mat> rt = this.posegen.get_pose(this.board_units, // rotation and translation of the guidance board
                                            nk,
                                            this.tgt_param,
                                            this.calib.K(),
                                            this.calib.cdist());
        rt.get(0).copyTo(this.tgt_r);
        rt.get(1).copyTo(this.tgt_t);

        rt.get(1).release();
        rt.remove(1);
        rt.get(0).release();
        rt.remove(0);
        
        this.board.create_maps(this.calib.K(), this.calib.cdist(), this.img_size);
        // make the guidance board warped and right size
        //board_warped_shape =  # Height Width Channels (720, 1280, 3)
        this.board_warped.release();
         
        this.board_warped = this.board.project(this.tgt_r, this.tgt_t, false, Imgproc.INTER_NEAREST);

        LOGGER.finest("r/t and board_warped " + this.tgt_r.dump() + this.tgt_t.dump()  + board_warped);
    }
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     pose_close_to_tgt                                           */
/*                                     pose_close_to_tgt                                           */
/*                                     pose_close_to_tgt                                           */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    /**
     * Jaccard similarity coefficient (Jaccard index) compares estimated image pose and target GuidanceBoard pose.
     * Change from original - Returning numerical index instead of the true/false decision of close to target.
     * Calling program can decide what to do with the number. This can put all the decisions
     * in the same place instead of dispersed.
     * @param progressInsert
     * @return Jaccard similarity coefficient of estimated image pose and desired (target) guidance pose
     */
    private double pose_close_to_tgt(Mat progressInsert)
    {
        LOGGER.finer("method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        LOGGER.finest("pose_valid " + this.tracker.pose_valid() + ", tgt_r empty " + this.tgt_r.empty());
        
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
        tmp.copyTo(progressInsert); // test 1 has the board projected (warped) from where the detector thinks is the camera image pose
        this.overlap.copyTo(tempImg); // tempImg has the warped guidance board

        Core.multiply(progressInsert, Cfg.progressInsertCameraGrey, progressInsert); // brighten (to near white) so it can be seen by humans
        Core.multiply(tempImg, Cfg.progressInsertGuidanceGrey, tempImg); // brighten (to dark gray) so it can be seen by humans
        Core.add(progressInsert, tempImg, progressInsert); // where they overlap is bright white

        LOGGER.finest("shadow_warped created r/t " + this.tracker.rvec().dump() + this.tracker.tvec().dump()  + board_warped);

        int Ab = Core.countNonZero(tmp); // number of on (1) pixels in the warped shadow board
        Core.bitwise_and(this.overlap, tmp, this.overlap); // make the overlapped pixels on (1)
        int Aab = Core.countNonZero(this.overlap); // number of on (1) pixels that overlap on the 2 boards
        
        // circumvents instability during initialization and large variance in depth later on
        // Jaccard similarity index
        jaccard = (double)Aab / (double)(Aa + Ab - Aab);

        tmp.release();
        tempImg.release();

        LOGGER.finest("jaccard " + jaccard);
    
        return jaccard;
    }
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     update                                                      */
/*                                     update                                                      */
/*                                     update                                                      */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    /**
     * @param force
     * @return true if a new pose was captured
     */ 
    public boolean update(boolean force, Mat progressInsert)
    {
        LOGGER.finer("method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // first pose needs to see at least half of the corners
        // this image frame may or may not be good enough to capture so check if better than previous reperr (or initial minimum allowed)
        // if image frame not good enough to capture we'll be back here next time
        // if it is good enough to capture, the calibration is redone far below but that's not too much of a waste
        if (this.calib.keyframes.isEmpty() && this.tracker.N_pts() >= minCornersInitially)
        {
            LOGGER.finest("initial calibrate");
            // try to estimate intrinsic params from single frame
            this.calib.calibrate(Arrays.asList(this.tracker.get_calib_pts())); // no captures yet so estimate from this frame detection
            // is this bootstrap calibration good enough to use at least for the next guidance display
            if (this.calib.reperr() < this.min_reperr_init) // assume K is all numeric - no way it couldn't be, original checked for nan but it never was
            {
                // better than previous reperr so use these intrinsics for next guidance display
                LOGGER.finest("initial set_next_pose and intrinsics");
                this.tracker.set_intrinsics(this.calib); // use the better (we are hopeful) intrinsics (rkt reversed this and the next line! did I screw up?)
                this.set_next_pose();  // update target guidance pose display based on the new intrinsics
                this.min_reperr_init = this.calib.reperr(); // ratchet what's considered better
            }
        }

        // check if alignment to guidance board is good enough to calibrate and keep as a capture

        this.pose_reached = force && this.tracker.N_pts() >= Cfg.minCorners; // original had > 4

        this.pose_close_to_tgt = this.pose_close_to_tgt(progressInsert);

        if (this.pose_close_to_tgt > Cfg.pose_close_to_tgt_min)
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

        LOGGER.finest(
            "corners " + this.tracker.N_pts() +
            ", pose_close_to_tgt " + pose_close_to_tgt +
            ", still " + this.still +
            ", mean_flow " + this.tracker.mean_flow() +
            ", pose_reached " + this.pose_reached +
            ", force " + force);

        if ( ! this.capture)
        {
            return false;            
        }

        // image captured (saved) to use for calibration
        // check for all parameters converged
        // set the next guidance board pose if not all converged

        this.calib.keyframes.add(this.tracker.get_calib_pts());

        LOGGER.info("image capture number " + this.calib.keyframes.size());

        // update calibration with all keyframes
        this.calibrate();

        // use the updated calibration results for tracking
        this.tracker.set_intrinsics(this.calib);

        LOGGER.fine("camera matrix\n" + this.calib.K().dump());
        LOGGER.fine("camera distortion\n" + this.calib.cdist().dump());

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
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     _update_user_info                                           */
/*                                     _update_user_info                                           */
/*                                     _update_user_info                                           */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    private void _update_user_info()
    {
        LOGGER.finer("method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

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
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     draw                                                        */
/*                                     draw                                                        */
/*                                     draw                                                        */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    /**
     * add the guidance board to the camera image to make the new user display
     * @param img in/out; the composite of everything Mat that will be displayed to the user
     * @param mirror
     */
    public void draw(Mat img, boolean mirror) // force users to specify mirror false instead of defaulting
    {
        LOGGER.finer("method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // assumes both img and board are 3 color channels BGR
        if ( ! this.tgt_r.empty())
        {
            // process complete Mats' temp buffers for efficient access
            byte[] imgBuff = new byte[img.rows()*img.cols()*img.channels()];
            byte[] board_warpedBuff = new byte[this.board_warped.rows()*this.board_warped.cols()*this.board_warped.channels()];

            if (imgBuff.length != board_warpedBuff.length) {
                LOGGER.severe("major trouble here");
            }
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

//    seed NOT USED -- NOT CONVERTED

/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     write                                                       */
/*                                     write                                                       */
/*                                     write                                                       */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    public void write()
    {
        LOGGER.finer("method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        LOGGER.info("\nCamera Calibration Data");
        LOGGER.info("nr_of_frames: " + this.calib.keyframes.size());
        LOGGER.info("image_width: " + this.calib.img_size().width);
        LOGGER.info("image_height: " + this.calib.img_size().height);
        LOGGER.info("board_width: " + this.tracker.board_sz().width);
        LOGGER.info("board_height: " + this.tracker.board_sz().height);
        LOGGER.info("square_size: " + this.square_len);
        LOGGER.info("marker_size: " + this.marker_len);
        LOGGER.info("fisheye_model: " + 0);
        LOGGER.info("camera_matrix:\n" + this.calib.K().dump());
        LOGGER.info("distortion_coefficients:\n" + this.calib.cdist().dump());
        LOGGER.info("avg_reprojection_error: " + this.calib.reperr());
        LOGGER.info("End of Calibration\n");
    }
}
