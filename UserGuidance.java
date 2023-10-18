package calibrator;

import static calibrator.ArrayUtils.argmax;
import static calibrator.ArrayUtils.argmin;
import static calibrator.ArrayUtils.isAllTrue;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.logging.Level;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
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

    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}

    public Calibrator calib;

    String[] AX_NAMES = {"red", "green", "blue"};
    String[] INTRINSICS = {"fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3"};
    String[] POSE = {"fx", "ry", "rz", "tx", "ty", "tz"};

    // parameters that are optimized by the same board poses
    int PARAM_GROUPS[][] = {{0, 1, 2, 3}, {4, 5, 6, 7, 8}}; // grouping and numbering the INTRINSICS

    // get geometry from tracker
    public ChArucoDetector tracker;
    int allpts;
    int square_len;
    int marker_len;
    int SQUARE_LEN_PIX = 12; // immediately changed in constructor. Why init here? Maybe 0 init would be safer.

    Size img_size;
    Mat overlap;
    BoardPreview board;
    Mat board_units;
    Mat board_warped = new Mat();
    private double var_terminate;
    boolean converged = false;
    boolean[] pconverged;
    double min_reperr_init = Double.POSITIVE_INFINITY;

    int tgt_param = -1; // not sure this needs init but it came from None in Python which throws error if accessed

    // actual user guidance
    boolean pose_reached = false;
    boolean capture = false;
    boolean still = false;
    public String user_info_text = "";

    PoseGeneratorDist posegen;
    Mat tgt_r = new Mat();
    Mat tgt_t = new Mat();

    int unknownFlags;
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     UserGuidance constructor                                               */
/*                                     UserGuidance constructor                                               */
/*                                     UserGuidance constructor                                               */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    UserGuidance(ChArucoDetector tracker, double var_terminate) throws Exception // force use of var_terminate=0.1 instead of defaulting
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        this.tracker = tracker;
        this.var_terminate = var_terminate;
        calib = new Calibrator(tracker.img_size);
        pconverged = new boolean[calib.nintr];
        Arrays.fill(pconverged, false);

        this.allpts = (Cfg.board_x-1)*(Cfg.board_y-1); // board w = 9 h = 6 = 54 squares; 8x5 = 40 interior corners
        this.square_len = Cfg.square_len;
        this.marker_len = Cfg.marker_len;
        this.SQUARE_LEN_PIX = this.square_len;
        this.img_size = tracker.img_size;
        this.overlap = Mat.zeros((int)img_size.height, (int)img_size.width, CvType.CV_8UC1);

        // preview image
        this.board = new BoardPreview(this.tracker.boardImage);
        Main.LOGGER.log(Level.WARNING, "whole board " + this.board.img);
        // desired pose of board for first frame
        // translation defined in terms of board dimensions
        board_units = new Mat(3, 1, CvType.CV_64FC1);
        board_units.put(0, 0, tracker.board_sz.width*square_len);
        board_units.put(1, 0, tracker.board_sz.height*square_len);
        board_units.put(2, 0, tracker.board_sz.width*square_len);
        this.posegen = new PoseGeneratorDist(img_size);

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
    public void calibrate() throws Exception
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        if(this.calib.keyframes.size() < 2) // need at least 2 keyframes
        {
            return;
        }

        double[] pvar_prev = this.calib.varIntrinsics.clone(); // calib still has the previous intrinsics
        boolean first = this.calib.keyframes.size() == 2;

        List<keyframe> placeholder = new ArrayList<>(20); // dummy so I don't have to overload the method
        // compute the new intrinsics
        double[] index_of_dispersion = this.calib.calibrate(placeholder).clone();  // local copy so original not changed

        double[] pvar = this.calib.varIntrinsics; // save the new intrinsics - just a shorter name

        double[] rel_pstd = new double[pvar.length];
        if(! first)
        {      
            double total_var_prev = Arrays.stream(pvar_prev).sum();
            double total_var = Arrays.stream(pvar).sum();

            if(total_var > total_var_prev)
            {
                Main.LOGGER.log(Level.WARNING, "note: total var degraded");
            }
            // check for convergence
            for(int i = 0; i < pvar.length; i++)
            {
                rel_pstd[i] = 1 - Math.sqrt(pvar[i]) / Math.sqrt(pvar_prev[i]);
            }

            Main.LOGGER.log(Level.WARNING, "relative stddev " + Arrays.toString(rel_pstd));
            
            if(rel_pstd[this.tgt_param] < 0)
            {
                Main.LOGGER.log(Level.WARNING, this.INTRINSICS[this.tgt_param] + " degraded");
            }

            // g (0, 1, 2, 3)  p 0 p 1 p 2 p 3 g (4, 5, 6, 7, 8) p 4 p 5 p 6 p 7 p 8
            for(int gIdx=0; gIdx< this.PARAM_GROUPS.length; gIdx++) // loop through all groups (2 groups)
            {
                int[] g = this.PARAM_GROUPS[gIdx]; // change the name to match original

                // check if in this group
                boolean inGroup = false; // first assume not in this group
                for(int p : g) // loop through whole group (4 or 5 items)
                {
                    if(this.tgt_param == p)
                    {
                        inGroup = true; // found it in this group
                        break; // so no need to check further
                    }
                }
                if( ! inGroup)
                {
                    continue; // not in this group so move on to next group            
                }

                StringBuilder converged = new StringBuilder();

                for(int p : g)
                {
                    // if index_of_dispersion[p] < 0.05:
                    if(rel_pstd[p] > 0 && rel_pstd[p] < this.var_terminate)
                    {    
                        if(! this.pconverged[p])
                          {
                            converged.append(this.INTRINSICS[p]);
                            this.pconverged[p] = true;
                          }
                    }
                }
                if( ! converged.isEmpty())
                {
                    Main.LOGGER.log(Level.WARNING, "{" + converged + "} converged");
                }
            }
        }
        // if an intrinsic has converged, then set it to 0 so it can't be selected (again) as the max 
        for(int i = 0; i < this.pconverged.length; i++)
        {
            if(this.pconverged[i])
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
    public void set_next_pose() throws Exception
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        int nk = this.calib.keyframes.size();
    
        List<Mat> rt = this.posegen.get_pose(this.board_units, // rotation and translation of the guidance board
                                            nk,
                                            this.tgt_param,
                                            this.calib.K,
                                            this.calib.cdist);
        rt.get(0).copyTo(this.tgt_r);
        rt.get(1).copyTo(this.tgt_t);

        rt.get(0).copyTo(tracker.tgt_r); // copy for estimating camera img pose
        rt.get(1).copyTo(tracker.tgt_t);
                                    
        Main.LOGGER.log(Level.WARNING, "rt1 " + tgt_r.dump() + " " + tgt_t.dump());
        rt.get(1).release();
        rt.remove(1);
        rt.get(0).release();
        rt.remove(0);
        
        this.board.create_maps(this.calib.K, this.calib.cdist, this.img_size);
        // make the guidance board warped and right size
        //board_warped_shape =  # Height Width Channels (720, 1280, 3)
        this.board_warped.release(); // rkt
        Main.LOGGER.log(Level.WARNING, "rt2 " + this.tgt_r.dump() + " " + this.tgt_t.dump());

        this.board_warped = this.board.project(this.tgt_r, this.tgt_t, false, Imgproc.INTER_NEAREST);

        Main.LOGGER.log(Level.WARNING, "board_warped created r/t " + this.tgt_r.dump() + this.tgt_t.dump()  + board_warped);
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
    public double pose_close_to_tgt()
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        Main.LOGGER.log(Level.WARNING, "pose_valid " + this.tracker.pose_valid + ", tgt_r empty " + this.tgt_r.empty());
        
        double jaccard = 0.;
    
        if( ! this.tracker.pose_valid)
            return jaccard;

        if(this.tgt_r.empty())
            return jaccard;
    
        // this.overlap[:, :] = this.board_warped[:, :, 1] != 0 // initialize overlap with essentially the warped board channel 1
        // for(int row = 0; row < this.overlap.rows(); row++)
        // for(int col = 0; col < this.overlap.cols(); col++)
        // {
        //     this.overlap.put(row, col, this.board_warped.get(row, col)[1] != 0 ? 1. : 0.);                
        // }

        byte[] board_warpedArray = new byte[this.board_warped.rows()*this.board_warped.cols()*this.board_warped.channels()];
        this.board_warped.get(0, 0, board_warpedArray); // efficient retrieval of board_warped

        byte[] overlapArray = new byte[this.overlap.rows()*this.overlap.cols()]; // 1 channel, java sets this to all zeros

        int indexBoard_warpedArray = 1; // start position; extracting channel 1 (of 0, 1, 2)
        for(int row = 0; row < overlapArray.length; row++)
        {
            if(board_warpedArray[indexBoard_warpedArray] != 0)
            {
                overlapArray[row] = 1;
            }
            indexBoard_warpedArray += 3; // bump to next pixel; incrementing by number of channels
        }
        this.overlap.put(0, 0, overlapArray);

        this.overlap.copyTo(Main.testImg2); // test 2 has the guidance board b&w//FIXME at this point the rotation of the shadow board is reversed from Python
        Core.multiply(Main.testImg2, new Scalar(175.), Main.testImg2);
        Imgproc.putText(Main.testImg2, Main.frame, new Point(0, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(0, 0, 0), 4);
        Imgproc.putText(Main.testImg2, Main.frame, new Point(0, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(255, 255, 255), 2);

        int Aa = Core.countNonZero(this.overlap); // number of on (1) pixels in the warped_board (from above)

        Mat tmp = this.board.project(this.tracker.rvec, // create projected shadow same way as the guidance board (hopefully)
                                    this.tracker.tvec, 
                                    true,
                                    Imgproc.INTER_NEAREST);
                                    
        tmp.copyTo(Main.testImg1); // test 1 has the board projected from where the detector thinks is the camera image pose
        Core.multiply(Main.testImg1, new Scalar(255.), Main.testImg1);
        Imgproc.putText(Main.testImg1, Main.frame, new Point(0, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(0, 0, 0), 4);
        Imgproc.putText(Main.testImg1, Main.frame, new Point(0, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(255, 255, 255), 2);
        
        Main.LOGGER.log(Level.WARNING, "shadow_warped created r/t " + this.tracker.rvec.dump() + this.tracker.tvec.dump()  + board_warped);

        int Ab = Core.countNonZero(tmp); // number of on (1) pixels in the warped shadow board
        Core.bitwise_and(this.overlap, tmp, this.overlap); // make the overlapped pixels on (1)
        int Aab = Core.countNonZero(this.overlap); // number of on (1) pixels that overlap on the 2 boards
        
        // circumvents instability during initialization and large variance in depth later on
        // Jaccard similarity index
        jaccard = (double)Aab / (double)(Aa + Ab - Aab);

        tmp.release();
        Main.LOGGER.log(Level.WARNING, "jaccard " + jaccard);
    
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
    public boolean update(boolean force) throws Exception
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        // first time need to see at least half of the interior corners or force
        if((this.calib.keyframes.isEmpty() && this.tracker.N_pts >= this.allpts/2))
        {
            // try to estimate intrinsic params from single frame
            this.calib.calibrate(Arrays.asList(this.tracker.get_calib_pts()));

            if( /*! np.isnan(this.calib.K).any() &&*/ this.calib.reperr < this.min_reperr_init)
            {
                this.set_next_pose();  // update target pose
                this.tracker.set_intrinsics(this.calib);
                this.min_reperr_init = this.calib.reperr;
            }
        }

        this.pose_reached = force && this.tracker.N_pts >= Cfg.minCorners; // original had > 4

        double pose_close_to_tgt = this.pose_close_to_tgt(); // save it to print it
        if(pose_close_to_tgt > Cfg.pose_close_to_tgt_min)
        {
            this.pose_reached = true;
        }
        // we need at least 57.5 points after 2 frames # rkt - the calc yields 27 with init nintr of 9, not 57.5
        // and 15 points per frame from then on
        int n_required = ((this.calib.nintr + 2 * 6) * 5 + 3) / (2 * 2); // 27

        if(this.calib.keyframes.size() >= 2)
        {
            n_required = 6 / 2 * 5; // yup - that's a 15 rkt
        }

        this.still = this.tracker.mean_flow < Cfg.mean_flow_max;
        // use all points instead to ensure we have a stable pose
        this.pose_reached &= this.tracker.N_pts >= n_required;

        this.capture = this.pose_reached && (this.still || force);

        Main.LOGGER.log(Level.WARNING,
            "corners " + this.tracker.N_pts +
            ", pose_close_to_tgt " + pose_close_to_tgt +
            ", still " + this.still +
            ", mean_flow " + this.tracker.mean_flow +
            ", pose_reached " + this.pose_reached +
            ", force " + force);

        if( ! this.capture)
        {
            return false;            
        }

        this.calib.keyframes.add(this.tracker.get_calib_pts());

        // update calibration with all keyframe
        this.calibrate();

        // use the updated calibration results for tracking
        this.tracker.set_intrinsics(this.calib);

        this.converged = isAllTrue(this.pconverged);

        if(this.converged)
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

//     def update(self, force=False, dry_run=False):
//         """
//         @return True if a new pose was captured
//         """
//         if not self.calib.keyframes and self.tracker.N_pts >= self.allpts // 2:
//             # print("update in if", len(self.calib.keyframes), self.calib.keyframes, self.calib.reperr, sep=", ")
//             # try to estimate intrinsic params from single frame
//             self.calib.calibrate([self.tracker.get_calib_pts()])

//             if not np.isnan(self.calib.K).any() and self.calib.reperr < self.min_reperr_init:
//                 self.set_next_pose()  # update target pose
//                 self.tracker.set_intrinsics(self.calib)
//                 self.min_reperr_init = self.calib.reperr
                
//         # print("b4 pose_reached", force, self.tracker.N_pts > 4)
//         self.pose_reached = force and self.tracker.N_pts > 4

//         if self.pose_close_to_tgt() > GlobalStuff.pose_close_to_tgt_min:
//             self.pose_reached = True
//         # print("after if pose_reached", self.pose_reached)
//         # we need at least 57.5 points after 2 frames # rkt - the calc yields 27 with init nintr of 9, not 57.5
//         # and 15 points per frame from then
//         n_required = ((self.calib.nintr + 2 * 6) * 5 + 3) // (2 * 2)  # integer floor

//         if len(self.calib.keyframes) >= 2:
//             n_required = 6 // 2 * 5 # yup - that's a 15 rkt
//             if self.tracker.N_pts < 17: # rkt
//                 n_required = int(self.tracker.N_pts * GlobalStuff.n_requiredFactor) # rkt

//         self.still = self.tracker.mean_flow < GlobalStuff.mean_flow_max
//         # use all points instead to ensure we have a stable pose

//         self.pose_reached *= self.tracker.N_pts >= n_required

//         self.capture = self.pose_reached and (self.still or force)
//         # print("cap", self.calib.nintr, self.capture, n_required, self.tracker.N_pts, self.pose_reached, self.still, force, GlobalStuff.similarity, self.tracker.mean_flow) # rkt capture info

//         # why is flow None so much with small board in the corner?

//         if not self.capture:
//             # print("returning False from update")
//             return False

//         self.calib.keyframes.append(self.tracker.get_calib_pts())

//         # update calibration with all keyframe
//         self.calibrate()

//         # use the updated calibration results for tracking
//         self.tracker.set_intrinsics(self.calib)

//         self.converged = self.pconverged.all()

//         if dry_run:
//             # drop last frame again
//             del self.calib.keyframes[-1] #fixed was "caib" but this is never executed - never dry_run true rkt

//         if self.converged:
//             self.tgt_r = None
//         else:
//             self.set_next_pose()

//         self._update_user_info()

//         return True

/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     _update_user_info                                                      */
/*                                     _update_user_info                                                      */
/*                                     _update_user_info                                                      */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public void _update_user_info()
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        this.user_info_text = "";

        if(this.calib.keyframes.size() < 2)
        {
                        this.user_info_text = "initialization";
        }
        else if( ! this.converged)
        {
            String action = "";
            int axis;
            if(this.tgt_param < 2)
            {
                action = "rotate";
                // do not consider r_z as it does not add any information
                double[] temp = {this.calib.pose_var[0], this.calib.pose_var[1]};
                axis = argmin(temp);
            }
            else
            {
                action = "translate";
                // do not consider t_z
                //FIXME above comment doesn't match the code below
                double[] temp = {this.calib.pose_var[3], this.calib.pose_var[4], this.calib.pose_var[5]};
                axis = argmin(temp) + 3; // find min of t_x, t_y, t_z and add 3 to that index to skip the rotation locations
            }
            String param = this.INTRINSICS[this.tgt_param];
            this.user_info_text = String.format("{%s} {%s} to minimize {%s}", action, this.POSE[axis], param);
            //translate 'ty' to minimize 'k3' this message comes with nearly 100% similarity but it still won't accept and move to next pose
        }
        else
        {
            this.user_info_text = "converged at MSE: {" + this.calib.reperr + "}";
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
    // this adds the guidance board to the camera image to make the new user display
    public void draw(Mat img, boolean mirror) // force users to specify mirror false instead of defaulting
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
/*
img after1 (480, 640, 3) boardPreview project
img after2 (720, 1280, 3)
board warped 2764800 (720, 1280, 3) userGuidance set_next_pose
draw><(720, 1280, 3)><(720, 1280, 3)
draw><(720, 1280, 3)><(720, 1280, 3)
...
done
*/
// assumes both img and board are 3 color channels BGR
        if( ! this.tgt_r.empty())
        {
            // process one row at a time for more cpu efficiency than one element at a time
            byte[] imgBuffRow = new byte[img.cols()*img.channels()]; // temp buffers for efficient access to a row
            byte[] board_warpedBuffRow = new byte[this.board_warped.cols()*this.board_warped.channels()];
            for(int row = 0; row < img.rows(); row++)
            {
                img.get(row, 0, imgBuffRow); // get the row
                this.board_warped.get(row, 0,board_warpedBuffRow);
                for(int col = 0; col < imgBuffRow.length; col++) // process each element of the row
                {
                    // if there is a non-black pixel in the warped board then use it in img
                    if(board_warpedBuffRow[col] != 0)
                    {
                        imgBuffRow[col] = board_warpedBuffRow[col];
                    }
                }
                img.put(row, 0, imgBuffRow);
            }
        }
        if(this.tracker.pose_valid)
        {
            this.tracker.draw_axis(img); // draw axis on the detected board from the camera image
        }
        if(mirror)
        {
            Core.flip(img, img, 1);
        }
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     write                                                                  */
/*                                     write                                                                  */
/*                                     write                                                                  */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public void write()
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        Main.LOGGER.log(Level.SEVERE, "Writing the calibration data");
        Main.LOGGER.log(Level.SEVERE, "calibration_time " + LocalDateTime.now());
        Main.LOGGER.log(Level.SEVERE, "nr_of_frames " + this.calib.keyframes.size());
        Main.LOGGER.log(Level.SEVERE, "image_width " + this.calib.img_size.width);
        Main.LOGGER.log(Level.SEVERE, "image_height " + this.calib.img_size.height);
        Main.LOGGER.log(Level.SEVERE, "board_width " + this.tracker.board_sz.width);
        Main.LOGGER.log(Level.SEVERE, "board_height " + this.tracker.board_sz.height);
        Main.LOGGER.log(Level.SEVERE, "square_size " + this.square_len);
        Main.LOGGER.log(Level.SEVERE, "marker_size ", this.marker_len);
        Main.LOGGER.log(Level.SEVERE, formatFlags(calib.flags));
        Main.LOGGER.log(Level.SEVERE, "fisheye_model " + 0);
        Main.LOGGER.log(Level.SEVERE, "camera_matrix\n" + this.calib.K.dump());
        Main.LOGGER.log(Level.SEVERE, "distortion_coefficients\n" + this.calib.cdist.dump());
        Main.LOGGER.log(Level.SEVERE, "avg_reprojection_error " + this.calib.reperr);
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
    public String formatFlags(int flagsCalibration)
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        HashMap<Integer, String> flags = new HashMap<>(3);
        flags.put(Calib3d.CALIB_FIX_PRINCIPAL_POINT, "+fix_principal_point");
        flags.put(Calib3d.CALIB_ZERO_TANGENT_DIST, "+zero_tangent_dist");
        flags.put(Calib3d.CALIB_USE_LU, "+use_lu");
        StringBuilder flags_str = new StringBuilder("flags: ");
        unknownFlags = flagsCalibration; // initially assume all flags are unknown to the hashmap
        flags.forEach(
            (f, s) -> 
            {
                if((flagsCalibration & f) == f)
                {
                    flags_str.append(s);
                    unknownFlags -= f; // this flag is known so un-mark unknown flags
                }
            }
        );
        flags_str.append("\nflags ");
        flags_str.append(flagsCalibration);
        if(unknownFlags != 0)
        {
            flags_str.append("; unknown flag usage = ");
            flags_str.append(unknownFlags);          
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
// import cv2
// import datetime

// import numpy as np
// import GlobalStuff

// from utils import Calibrator
// from render import BoardPreview
// from posegen import PoseGeneratorDist

// # def debug_jaccard(img, tmp, text): # img overlap (obscured); tmp detected board jumpy
// #     dbg = (img.copy() + tmp * 2) * 127
// #     cv2.putText(dbg, text, (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
// #     cv2.imshow("jaccard", dbg)

// class UserGuidance:
//     AX_NAMES = ("red", "green", "blue")
//     INTRINSICS = ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3")
//     POSE = ("rx", "ry", "rz", "tx", "ty", "tz")

//     SQUARE_LEN_PIX = 12
//     # parameters that are optimized by the same board poses
//     PARAM_GROUPS = [(0, 1, 2, 3), (4, 5, 6, 7, 8)]

//     def __init__(self, tracker, var_terminate=0.1):
//         # get geometry from tracker
//         self.tracker = tracker
//         self.allpts = np.prod(tracker.board_sz - 1)
//         print("userGuidance init1", tracker.board.getSquareLength(), tracker.board.getMarkerLength())
//         self.square_len = tracker.board.getSquareLength()
//         self.marker_len = tracker.board.getMarkerLength()
//         self.SQUARE_LEN_PIX = int(self.square_len)
//         print("userGuidance init2", tracker.board.getSquareLength(), tracker.board.getMarkerLength(), int(self.square_len), sep="><")
//         # userGuidance init1 280.0 182.0
//         # userGuidance init2><280.0><182.0><280
        
//         self.img_size = tracker.img_size

//         self.overlap = np.zeros((self.img_size[1], self.img_size[0]), dtype=np.uint8)

//         # preview image
//         self.board = BoardPreview(self.tracker.board.draw(tuple(tracker.board_sz * self.SQUARE_LEN_PIX)))

//         self.calib = Calibrator(tracker.img_size)
//         self.min_reperr_init = float("inf")

//         # desired pose of board for first frame
//         # translation defined in terms of board dimensions
//         self.board_units = np.array([tracker.board_sz[0], tracker.board_sz[1], tracker.board_sz[0]]) * self.square_len
//         self.board_warped = None

//         self.var_terminate = var_terminate
//         self.pconverged = np.zeros(self.calib.nintr, dtype=np.bool)

//         self.converged = False
//         self.tgt_param = None

//         # actual user guidance
//         self.pose_reached = False
//         self.capture = False
//         self.still = False
//         self.user_info_text = ""

//         self.posegen = PoseGeneratorDist(self.img_size)
//         print("userGuidance init3", self.img_size, tracker.board_sz, self.board_units, self.calib.nintr, self.posegen, sep="><")

//         # set first pose
//         self.set_next_pose()
//         """
//         userGuidance init1 280.0 182.0
//         userGuidance init2><280.0><182.0><280
//         BoardPreview 1><(1680, 2520)><(1680, 2520)
//         BoardPreview 2><(1680, 2520, 3)><(1680, 2520)><(1680, 2520)
//         self.orbital (<generator object gen_bin at 0x0000024B980DAEB0>, <generator object gen_bin at 0x0000024B980DAF20>)
//         userGuidance init3><(1280, 720)><[9 6]><[2520. 1680. 2520.]><9><<posegen.PoseGeneratorDist object at 0x0000024B980AC850>
//         """
//     def calibrate(self):
//         if len(self.calib.keyframes) < 2:
//             # need at least 2 keyframes
//             return

//         pvar_prev = self.calib.varIntrinsics # rkt opencv stddev
//         first = len(self.calib.keyframes) == 2

//         index_of_dispersion = self.calib.calibrate().copy() # local copy so calib.disp_idx isn't changed herein

//         pvar = self.calib.varIntrinsics # rkt opencv stddev

//         if not first:
//             total_var_prev = np.sum(pvar_prev)
//             total_var = np.sum(pvar)

//             if total_var > total_var_prev:
//                 # del self.calib.keyframes[-1]
//                 print("note: total var degraded")
//                 # return

//             # check for convergence
//             rel_pstd = 1 - np.sqrt(pvar) / np.sqrt(pvar_prev)
//             # np.set_printoptions(linewidth=800)
//             # print(np.abs(np.sqrt(var) / vals))
//             # print(rel_pstd[self.tgt_param])
//             #assert rel_pstd[self.tgt_param] >= 0, self.INTRINSICS[self.tgt_param] + " degraded"
//             if rel_pstd[self.tgt_param] < 0:
//                 print(self.INTRINSICS[self.tgt_param] + " degraded")
//             for g in self.PARAM_GROUPS:
//                 if self.tgt_param not in g:
//                     continue

//                 converged = []

//                 for p in g:
//                     # if index_of_dispersion[p] < 0.05:
//                     if rel_pstd[p] > 0 and rel_pstd[p] < self.var_terminate:
//                         if not self.pconverged[p]:
//                             converged.append(self.INTRINSICS[p])
//                             self.pconverged[p] = True

//                 if converged:
//                     print("{} converged".format(converged))

//         index_of_dispersion[self.pconverged] = 0

//         self.tgt_param = index_of_dispersion.argmax()

//         # how well is the requirement 5x more measurements than unknowns is fulfilled
//         # print(self.N_pts*2/self.unknowns, self.N_pts, self.unknowns)
//         # print("keyframes: ", len(self.keyframes))

//         # print("pvar min", self.pose_var.argmin())
//         # print(np.diag(K), cdist)

//     def set_next_pose(self):
//         nk = len(self.calib.keyframes)

//         print("set_next_pose", nk)

//         self.tgt_r, self.tgt_t = self.posegen.get_pose(self.board_units, # rotation and translation of the guidance board
//                                                        nk,
//                                                        self.tgt_param,
//                                                        self.calib.K,
//                                                        self.calib.cdist)

//         print("set_next_pose self.img_size cdist create_maps in", self.img_size, self.calib.cdist)

//         self.board.create_maps(self.calib.K, self.calib.cdist, self.img_size)
//         print("set_next_pose self.img_size cdist create_maps out", self.img_size, self.calib.cdist)
//         self.board_warped = self.board.project(self.tgt_r, self.tgt_t) # make the guidance board warped and right size
//         print("board warped", self.board_warped.size, np.shape(self.board_warped))
//         #board_warped_shape =  # Height Width Channels (720, 1280, 3)

//         GlobalStuff.RTguidance[:3] = self.tgt_r # rkt
//         GlobalStuff.RTguidance[3:] = self.tgt_t
//         GlobalStuff.RTguidance[3] *= int(self.board_warped.shape[1]/self.board.SIZE[0])
//         GlobalStuff.RTguidance[4] *= int(self.board_warped.shape[0]/self.board.SIZE[1])
//         #print(GlobalStuff.RTguidance)
//         #GlobalStuff.RTguidance[5:] =* 
//         #print("board", self.board.SIZE)
//         #height, width, channels = img.shape
//         #print("board", self.board.sz)
//         #print("board_warped", board_warped.shape)
//         #board (640, 480)
//         #board (1280, 720)
//         #board_warped (720, 1280, 3)
//         #        need ratio of original img size to self.sz (apparently a two tuple (x, y))

//     def pose_close_to_tgt(self):
//         jaccard = 0.

//         if not self.tracker.pose_valid:
//             return jaccard

//         if self.tgt_r is None:
//             return jaccard

//         self.overlap[:, :] = self.board_warped[:, :, 1] != 0 # initialize overlap with essentially the warped board
//         # on (1) if not 0 else 0

//         Aa = np.sum(self.overlap) # number of on (1) pixels in the warped_board

//         tmp = self.board.project(self.tracker.rvec, # create projected camera image shadow board
//                                  self.tracker.tvec, 
//                                  shadow=True)
//         Ab = np.sum(tmp) # number of on (1) pixels in the projected camera image shadow board
//         self.overlap *= tmp[:, :] # multiply to get the "AND" to make the overlap on pixels (1)
//         Aab = np.sum(self.overlap)

//         # circumvents instability during initialization and large variance in depth later on
//         # similarity index
//         jaccard = Aab / (Aa + Ab - Aab) # possible RuntimeWarning: invalid value encountered in ulong_scalars
//         if not jaccard: # if something happened and not a number
//             jaccard = 0.

//         GlobalStuff.similarity = jaccard
//         # debug_jaccard(self.overlap, tmp, str(jaccard)[0:5]) # rkt improved this debugging

//         return jaccard

//     def update(self, force=False, dry_run=False):
//         """
//         @return True if a new pose was captured
//         """
//         # print("update b4 if", len(self.calib.keyframes), getattr(self.calib, "N_pts", "no N_pts"), self.calib.reperr, sep=", ")
//         """
//         update b4 if, 0, no N_pts, nan
//         ...
//         update b4 if, 0, no N_pts, nan
//         update b4 if, 0, no N_pts, nan
//         update in if, 0, [], nan
//         set_next_pose 0
//         get_pose
//         orbital_pose
//         """
//         """
//         update b4 if, 0, no N_pts, nan
//         b4 pose_reached False False
//         after if pose_reached False
//         b4 if capture
//         returning False from update
//         update b4 if, 0, no N_pts, nan
//         update in if, 0, [], nan
//         set_next_pose 0
//         get_pose
//         orbital_pose
//         set_next_pose self.img_size cdist create_maps in (1280, 720) [[0. 0. 0. 0. 0.]]
//         create_maps K [[1.13887442e+03 0.00000000e+00 6.39500000e+02]
//         [0.00000000e+00 1.13887442e+03 3.59500000e+02]
//         [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
//         create_maps><(640, 480)><(1280, 720)><(1280, 720)><(3, 3)><(3, 3)><[[0.5        0.         0.        ]
//         [0.         0.66666667 0.        ]
//         [0.         0.         1.        ]]><(3, 3)><[[569.43721073   0.         319.75      ]
//         [  0.         759.24961431 239.66666667]
//         [  0.           0.           1.        ]]
//         set_next_pose self.img_size cdist create_maps out (1280, 720) [[0. 0. 0. 0. 0.]]
//         [ 2.71884603 -0.5408121  -1.1261829 ]
//         board warped 2764800 (720, 1280, 3)
//         set_intrinsics
//         b4 pose_reached False True
//         after if pose_reached False
//         b4 if capture
//         returning False from update
//         update b4 if, 0, 31, 0.4088922706621993
//         update in if, 0, [], 0.4088922706621993
//         b4 pose_reached False True
//         [-2.72374778  0.66082899  1.17744644]
//         after if pose_reached True
//         b4 if capture
//         set_intrinsics
//         set_next_pose 1
//         get_pose
//         pose_planar_fullscreen
//         set_next_pose self.img_size cdist create_maps in (1280, 720) [[0. 0. 0. 0. 0.]]
//         create_maps K [[1.13887442e+03 0.00000000e+00 6.39500000e+02]
//         [0.00000000e+00 1.13887442e+03 3.59500000e+02]
//         [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
//         create_maps><(640, 480)><(1280, 720)><(1280, 720)><(3, 3)><(3, 3)><[[0.5        0.         0.        ]
//         [0.         0.66666667 0.        ]
//         [0.         0.         1.        ]]><(3, 3)><[[569.43721073   0.         319.75      ]
//         [  0.         759.24961431 239.66666667]
//         [  0.           0.           1.        ]]
//         set_next_pose self.img_size cdist create_maps out (1280, 720) [[0. 0. 0. 0. 0.]]
//         [3.14159265 0.         0.        ]
//         board warped 2764800 (720, 1280, 3)
//         update_user_info 1 None [0. 0. 0. 0. 0. 0.]
//         update b4 if, 1, 31, 0.4088922706621993
//         b4 pose_reached False True
//         [-2.71643719  0.66961039  1.18308226]
//         after if pose_reached False
//         b4 if capture
//         returning False from update
//         update b4 if, 1, 31, 0.4088922706621993
//         b4 pose_reached False True
//         [-2.70442102  0.67144888  1.17697462]
//         after if pose_reached False
//         b4 if capture
//         returning False from update
//         """
//         if not self.calib.keyframes and self.tracker.N_pts >= self.allpts // 2:
//             # print("update in if", len(self.calib.keyframes), self.calib.keyframes, self.calib.reperr, sep=", ")
//             # try to estimate intrinsic params from single frame
//             self.calib.calibrate([self.tracker.get_calib_pts()])

//             if not np.isnan(self.calib.K).any() and self.calib.reperr < self.min_reperr_init:
//                 self.set_next_pose()  # update target pose
//                 self.tracker.set_intrinsics(self.calib)
//                 self.min_reperr_init = self.calib.reperr
                
//         # print("b4 pose_reached", force, self.tracker.N_pts > 4)
//         self.pose_reached = force and self.tracker.N_pts > 4

//         if self.pose_close_to_tgt() > GlobalStuff.pose_close_to_tgt_min:
//             self.pose_reached = True
//         # print("after if pose_reached", self.pose_reached)
//         # we need at least 57.5 points after 2 frames # rkt - the calc yields 27 with init nintr of 9, not 57.5
//         # and 15 points per frame from then
//         n_required = ((self.calib.nintr + 2 * 6) * 5 + 3) // (2 * 2)  # integer floor

//         if len(self.calib.keyframes) >= 2:
//             n_required = 6 // 2 * 5 # yup - that's a 15 rkt
//             if self.tracker.N_pts < 17: # rkt
//                 n_required = int(self.tracker.N_pts * GlobalStuff.n_requiredFactor) # rkt

//         self.still = self.tracker.mean_flow < GlobalStuff.mean_flow_max
//         # use all points instead to ensure we have a stable pose

//         self.pose_reached *= self.tracker.N_pts >= n_required

//         self.capture = self.pose_reached and (self.still or force)
//         # print("cap", self.calib.nintr, self.capture, n_required, self.tracker.N_pts, self.pose_reached, self.still, force, GlobalStuff.similarity, self.tracker.mean_flow) # rkt capture info

//         # why is flow None so much with small board in the corner?
//         """
//         len keyframes 1
//         cap 9 0 27 40 0 True False 0.746209106308292 5.2570934
//         len keyframes 1
//         cap 9 True 27 40 1 True False 0.7586640881603586 3.5918686
//         set_intrinsics
//         set_next_pose 1
//         initialization
//         cap 9 0 27 40 0 True False 0.13252604166666668 3.316761
//         initialization
//         cap 9 0 27 40 0 False False 0.13301866319444444 7.0396323
//         """

//         """
//         cap 9 False 12 14 1 False False 0.8785608583055864 None
//         translate 'ty' to minimize 'k3'
//         cap 9 False 9 11 1 False False 0.8886401894226679 None
//         translate 'ty' to minimize 'k3'
//         cap 9 False 10 12 1 False False 0.9056586322351505 None
//         translate 'ty' to minimize 'k3'
//         cap 9 True 10 12 1 True False 0.9210959613712467 5.8348656
//         c: Users RKT frc FRC2023 Code pose_calib utils.py:296: VisibleDeprecationWarning: Creating an ndarray from ragged nested sequences (which is a list-or-tuple of lists-or-tuples-or ndarrays with different lengths or shapes) is deprecated. If you meant to do this, you must specify 'dtype=object' when creating the ndarray.  
//         res = cv2.calibrateCamera(np.array(pts3d), np.array(pts2d), img_size, K, None, flags=flags)
//         k3 degraded
//         set_intrinsics
//         set_next_pose 8
//         translate 'ty' to minimize 'cx'
//         cap 9 0 14 16 0 False False 0.0 None
//         """
//         """
//         small image in the corner - extreme pain trying to get it right
//         looks like mean flow none was the worst problem to try to get rid of
//         along with the number of points was hard to get to the required number
//         since the guidance was only part of a board (forget how much - half?)
//         seems like it's usually k3 that is the worst problem at the edge or corner
//         summary - in the corner hard to get the few points at a distance aligned two frames in a row for k3
//         translate 'ty' to minimize 'k3'
//         cap 9 False 15 20 1 False False 0.8198723490352604 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 7 0 False False 0.8179072037722642 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 11 0 False False 0.8109320637245926 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 13 0 False False 0.813168106604188 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 5 0 False False 0.817684394834756 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 13 0 False False 0.8020854220974534 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 11 0 False False 0.7948035857444793 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 13 0 False False 0.7918173925625814 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 14 0 False False 0.7993737826079041 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 10 0 False False 0.795853153709285 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 10 0 False False 0.7890396526983089 None
//         translate 'ty' to minimize 'k3'
//         cap 9 False 15 15 1 False False 0.7978269938872866 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 13 0 False False 0.7981352992194276 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 9 0 False False 0.8112271970431633 None
//         translate 'ty' to minimize 'k3'
//         cap 9 False 15 16 1 False False 0.8129150278971919 None
//         translate 'ty' to minimize 'k3'
//         cap 9 False 15 16 1 False False 0.8177793227098923 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 12 0 False False 0.8170247782755783 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 12 0 False False 0.8027280167318359 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 6 0 False False 0.7883326692505523 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 10 0 False False 0.7995776750282156 None
//         translate 'ty' to minimize 'k3'
//         cap 9 0 15 14 0 False False 0.7787208628927176 None
//         translate 'ty' to minimize 'k3'
//         cap 9 False 15 20 1 False False 0.7590041523354659 None
//         translate 'ty' to minimize 'k3'
//         cap 9 True 15 20 1 True False 0.7557775166470818 3.4172153
//         ['k1', 'p1', 'p2'] converged
//         set_intrinsics
//         set_next_pose 4
//         """
//         # print("b4 if capture")
//         if not self.capture:
//             # print("returning False from update")
//             return False

//         self.calib.keyframes.append(self.tracker.get_calib_pts())

//         # update calibration with all keyframe
//         self.calibrate()

//         # use the updated calibration results for tracking
//         self.tracker.set_intrinsics(self.calib)

//         self.converged = self.pconverged.all()

//         if dry_run:
//             # drop last frame again
//             del self.calib.keyframes[-1] #fixed was "caib" but this is never executed - never dry_run true rkt

//         if self.converged:
//             self.tgt_r = None
//         else:
//             self.set_next_pose()

//         self._update_user_info()

//         return True

//     def _update_user_info(self):
//         print("update_user_info", len(self.calib.keyframes), self.tgt_param, self.calib.pose_var)
//         self.user_info_text = ""

//         if len(self.calib.keyframes) < 2:
//             self.user_info_text = "initialization"
//         elif not self.converged:
//             action = None
//             axis = None
//             if self.tgt_param < 2:
//                 action = "rotate"
//                 # do not consider r_z as it does not add any information
//                 axis = self.calib.pose_var[:2].argmin()
//             else:
//                 action = "translate"
//                 # do not consider t_z
//                 axis = self.calib.pose_var[3:6].argmin() + 3

//             param = self.INTRINSICS[self.tgt_param]
//             self.user_info_text = "{} '{}' to minimize '{}'".format(action, self.POSE[axis], param)
//             #translate 'ty' to minimize 'k3' this message comes with nearly 100% similarity but it still won't accept and move to next pose
//         else:
//             self.user_info_text = "converged at MSE: {}".format(self.calib.reperr)

//         if self.pose_reached and not self.still:
//             self.user_info_text += "\nhold camera steady"

//     def draw(self, img, mirror=False):
//         if self.tgt_r is not None:
//             img[self.board_warped != 0] = self.board_warped[self.board_warped != 0] # add guidance board to camera img

//         if self.tracker.pose_valid:
//             self.tracker.draw_axis(img) # draw on the detected board from the camera image

//         if mirror:
//             cv2.flip(img, 1, img)

//     def seed(self, imgs):
//         for img in imgs:
//             self.tracker.detect(img)
//             self.update(force=True)

//     def write(self, outfile):
//         flags = [(cv2.CALIB_FIX_PRINCIPAL_POINT, "+fix_principal_point"),
//                  (cv2.CALIB_ZERO_TANGENT_DIST, "+zero_tangent_dist"),
//                  (cv2.CALIB_USE_LU, "+use_lu")]

//         fs = cv2.FileStorage(outfile, cv2.FILE_STORAGE_WRITE)
//         fs.write("calibration_time", datetime.datetime.now().strftime("%c"))
//         fs.write("nr_of_frames", len(self.calib.keyframes))
//         fs.write("image_width", self.calib.img_size[0])
//         fs.write("image_height", self.calib.img_size[1])
//         fs.write("board_width", self.tracker.board_sz[0])
//         fs.write("board_height", self.tracker.board_sz[1])
//         fs.write("square_size", self.square_len)
//         fs.write("marker_size", self.marker_len)

//         flags_str = " ".join([s for f, s in flags if self.calib.flags & f])
//         fs.writeComment("flags: " + flags_str)

//         fs.write("flags", self.calib.flags)
//         fs.write("fisheye_model", 0)
//         fs.write("camera_matrix", self.calib.K)
//         fs.write("distortion_coefficients", self.calib.cdist)
//         fs.write("avg_reprojection_error", self.calib.reperr)
//         fs.release()
