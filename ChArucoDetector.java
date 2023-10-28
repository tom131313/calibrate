package calibrator;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.objdetect.CharucoBoard;
import org.opencv.objdetect.CharucoDetector;
import org.opencv.objdetect.CharucoParameters;
import org.opencv.objdetect.DetectorParameters;
import org.opencv.objdetect.Dictionary;
import org.opencv.objdetect.Objdetect;
import org.opencv.objdetect.RefineParameters;

/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     ChArucoDetector class                                                  */
/*                                     ChArucoDetector class                                                  */
/*                                     ChArucoDetector class                                                  */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
public class ChArucoDetector {
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     ChArucoDetector constructor                                            */
/*                                     ChArucoDetector constructor                                            */
/*                                     ChArucoDetector constructor                                            */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}
 
    // configuration
    private Size board_sz = new Size(Cfg.board_x, Cfg.board_y);
    private double square_len = Cfg.square_len;
    private double marker_len = Cfg.marker_len;
    Size img_size = new Size(Cfg.image_width, Cfg.image_height);

    // per frame data

    // p3d is the object coordinates of the perfect undistorted ChArUco Board corners that the camera is pointing at.
    // In this case the board is flat at its Z origin (say, the wall upon which it is mounted) so the Z coordinate is always 0.
    // p2d is the coordinates of the corresponding board corners as they are located in the camera image,
    // distorted by perspective (pose) and camera intrinsic parameters and camera distortion.
    private final Mat p3d = new Mat(); // 3 dimensional currentObjectPoints, the physical target ChArUco Board
    private final Mat p2d = new Mat(); // 2 dimensional currentImagePoints, the likely distorted board on the flat camera sensor frame posed relative to the target
    private int N_pts = 0;
    private boolean pose_valid = false;
    // private Mat raw_img = null; // not used

    /// Charuco Board
    private final Dictionary dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50);
    private final Size boardImageSize = new Size(Cfg.board_x*Cfg.square_len, Cfg.board_y*Cfg.square_len);
    final Mat boardImage = new Mat();
    private final CharucoBoard board = new CharucoBoard(this.board_sz, Cfg.square_len, Cfg.marker_len, this.dictionary);
    private CharucoDetector detector; // the OpenCV detector spelled almost the same - fooled me too many times!!!!!

    private Mat rvec = new Mat();
    private Mat tvec = new Mat();

    private boolean intrinsic_valid = false;
    private Mat K;
    private Mat cdist;

    private final Mat ccorners = new Mat(); // currentCharucoCorners
    private final Mat cids = new Mat(); // currentCharucoIds

    // optical flow calculation
    private Mat last_ccorners = new Mat(); // previous ChArUcoBoard corners
    private Mat last_cids = new Mat(); // previous ChArUcoBoard ids
    private double mean_flow = Double.MAX_VALUE; // mean flow of the same corners that are detected in consecutive frames (relaxed from original)

    // getters
    int N_pts(){return N_pts;}
    Size board_sz(){return board_sz;}
    boolean pose_valid(){return this.pose_valid;}
    Mat rvec(){return rvec;}
    Mat tvec(){return tvec;}
    double mean_flow(){return this.mean_flow;}

    public ChArucoDetector()
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        /// create board
        // this.board.setLegacyPattern(Cfg.legacyPattern); //FIXME shouldn't use - remove when we match the original and we can go our own way
        this.board.generateImage(this.boardImageSize, this.boardImage);

        if(Cfg.writeBoard)
        {
            // write ChArUco Board to file for print to use for calibration
            final MatOfInt writeBoardParams = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, 100); // pair-wise; param1, value1, ...
            final String file = "ChArUcoBoard.jpg";
            Imgcodecs.imwrite(
            file,
            this.boardImage,
            writeBoardParams);
            Main.LOGGER.log(Level.SEVERE, "ChArUcoBoard to be printed is in file ChArUcoBoard.jpg");
        }
        /// end create board

        /// board detector
        final DetectorParameters detectParams = new DetectorParameters();
        final RefineParameters refineParams = new RefineParameters();
        final CharucoParameters charucoParams = new CharucoParameters();

        charucoParams.set_minMarkers(Cfg.pt_min_markers);
        charucoParams.set_tryRefineMarkers(true);
        // charucoParams.set_cameraMatrix();
        // charucoParams.set_distCoeffs();
        detectParams.set_cornerRefinementMaxIterations(2000);
        detectParams.set_cornerRefinementMethod(Objdetect.CORNER_REFINE_CONTOUR); // 2
        detector = new CharucoDetector(this.board, charucoParams, detectParams, refineParams);
        
        Main.LOGGER.log(Level.CONFIG, "" + detector.getCharucoParameters().get_minMarkers()); // 2 default
        Main.LOGGER.log(Level.CONFIG, "" + detector.getCharucoParameters().get_tryRefineMarkers()); // false default
        Main.LOGGER.log(Level.CONFIG, "" + detector.getDetectorParameters().get_cornerRefinementMaxIterations()); // 30 default
        Main.LOGGER.log(Level.CONFIG, "" + detector.getDetectorParameters().get_cornerRefinementMethod()); // 0 default
        Main.LOGGER.log(Level.CONFIG, "" + detector.getRefineParameters().get_checkAllOrders()); // true default
        Main.LOGGER.log(Level.CONFIG, "" + detector.getRefineParameters().get_errorCorrectionRate()); // 3.0 default
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     set_intrinsics                                                         */
/*                                     set_intrinsics                                                         */
/*                                     set_intrinsics                                                         */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public void set_intrinsics(Calibrator calib)
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        this.intrinsic_valid = true;
        this.K = calib.K();
        this.cdist = calib.cdist();
        Main.Kcsv(Id.__LINE__(), this.K);
        //Main.LOGGER.log(Level.WARNING, "class private K\n" + K.dump());
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     draw_axis                                                              */
/*                                     draw_axis                                                              */
/*                                     draw_axis                                                              */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public void draw_axis(Mat img)
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        Calib3d.drawFrameAxes(img, this.K, this.cdist, this.rvec, this.tvec, (float)this.square_len);
    }   
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     detect_pts                                                             */
/*                                     detect_pts                                                             */
/*                                     detect_pts                                                             */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public void detect_pts(Mat img) throws Exception
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        final List<Mat> markerCorners = new ArrayList();
        final Mat markerIds = new Mat();

        try
        {
            detector.detectBoard
                ( img, this.ccorners, this.cids, markerCorners, markerIds );
        }
        catch(Exception e) {Main.LOGGER.log(Level.SEVERE, img + " " + this.ccorners + " " + this.cids + "\n" + e);}

        this.N_pts = 0;
        this.mean_flow = Double.MAX_VALUE;

        if( ! this.cids.empty() && this.cids.rows() > 0) // double check since there was some unknown failure to get the N_pts set right
        {
            this.N_pts = this.cids.rows();
        }

        //Main.LOGGER.log(Level.WARNING, "N_pts " + this.N_pts);
    
        if(this.N_pts == 0) // maybe use the min N_pts from Cfg
        {
            // for optical flow calculation
            this.last_ccorners.release();
            this.last_cids.release();
            this.last_ccorners = new Mat();
            this.last_cids = new Mat();
            return;
        }

        //Main.LOGGER.log(Level.WARNING, "detected ccorners\n" + this.ccorners.dump());
        //Main.LOGGER.log(Level.WARNING, "detected cids\n" + this.cids.dump());
        
        // reformat the Mat to a List<Mat> for matchImagePoints
        final List<Mat> ccornersList = new ArrayList<>();
        for(int i = 0; i < this.ccorners.total(); i++) {
          ccornersList.add(this.ccorners.row(i));
        }

        // display the detected cids on the board (debugging)
        // Objdetect.drawDetectedCornersCharuco(img, ccorners, cids);

        board.matchImagePoints(ccornersList, this.cids,this.p3d, this.p2d); // p2d same data as ccornersList
        // oddly this method returns 3 channels instead of 2 for imgPoints and there isn't much to do about it and it works in solvePnP
        // after copying to MatOfPoint2f. A waste of cpu and memory.

        //Main.LOGGER.log(Level.WARNING, "p3d\n" + this.p3d.dump()); // data okay here
        //Main.LOGGER.log(Level.WARNING, "p2d\n" + this.p2d.dump()); // data okay here

        if(this.p3d.empty() || this.p2d.empty()) throw new Exception("p3d or p2d empty"); // shouldn't happen

        // compute mean flow of the image for the check for stillness elsewhere
        computeMeanFlow(); // the rkt way - relaxed criteria from original

        // abandoned the original algorithm in favor of more relaxed version - rkt
        // // mean flow if the same corners are detected in consecutive frames
        // // relaxed criterion to same number of corners and not necessarily the same corners - rkt
        // if(this.last_cids.total() != this.cids.total())
        // {
        //     this.ccorners.copyTo(this.last_ccorners);
        //     this.cids.copyTo(this.last_cids);
        //     return;
        // }
        // for(int row = 0; row < this.last_cids.rows(); row++)
        // for(int col = 0; col < this.last_cids.cols(); col++)
        // {
        //     if(this.last_cids.get(row, col)[0] != this.cids.get(row, col)[0])
        //     {
        //         this.ccorners.copyTo(this.last_ccorners);
        //         this.cids.copyTo(this.last_cids);
        //         return;                
        //     }
        // }

        // // calculate mean flow.
        // // Not sure what original axis=1 norm is. Below is 2 axes which is better, I think
        // Mat diff = new Mat();
        // Core.subtract(this.last_ccorners, this.ccorners, diff);
        // // //Main.LOGGER.log(Level.WARNING, "diffpts " + diff + "\n" + diff.dump());

        // Mat normMat = new Mat(diff.rows(), diff.cols(), CvType.CV_64FC1);

        // for(int row =0; row < diff.rows(); row++)
        // for(int col = 0; col < diff.cols(); col++)
        // {
        //     float[] point = new float[2]; // get the 2 channels of data x in 0 and y in 1
        //     diff.get(row, col, point);
        //     double norm = Math.sqrt(Math.pow(point[0], 2) + Math.pow(point[1], 2)); // L2 norm (Frobenious)
        //     normMat.put(row, col, norm);
        // }
        // // //Main.LOGGER.log(Level.WARNING, "normMat\n" + normMat.dump());

        // this.mean_flow = Core.mean(normMat).val[0];

        //Main.LOGGER.log(Level.WARNING, "mean_flow " + this.mean_flow);
        this.ccorners.copyTo(this.last_ccorners);
        this.cids.copyTo(this.last_cids);
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     computeMeanFlow                                                        */
/*                                     computeMeanFlow                                                        */
/*                                     computeMeanFlow                                                        */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public void computeMeanFlow()
    {
        // cids: int 1 col, 1 channel; ccorners: float 1 col, 2 channels (x, y)

        this.mean_flow = Double.MAX_VALUE;

        if(last_cids.rows() >= 1 && cids.rows() >= 1) // handles the first time and other cases
        {
            // get all the last cids and ccorners and all the current cids and ccorners in arrays.
            // do all the computations in the arrays.
            float[] last_ccornersArray = new float[last_ccorners.rows()*last_ccorners.channels()];
            int[] last_cidsArray = new int[last_cids.rows()]; // assume 1 col 1 channel
            float[] ccornersArray = new float[ccorners.rows()*ccorners.channels()];
            int[] cidsArray = new int[cids.rows()];
            
            this.last_ccorners.get(0,0, last_ccornersArray);
            this.last_cids.get(0, 0, last_cidsArray);
            this.ccorners.get(0, 0, ccornersArray);
            this.cids.get(0, 0,cidsArray );
            
            // assume the cids and last_cids are in order ascending

            // mean flow of only the corners in common that are detected in consecutive frames
            // relaxed criterion from original need of exactly the same corners in successive frames - rkt
            if(this.last_cids.total() <= 0 || this.cids.total() <= 0)
            {
                return; // one of them has no corners to compare to
            }

            this.mean_flow = 0.; // starting at 0 for a summation process but that will change to flow or max value
            double countMatchingCids = 0;
            int indexCurrent = 0;
            int indexLast = 0;

            // merge/match last_cids and cids; assume they are already sorted ascending
            while(indexCurrent< this.cids.rows() && indexLast< this.last_cids.rows())
            {
                if(cidsArray[indexCurrent] == last_cidsArray[indexLast])
                {// great we have matching cids to compare then bump both to stay in sync
                    double diffX = ccornersArray[2*indexCurrent  ] - last_ccornersArray[2*indexLast  ]; // current - last
                    double diffY = ccornersArray[2*indexCurrent+1] - last_ccornersArray[2*indexLast+1]; // current - last

                    this.mean_flow += Math.sqrt(Math.pow(diffX, 2) + Math.pow(diffY, 2)); // sum the L2 norm (Frobenious)
                    countMatchingCids++;
                    indexCurrent++;
                    indexLast++;
                    continue;
                }
                else
                if(cidsArray[indexCurrent] < last_cidsArray[indexLast])
                {// missing corresponding last so bump current to catch up to last
                    indexCurrent++;
                    continue;
                }
                else
                if(cidsArray[indexCurrent] > last_cidsArray[indexLast])
                {// missing corresponding current so bump last to catch up to current
                    indexLast++;
                    continue;
                }
                Main.LOGGER.log(Level.SEVERE, "Can't be here; 3-way 'if' failed");
            }

            if(countMatchingCids > 0           // allow some missing cids from either list by a Jaccard index similarity
                && (countMatchingCids/(this.last_cids.rows() + this.cids.rows() - countMatchingCids)) > Cfg.matchStillCidsMin)
            {
                this.mean_flow /= countMatchingCids; // mean of the sum of the norms
            }
            else
            {
                this.mean_flow = Double.MAX_VALUE; // avoided the divide by zero and return flow is big (no need to be infinite)
            }
        }
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     detect                                                                 */
/*                                     detect                                                                 */
/*                                     detect                                                                 */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public void detect(Mat img) throws Exception
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        // raw_img never used - not converted
        this.detect_pts(img);

        if(this.intrinsic_valid)
        {
            this.update_pose();
        }
    } 
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                    get_pts3d                                                               */
/*                                    get_pts3d                                                               */
/*                                    get_pts3d                                                               */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public Mat get_pts3d()
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        return this.p3d;
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     get_calib_pts                                                          */
/*                                     get_calib_pts                                                          */
/*                                     get_calib_pts                                                          */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public keyframe get_calib_pts()
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        return new keyframe(this.ccorners.clone(), this.get_pts3d().clone());
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     update_pose                                                            */
/*                                     update_pose                                                            */
/*                                     update_pose                                                            */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public void update_pose() throws Exception
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        if(this.N_pts < Cfg.minCorners) // original had 4; solvePnp wants 6 sometimes, and UserGuidance wants many more
        {
            Main.LOGGER.log(Level.SEVERE, "too few corners ", this.N_pts);
            this.pose_valid = false;
            return;
        }

        MatOfPoint3f p3dReTyped = new MatOfPoint3f(this.p3d);
        MatOfPoint2f p2dReTyped = new MatOfPoint2f(this.p2d);
        MatOfDouble distReTyped = new MatOfDouble(this.cdist);

        //Main.LOGGER.log(Level.WARNING, "p3d\n" + p3dReTyped.dump());
        //Main.LOGGER.log(Level.WARNING, "p2d\n" + p2dReTyped.dump());
        
        Mat rvec = new Mat(); // neither previous pose nor guidance board pose helped the solvePnP (made pose estimate worse)
        Mat tvec = new Mat(); // so don't give solvePnP a starting pose estimate

        this.pose_valid = Calib3d.solvePnPRansac(
            p3dReTyped, p2dReTyped, this.K, distReTyped, rvec, tvec, false);

        if( ! this.pose_valid)
        {
            //Main.LOGGER.log(Level.WARNING, "pose not valid");
            return;            
        }

        Calib3d.solvePnPRefineVVS(p3dReTyped, p2dReTyped, this.K, distReTyped, rvec, tvec);
  
        //FIXME negating "x" makes the shadow for jaccard the right orientation for some unknown reason! Python doesn't need this.
        Core.multiply(rvec, new Scalar(-1., 1., 1.), rvec);

        this.rvec = rvec.t(); // t() like ravel(), solvePnp returns r and t as Mat(3, 1, )
        this.tvec = tvec.t(); // and the rest of the program uses Mat(1, 3, )

        //Main.LOGGER.log(Level.WARNING, "out rvec\n" + this.rvec.dump());
        //Main.LOGGER.log(Level.WARNING, "out tvec\n" + this.tvec.dump());
    }
}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     End ChArucoDetector class                                              */
/*                                     End ChArucoDetector class                                              */
/*                                     End ChArucoDetector class                                              */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*
Python & Java
pose gen orbital_pose
Pose Gen Rod Rf r returned [2.7188 -0.5408 -1.1262]
Pose Gen Rod Rf t returned><[-595.8323 1258.2399 4627.8323]

Python
ret = estimatePoseCharucoBoard(self.ccorners, self.cids, self.board, self.K, self.cdist)
update_pose rvec [[-2.6629] [0.6387] [1.1686]]
update_pose tvec [[1930.3167] [1424.4072] [7285.3986]]

Java
solvePnP
*/
// Parking lot

// https://stackoverflow.com/questions/12794876/how-to-verify-the-correctness-of-calibration-of-a-webcam/12821056#12821056
// The correct way to check calibration accuracy is to use the reprojection error provided by OpenCV. I'm not sure why this
// wasn't mentioned anywhere in the answer or comments, you don't need to calculate this by hand - it's the return value of
// calibrateCamera. In Python it's the first return value (followed by the camera matrix, etc).
// The reprojection error is the RMS error between where the points would be projected using the intrinsic coefficients and
// where they are in the real image. Typically you should expect an RMS error of less than 0.5px - I can routinely get around 
// 0.1px with machine vision cameras.
