package calibrator;

// import static calibrator.ArrayUtils.brief;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
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

public class ChArucoDetector {
    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}
 
    // configuration
    Size board_sz = new Size(Cfg.board_x, Cfg.board_y);
    double square_len = Cfg.square_len;
    double marker_len = Cfg.marker_len;
    Size img_size = new Size(Cfg.image_width, Cfg.image_height);

    // per frame data
    final Mat p3d = new Mat(); // currentObjectPoints
    final Mat p2d = new Mat(); // currentImagePoints
    int N_pts = 0;
    boolean pose_valid = false;
    Mat raw_img = new Mat();

    /// Charuco Board
    final Dictionary dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50);

    final Size boardImageSize = new Size(Cfg.board_x*Cfg.square_len, Cfg.board_y*Cfg.square_len);

    public final Mat boardImage = new Mat();
       
    public final CharucoBoard board = new CharucoBoard(board_sz, Cfg.square_len, Cfg.marker_len, dictionary);

    CharucoDetector detector; // the OpenCV detector spelled almost the same - fooled me too many times!!!!!

    Mat rvec = new Mat();
    Mat tvec = new Mat();
    Mat tgt_r = new Mat(); // from UserGuidance set_next_pose/get_pose cheat 'cuz bad
    Mat tgt_t = new Mat(); // organization with the detector twisted in with the pose

    boolean intrinsic_valid = false;
    Mat K;
    Mat cdist;

    final Mat ccorners = new Mat(); /*currentCharucoCorners */
    final Mat cids = new Mat(); /*currentCharucoIds */

    // optical flow calculation
    Mat last_ccorners = new Mat();
    Mat last_cids = new Mat();
    // mean flow if same corners are detected in consecutive frames - not fully implemented
    // changed to same number of corners and not necessarily the same corners
    double mean_flow = Double.MAX_VALUE;

    public ChArucoDetector()
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        /// create board
        board.setLegacyPattern(Cfg.legacyPattern);
        board.generateImage(boardImageSize, boardImage);

        boolean writeBoard = true;
        if(writeBoard)
        {
            // write ChArUco Board to file for print to use for calibration
            final MatOfInt writeBoardParams = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, 100); // pair-wise; param1, value1, ...
            final String file = "ChArUcoBoard.jpg";
            Imgcodecs.imwrite(
            file,
            boardImage,
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
        detector = new CharucoDetector(board, charucoParams, detectParams, refineParams);
        
        Main.LOGGER.log(Level.CONFIG, "" + detector.getCharucoParameters().get_minMarkers()); // 2 default
        Main.LOGGER.log(Level.CONFIG, "" + detector.getCharucoParameters().get_tryRefineMarkers()); // false default
        Main.LOGGER.log(Level.CONFIG, "" + detector.getDetectorParameters().get_cornerRefinementMaxIterations()); // 30 default
        Main.LOGGER.log(Level.CONFIG, "" + detector.getDetectorParameters().get_cornerRefinementMethod()); // 0 default
        Main.LOGGER.log(Level.CONFIG, "" + detector.getRefineParameters().get_checkAllOrders()); // true default
        Main.LOGGER.log(Level.CONFIG, "" + detector.getRefineParameters().get_errorCorrectionRate()); // 3.0 default
    }

    public void set_intrinsics(Calibrator calib)
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        this.intrinsic_valid = true;
        this.K = calib.K;
        this.cdist = calib.cdist;
    }

    public void draw_axis(Mat img)
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        Calib3d.drawFrameAxes(img, this.K, this.cdist, this.rvec, this.tvec, (float)this.square_len);
    }   

    public void detect_pts(Mat img) throws Exception
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        final List<Mat> markerCorners = new ArrayList();
        final Mat markerIds = new Mat();
        this.N_pts = 0;
        this.mean_flow = Double.MAX_VALUE;

        try
        {
        detector.detectBoard
            ( img, this.ccorners, this.cids, markerCorners, markerIds );
        }
        catch(Exception e) {Main.LOGGER.log(Level.SEVERE, img + " " + this.ccorners + " " + this.cids + "\n" + e);}

        this.N_pts = (int)this.cids.total();
        Main.LOGGER.log(Level.WARNING, "N_pts " + this.N_pts);
    
        if(this.N_pts == 0) // maybe use the min N_pts
        {
            // for optical flow calculation
            last_ccorners = new Mat();
            last_cids = new Mat();
            return;
        }

        // Main.LOGGER.log(Level.WARNING, "detected ccorners\n" + this.ccorners.dump());
        // Main.LOGGER.log(Level.WARNING, "detected cids\n" + this.cids.dump());
        
        // reformat the Mat to a List<Mat> for matchImagePoints
        final List<Mat> ccornersList = new ArrayList<>();
        for(int i = 0; i < this.ccorners.total(); i++) {
          ccornersList.add(this.ccorners.row(i));
        }

        // Objdetect.drawDetectedCornersCharuco(img, ccorners, cids);

        board.matchImagePoints(ccornersList, this.cids,this.p3d, this.p2d); // p2d same data as ccornersList
        // oddly this method returns 3 channels instead of 2 for imgPoints and there isn't much to do about it and it works in solvePnP
        // after copying to MatOfPoint2f. A waste of cpu and memory.

        // Main.LOGGER.log(Level.WARNING, "p3d" + this.p3d.dump()); // okay here
        // Main.LOGGER.log(Level.WARNING, "p2d" + this.p2d.dump()); // okay here
/*
00098 2023-10-16 18:35:00.839 WARNING [ calibrator.ChArucoDetector detect_pts] method entered  . . . . . . . . . . . . . . . . . . . . . . . . 
00098 2023-10-16 18:35:00.880 WARNING [ calibrator.ChArucoDetector detect_pts] N_pts 5 
00098 2023-10-16 18:35:00.880 WARNING [ calibrator.ChArucoDetector detect_pts] detected ccorners
 
00098 2023-10-16 18:35:00.881 WARNING [ calibrator.ChArucoDetector detect_pts] detected cids
 
00098 2023-10-16 18:35:00.881 WARNING [ calibrator.ChArucoDetector detect_pts] p3d
[560, 1120, 0;
 840, 1120, 0;
 280, 1400, 0;
 560, 1400, 0;
 840, 1400, 0] 
00098 2023-10-16 18:35:00.883 WARNING [ calibrator.ChArucoDetector detect_pts] p2d
[633.42603, 485.72961;
 684.49097, 465.53342;
 604.17853, 559.16071;
 653.65509, 541.08752;
 705.43335, 522.56055] 
 */
        if(this.p3d.empty() || this.p2d.empty()) throw new Exception("p3d or p2d empty"); // shouldn't happen

        // check for still image
        // mean flow if the same corners are detected in consecutive frames
        // relaxed criterion to same number of corners and not necessarily the same corners - rkt
        if(this.last_cids.total() != this.cids.total())
        {
            this.ccorners.copyTo(this.last_ccorners);
            this.cids.copyTo(this.last_cids);
            return;
        }
        for(int row = 0; row < this.last_cids.rows(); row++)
        for(int col = 0; col < this.last_cids.cols(); col++)
        {
            if(this.last_cids.get(row, col)[0] != this.cids.get(row, col)[0])
            {
                this.ccorners.copyTo(this.last_ccorners);
                this.cids.copyTo(this.last_cids);
                return;                
            }
        }

        // calculate mean flow.
        // Not sure what original axis=1 norm is. Below is 2 axes which is better, I think
        Mat diff = new Mat();
        Core.subtract(this.last_ccorners, this.ccorners, diff);
        // Main.LOGGER.log(Level.WARNING, "diffpts " + diff + "\n" + diff.dump());

        Mat normMat = new Mat(diff.rows(), diff.cols(), CvType.CV_64FC1);

        for(int row =0; row < diff.rows(); row++)
        for(int col = 0; col < diff.cols(); col++)
        {
            float[] point = new float[2]; // get the 2 channels of data x in 0 and y in 1
            diff.get(row, col, point);
            double norm = Math.sqrt(Math.pow(point[0], 2) + Math.pow(point[1], 2)); // L2 norm (Frobenious)
            normMat.put(row, col, norm);
        }
        // Main.LOGGER.log(Level.WARNING, "normMat\n" + normMat.dump());

        this.mean_flow = Core.mean(normMat).val[0];
        Main.LOGGER.log(Level.WARNING, "mean_flow " + this.mean_flow);
        this.ccorners.copyTo(this.last_ccorners);
        this.cids.copyTo(this.last_cids);
    }
  
    public void detect(Mat img) throws Exception
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        this.detect_pts(img);

        if(this.intrinsic_valid)
        {
            this.update_pose();
        }
    }
    
    public Mat get_pts3d()
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        return this.p3d.clone();
    }

    public keyframe get_calib_pts()
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        return new keyframe(this.ccorners.clone(), this.get_pts3d());
    }

    public void update_pose() throws Exception
    {
        Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        if(this.N_pts < Cfg.minCorners) // original had 4; solvePnp wants 6 sometimes, and UserGuidance wants many more
        {
            Main.LOGGER.log(Level.SEVERE, "too few corners ", this.N_pts);
            this.pose_valid = false;
            return;
        }

        MatOfPoint3f p3dReTyped = new MatOfPoint3f(this.p3d);
        MatOfPoint2f p2dReTyped = new MatOfPoint2f(this.p2d);
        MatOfDouble distReTyped = new MatOfDouble(this.cdist);
        // Main.LOGGER.log(Level.WARNING, "p3d\n" + p3dReTyped.dump());
        // Main.LOGGER.log(Level.WARNING, "p2d\n" + p2dReTyped.dump());

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        this.tgt_r.copyTo(rvec); // GuidanceBoard pose, put this here for us to try as a good initial
        this.tgt_t.copyTo(tvec); // guess of where the camera img is supposed to be - try it - rkt - doesn't seem to help

        Main.LOGGER.log(Level.WARNING, "K\n" + this.K.dump());
        Main.LOGGER.log(Level.WARNING, "distReTyped " + distReTyped.dump());
        Main.LOGGER.log(Level.WARNING, " in rvec\n" + rvec.dump());
        Main.LOGGER.log(Level.WARNING, " in tvec\n" + tvec.dump());
        this.pose_valid = Calib3d.solvePnP(p3dReTyped, p2dReTyped, this.K, distReTyped, rvec, tvec);

        Core.multiply(rvec, new Scalar(-1., -1., -1.), rvec); //FIXME this makes the shadow for jaccard the right orientation for some reason!
        //FIXME what's wrong?????
        Main.LOGGER.log(Level.WARNING, "out rvec\n" + rvec.t().dump());
        Main.LOGGER.log(Level.WARNING, "out tvec\n" + tvec.t().dump());

// ret = estimatePoseCharucoBoard(self.ccorners, self.cids, self.board, self.K, self.cdist)
// self.pose_valid, rvec, tvec = ret

        if( ! this.pose_valid)
        {
            Main.LOGGER.log(Level.WARNING, "pose not valid");
            return;            
        }
        this.rvec = rvec.t(); // t() like ravel(), solvePnp returns r and t as Mat(3, 1, )
        this.tvec = tvec.t(); // and the rest of the program uses Mat(1, 3, )
    }
}
// https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
// https://docs.opencv.org/4.8.0/d5/dae/tutorial_aruco_detection.html
// C:\Users\RKT\frc\FRC2023\opencv-4.x\modules\objdetect\test\test_aruco_utils.cpp
// C:\Users\RKT\frc\FRC2023\opencv-4.x\modules\objdetect\test\test_aruco_utils.hpp
// C:\Users\RKT\frc\FRC2023\opencv-4.x\modules\objdetect\test\test_charucodetection.cpp

//////////////////////////////
//////////////////////////////
//////////////////////////////
// bool getCharucoBoardPose(InputArray charucoCorners, InputArray charucoIds,  const aruco::CharucoBoard &board,
// InputArray cameraMatrix, InputArray distCoeffs, InputOutputArray rvec, InputOutputArray tvec,
// bool useExtrinsicGuess) {
//     CV_Assert((charucoCorners.getMat().total() == charucoIds.getMat().total()));
//     if(charucoIds.getMat().total() < 4) return false; // need, at least, 4 corners

//     std::vector<Point3f> objPoints;
//     objPoints.reserve(charucoIds.getMat().total());
//     for(unsigned int i = 0; i < charucoIds.getMat().total(); i++) {
//     int currId = charucoIds.getMat().at< int >(i);
//     CV_Assert(currId >= 0 && currId < (int)board.getChessboardCorners().size());
//     objPoints.push_back(board.getChessboardCorners()[currId]);
//     }

//     // points need to be in different lines, check if detected points are enough
//     if(!_arePointsEnoughForPoseEstimation(objPoints)) return false;

//     solvePnP(objPoints, charucoCorners, cameraMatrix, distCoeffs, rvec, tvec, useExtrinsicGuess);
//     return true;
//     }

// /** Check if a set of 3d points are enough for calibration. Z coordinate is ignored.
//  * Only axis parallel lines are considered */
// static bool _arePointsEnoughForPoseEstimation(const std::vector<Point3f> &points) {
//     if(points.size() < 4) return false;

//     std::vector<double> sameXValue; // different x values in points
//     std::vector<int> sameXCounter;  // number of points with the x value in sameXValue
//     for(unsigned int i = 0; i < points.size(); i++) {
//         bool found = false;
//         for(unsigned int j = 0; j < sameXValue.size(); j++) {
//             if(sameXValue[j] == points[i].x) {
//                 found = true;
//                 sameXCounter[j]++;
//             }
//         }
//         if(!found) {
//             sameXValue.push_back(points[i].x);
//             sameXCounter.push_back(1);
//         }
//     }

//     // count how many x values has more than 2 points
//     int moreThan2 = 0;
//     for(unsigned int i = 0; i < sameXCounter.size(); i++) {
//         if(sameXCounter[i] >= 2) moreThan2++;
//     }

//     // if we have more than 1 two xvalues with more than 2 points, calibration is ok
//     if(moreThan2 > 1)
//         return true;
//     return false;
// }
//////////////////////////////
//////////////////////////////
//////////////////////////////
// https://stackoverflow.com/questions/12794876/how-to-verify-the-correctness-of-calibration-of-a-webcam/12821056#12821056
// The correct way to check calibration accuracy is to use the reprojection error provided by OpenCV. I'm not sure why this
//  wasn't mentioned anywhere in the answer or comments, you don't need to calculate this by hand - it's the return value of
//   calibrateCamera. In Python it's the first return value (followed by the camera matrix, etc).
// The reprojection error is the RMS error between where the points would be projected using the intrinsic coefficients and
//  where they are in the real image. Typically you should expect an RMS error of less than 0.5px - I can routinely get around 
//  0.1px with machine vision cameras.

/////////////////////////////////////////////////
// class ChArucoDetector:
//     def __init__(self, cfg):
//         # configuration
//         self.board_sz = np.array([int(cfg.getNode("board_x").real()), int(cfg.getNode("board_y").real())])
//         self.square_len = cfg.getNode("square_len").real()
//         self.ardict = Dictionary_get(int(cfg.getNode("dictionary").real()))
        
//         self.marker_len = cfg.getNode("marker_len").real()
//         self.board = CharucoBoard_create(self.board_sz[0], self.board_sz[1], self.square_len, self.marker_len, self.ardict)
//         self.img_size = (int(cfg.getNode("image_width").real()), int(cfg.getNode("image_height").real()))

//         # per frame data
//         self.N_pts = 0
//         self.pose_valid = False
//         self.raw_img = None
//         self.pt_min_markers = int(cfg.getNode("pt_min_markers").real())

//         self.intrinsic_valid = False

//         # optical flow calculation
//         self.last_img = None
//         self.mean_flow = math.inf

//     def set_intrinsics(self, calib):
//         print("set_intrinsics")
//         self.intrinsic_valid = True
//         self.K = calib.K
//         self.cdist = calib.cdist

//     def draw_axis(self, img): # the board detected in the camera image
//         #drawAxis(img, self.K, self.cdist, self.rvec, self.tvec, self.square_len)
//         drawFrameAxes(img, self.K, self.cdist, self.rvec, self.tvec, 2.*self.square_len)

//     def detect_pts(self, img):
//         self.corners, ids, self.rejected = detectMarkers(img, self.ardict)

//         self.N_pts = 0

//         if ids is None or ids.size == 0:
//             return

//         res = interpolateCornersCharuco(self.corners, ids, img, self.board, minMarkers=self.pt_min_markers)
//         self.N_pts, self.ccorners, self.cids = res
//         # print("cids", self.cids)
//         if self.N_pts == 0:
//             return   

//         next_img = cv2.resize(img, (0, 0), fx=1./12., fy=1./12., interpolation=cv2.INTER_AREA)
//         next_img = cv2.cvtColor(next_img, cv2.COLOR_BGR2GRAY)
//         self.mean_flow = math.inf
//         if self.last_img is not None:
//             flow = cv2.calcOpticalFlowFarneback(self.last_img, next_img, None, 0.5, 3, 15, 3, 5, 1.2, 0)
//             mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
//             self.mean_flow = np.mean(mag)

//         self.last_img = next_img.copy()

//     def detect(self, img):
        
//         self.raw_img = img.copy()
//         # print("img shape", np.shape(img), np.shape(self.raw_img))

//         self.detect_pts(img)

//         if self.intrinsic_valid:
//             self.update_pose()

//     def get_pts3d(self):
//         return self.board.chessboardCorners[self.cids].reshape(-1, 3)

//     def get_calib_pts(self):
//         return (self.ccorners.copy(), self.get_pts3d())

//     def update_pose(self):
//         if self.N_pts < 4:
//             self.pose_valid = False
//             return

//         ret = estimatePoseCharucoBoard(self.ccorners, self.cids, self.board, self.K, self.cdist)
//         self.pose_valid, rvec, tvec = ret

//         if not self.pose_valid:
//             return

//         self.rvec = rvec.ravel()
//         self.tvec = tvec.ravel()

//         GlobalStuff.RTcamera[:3] = self.rvec
//         GlobalStuff.RTcamera[3:] = self.tvec
        
//         # print(cv2.RQDecomp3x3(cv2.Rodrigues(self.rvec)[0])[0])
//         # print(self.tvec)
