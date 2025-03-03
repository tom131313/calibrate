package Guidance;

import org.opencv.core.Scalar;
import org.opencv.core.TermCriteria;
import org.opencv.objdetect.Objdetect;

import java.util.logging.Level;

/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     Cfg class                                                   */
/*                                     Cfg class                                                   */
/*                                     Cfg class                                                   */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
public class Cfg
{
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     Cfg constructor                                             */
/*                                     Cfg constructor                                             */
/*                                     Cfg constructor                                             */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
// USER SPECIFIABLE
    static int dictionary = Dictionary.valueOf("FourByFour").dictionary;
    static int board_x = 8; //9;
    static int board_y = 8; //6;
    static int resXDPM = 9843; // board printing pixels per meter 9843 = 250 DPI
    static int resYDPM = 9843; // board printing pixels per meter 9843 = 250 DPI

// MAYBE SHOULD BE USER SPECIFIABLE
    static final Level loggerMinimumLevel = Level.FINE;
    static final String boardFile = "ChArUcoBoard";
    static final String cornersLog = "Corners"; // only used for a snapshot log
    static final String videoFile = "CalibrationVideo"; // video format of saved images
    static final long messageHoldTime = 5000L; // milliseconds to hold display of message

    // ALL THE FOLLOWING STUFF MIGHT NEVER NEED TO BE CHANGED
    public static final int garbageCollectionFrames = 500; // camera frames - periodically do garbage collection because Java doesn't know there are big Mats to be released
    static final double initialFocalLength = 1000.; // fx and fy, aspect ratio = 1 (fy/fx)
    // ChArUco Board pixels = (board_x*square_len, board_y*square_len)


    static final int square_len = 250; //280;
    static final int marker_len = 188; //182;
    // intensity of the green guidance board
    // suggest "white" [-100]; (dull) -128 to -1 (bright)
    // suggest "black" [1]; (somewhat transparent) 1 to 64 (more obscuring)
    static final byte guidanceWhite = -50; // green actually; (black) 0 to 127 (medium), (medium) -128 to -1 (bright)
    static final byte guidanceBlack = 1; // ; (dark) 1 to 127, -128 to -1 (bright); must be much less than guidanceWhite and NOT 0
    static final Scalar progressInsertCameraGrey = new Scalar(170.);
    static final Scalar progressInsertGuidanceGrey = new Scalar(105.);

    // config for convergence criteria
    static final int pt_min_markers = 1;
    static final boolean tryRefineMarkers = true;
    static final int cornerRefinementMaxIterations = 2000;
    static final int cornerRefinementMethod = Objdetect.CORNER_REFINE_CONTOUR;
    static final boolean checkAllOrders = true;
    static final float errorCorrectionRate = 3.0f;
    static final float minRepDistance = 10.0f;

    static final double mean_flow_max = 3.; // exclusive, larger is more movement allowed
    public static final double pose_close_to_tgt_min = 0.85; // exclusive, - minimum Jaccard score between shadow and actual img for auto capture; larger is less deviation allowed
    static final double MAX_OVERLAP = 0.9; // maximum fraction of distortion mask overlapping with this pose before pose considered not contributing enough to help fill distortion mask
    static final double minCorners = 6; // min for solvePnP (original needed 4 (or 5 w/o solvePnP)) but another place requires many more
    public static final double var_terminate = 0.1; // min variance to terminate an intrinsic's iterations [mm is whatever unit of measure?]

    static final double DBL_EPSILON = Math.ulp(1.);
    static final TermCriteria calibrateCameraCriteria = new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 30, DBL_EPSILON);

    static final float FLT_EPSILON = Math.ulp(1.f);
    static final TermCriteria solvePnPRefineVVSCriteria = new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 20, FLT_EPSILON);
    static final double solvePnPRefineVVSLambda = 1.;

    static final TermCriteria undistortPointsIterCriteria = new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 20, FLT_EPSILON); // default cv::TermCriteria(cv::TermCriteria::COUNT, 5, 0.01)
/////////////////////////////////////////////////////////

    // User choices for ArUco
    public enum Dictionary {
        FourByFour(Objdetect.DICT_4X4_1000),
        FiveByFive(Objdetect.DICT_5X5_1000);
        int dictionary;
        private Dictionary(int dictionary)
        {
            this.dictionary = dictionary;
        }
    }

    private Cfg()
    {
        throw new UnsupportedOperationException("This is a utility class");
    }
}
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     End Cfg class                                               */
/*                                     End Cfg class                                               */
/*                                     End Cfg class                                               */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
