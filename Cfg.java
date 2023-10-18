package calibrator;

public class Cfg
{
    static final double var_terminate = 0.1;

    // camera image size and thus user display screen size
    static final int image_width = 1280; // 640
    static final int image_height = 720; // 480

    // ChArUco Board pixels = (board_x*square_len, board_y*square_len)
    static final int board_x = 9;
    static final int board_y = 6;
    static final int square_len = 280;
    static final int marker_len = 182;
    static final int dictionary = 0;
    static final boolean legacyPattern = true; // true is old style with aruco first, default in OpenCV is false - new style with black first

    // user config for convergence criteria
    static final int pt_min_markers = 1;
    static final double mean_flow_max = 2.; // exclusive
    static final double pose_close_to_tgt_min = 0.8; // exclusive. was 0.85 overlap - the Jaccard score between shadow and actual img
    static final double MAX_OVERLAP = 0.9; // maximum fraction of distortion mask overlapping with this pose before pose considered not contributing enough to help fill distortion mask
    static final double minCorners = 6; // min for solvePnP (original needed 4 or 5 w/o solvePnP) but another place requires many more

    static final int wait = 20; // milliseconds to wait for user keyboard response to a new image
    static final long keyLockoutDelayMillis = 10000L; // allows banging on the keyboard trying to get a response within "wait" and ignore multiple presses

    private Cfg(){}
}
