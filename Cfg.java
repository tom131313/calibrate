package calibrator;

public class Cfg
{
    static final double var_terminate = 0.1;

    // camera image size
    static final int image_width = 1280; // 640
    static final int image_height = 720; // 480

    // ChArUco Board
    static final int board_x = 9;
    static final int board_y = 6;
    static final int square_len = 280;
    static final int marker_len = 182;
    static final int dictionary = 0;

    // user config for convergence criteria
    static final int pt_min_markers = 1;
    static final double pt_match_min_factor = 0.9; // if < 17 pts how many are required?
    static final double mean_flow_max = 2.; // exclusive
    static final double pose_close_to_tgt_min = 0.8; // exclusive. was 0.85 overlap
    static final double MAX_OVERLAP = 0.9; // minimum fraction of distortion mask filled

    static final int wait = 30; // milliseconds to wait for user keyboard response to a new image

    private Cfg(){}
}
