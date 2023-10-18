// This project is derived in part from the "Pose Calib" project by
// @author Pavel Rojtberg
// It is subject to his license terms in the LICENSE file.

//FIXME detector needs at least 6 points to keep solvePnP happy. UserGuidance needs 15 or more. Why not just one min?

package calibrator;

// changed method name oribital_pose to orbital_pose - rkt
// didn't change some other misspellings
// changed a couple of other "bugs" (maybe, I think)
// used current OpenCV methods. Some old ones were removed.

import java.io.BufferedOutputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     Main class                                                             */
/*                                     Main class                                                             */
/*                                     Main class                                                             */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
public class Main {
    static final String version = "CONVERTING IN PROGRESS"; // change this

    // LOGGER STUFF
    static final String logFile = "CalibrationLog.txt"; // user specified file name of the log

    public static final Logger LOGGER = Logger.getLogger("");
    private static final String outFormat = "%7$s%4$-7s [%3$s %2$s] %5$s %6$s%n";
    private static final String outTail = "THE END OF THE LOG\n";
    private static final String errFormat = "%7$s%1$tY-%1$tm-%1$td %1$tH:%1$tM:%1$tS.%1$tL %4$-7s [%3$s %2$s] %5$s %6$s%n";
    private static final String errTail = "THE END OF THE LOG\n";

    // normally don't change header but it's here (or below) to use version
    static final String header = "\n\nStarting Log for Camera Calibration Program Version " + version
    + ", current time " + java.time.LocalDateTime.now() + "\n\n";
    //    + "  current time ms " + System.currentTimeMillis() + "\n\n";   
    private static final String errHeader = header;
    private static final String outHeader = header;

    // 1st stage log message filter by level
    // whatever passes this stage goes to the 2nd stage filter
    private static final Level outLevel = Level.ALL; // 1st stage filter log level for out (which also gets a copy of err)
    private static final Level errLevel = Level.ALL; // 1st stage filter log level for err

    // 2nd stage filter by level and Class
    // there also may be custom filters in the 2nd stage - see the code
    // individual log level specifications can be set by Java properties -Dkey=value; nothing implemented yet
    // true allows all 2nd stage messages to pass; false allows 2nd stage custom filters to work
    static boolean outOverride2ndStageClassFilter = false;
    static boolean errOverride2ndStageClassFilter = false;

    // classes to specify log level
    static final String[] classesLog = { 
        "calibrator.PoseGeneratorDist",
        "calibrator.BoardPreview",
        "calibrator.Distortion",
        "calibrator.ChArucoDetector",
        "calibrator.UserGuidance",
        "calibrator.Calibrator",
        "calibrator.Cfg",
        "calibrator.Main"
        };
    static final Map<String, Level> classLevels = new HashMap<String, Level>(15);
    static { 
    // set all minimum log levels to display the same easily
    // java.util.logging Levels	ALL FINEST FINER	FINE	INFO	CONFIG  WARNING	SEVERE	OFF
    for (String key : classesLog)
        {
        String value = "FINEST";
        classLevels.put(key, Level.parse(value));
        }
    }
    static int frameNumber = 0;
    static String frame = "00000 ";
    // END LOGGER STUFF

    static
    {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME); // Load the native OpenCV library
    }

    // keyboard mapping returns from waitKey
    static final int keyTerminate = 27;
    static final int keyCapture = 67;
    static final int keyMirrorToggle = 77;
    static final int timedOut = -1;  // timed out no key pressed
    static long allowKeyPressAfterTime = 0; // time when next key press is allowed - prevents multiple rapid, unintentional presses; allows holding key down for extended period so it is seen by the keywait

    // Checks for the specified camera and uses it if present. 0 internal, 1 external if there is a 0 internal (sometimes)    
    static final int camId = 0;
    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}

    public static Mat testImg1 = new Mat(); // testing only
    public static Mat testImg2 = new Mat(); // testing only

/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     main                                                                   */
/*                                     main                                                                   */
/*                                     main                                                                   */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    public static void main(String[] args) throws Exception
    {
        OutputStream copySystemErr = System.err; // initialize System.err duplicated stream to just the err
        // add the file that is a running duplicate of System.err
        try { // create a stream with the 2 substreams
            copySystemErr = new BufferedOutputStream(new FileOutputStream(logFile, true));
            TeePrintStream errStreamCopied = new TeePrintStream(
                                System.err,
                                copySystemErr,
                                true);
            System.setErr(errStreamCopied); // use the duplicating stream as System.err
        } catch (IOException e) {
            e.printStackTrace();
        }

        Loggers.setupLoggers(copySystemErr, outFormat, outHeader, outTail, outLevel, errFormat, errHeader, errTail, errLevel);

        // not sure which of these are required for the camera server so leave them all in
        NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
        WPIMathJNI.Helper.setExtractOnStaticLoad(false);
        CameraServerJNI.Helper.setExtractOnStaticLoad(false);
        CombinedRuntimeLoader.loadLibraries(Main.class, "wpiutiljni", "wpimathjni", "ntcorejni", "cscorejnicvstatic");
        // var inst = NetworkTableInstance.getDefault(); // not using NT in this program

        /** video image capture setup **/
        // Get the UsbCamera from CameraServer
        final UsbCamera camera = CameraServer.startAutomaticCapture(camId);
        camera.setPixelFormat(PixelFormat.kYUYV);
        camera.setResolution(Cfg.image_width, Cfg.image_height);
        camera.setExposureAuto();
        // camera.setExposureManual(65);
        // camera.setBrightness(50);
        // Get a CvSink. This will capture Mats from the camera
        JavaCvSink cap = new JavaCvSink("sink1");
        cap.setSource(camera);
        Mat _img = new Mat(); // this follows the camera input
        Mat  img = Mat.zeros(Cfg.image_height, Cfg.image_width, CvType.CV_8UC3); // set by user config
        /** end video image capture setup **/

        Mat out = new Mat(); // user display Mat

        // runtime variables
        boolean mirror = true;
        boolean save = false; // indicator for user pressed the "c" key to capture (save) manually    
        ChArucoDetector tracker = new ChArucoDetector();
        UserGuidance ugui = new UserGuidance(tracker, Cfg.var_terminate);

        grabFrameLoop:
        while (!Thread.interrupted())
        {
            frameNumber++;
            frame = String.format("%05d ", frameNumber);
            boolean force = false;  // force add frame to calibration (no support yet still images else (force = !live)
            if (cap.grabFrame(_img, 0.5) != 0)
            {
                if(_img.height() != Cfg.image_height || img.width() != Cfg.image_width) // enforce camera matches user spec
                {
                    // Imgproc.resize(_img, _img, new Size(Cfg.image_width, Cfg.image_height), 0, 0, Imgproc.INTER_CUBIC);
                    Main.LOGGER.log(Level.SEVERE, "image grabbed not correct size - ignoring it");
                    continue;
                }
                _img.copyTo(img);
            }
            else
            {
                LOGGER.log(Level.SEVERE, "grabFrame error " + cap.getError());
                force = false;
                continue; // pretend frame never happened rkt addition
            }
            
            tracker.detect(img);

            if (save)
            {
                save = false;
                force = true;
            }

            img.copyTo(out); // out has the camera image at his point

            ugui.draw(out, mirror); // this adds the guidance board to the camera image (out) to make the new out

            ugui.update(force); // calibrate
            
            if (ugui.converged) // are we there yet?
            {
                ugui.write(); // write all the calibration data

                break grabFrameLoop; // the end
            }

            displayOverlay(out, ugui);

            if( ! testImg1.empty()) HighGui.imshow("test img 1", testImg1);
            if( ! testImg2.empty()) HighGui.imshow("test img 2", testImg2);

            HighGui.imshow("PoseCalib", out);

            int k = HighGui.waitKey(Cfg.wait);

            if(k == timedOut)
            {
                continue; // no key press to process
            }
            
            // have a key
            long currentTimeMillis = System.currentTimeMillis();
            if(currentTimeMillis < allowKeyPressAfterTime)
            {// smarter way is check for SAME key pressed rapidly but this checking for any key pressed is good enough for now
                continue; // a key pressed soon after previous press so ignore it
            }
            
            // process this key
            Main.LOGGER.log(Level.WARNING, "Pressed Key " + k);
            
            allowKeyPressAfterTime = currentTimeMillis + Cfg.keyLockoutDelayMillis; // lockout more presses for a awhile

            switch(k)
            {
                case keyTerminate: // terminate key pressed to stop loop immediately
                        break grabFrameLoop;

                case keyMirrorToggle: // mirror/no mirror key pressed
                        mirror = !mirror;
                        break;

                case keyCapture: // capture frame key pressed
                        save = true;
                        break;

                default: // unassigned key
                        break;
            }
        } // end grabFrameLoop

        ugui.write(); //FIXME temp just to see what comes out - remove when program working right and converging

        Main.LOGGER.log(Level.CONFIG,"End of running main");
        System.exit(0);
    }
    
    public static void displayOverlay(Mat out, UserGuidance ugui)
    {
        Imgproc.putText(out, Main.frame, new Point(0, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(0, 0, 0), 2);
        Imgproc.putText(out, Main.frame, new Point(0, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(255, 255, 255), 1);

        if (ugui.user_info_text.length() > 0) // is there a message to display?
        {
            if ( ! ugui.user_info_text.equals("initialization")) // rkt stop spamming "initialization" to log
                Main.LOGGER.log(Level.WARNING,ugui.user_info_text);

            Imgproc.putText(out, ugui.user_info_text, new Point(80, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(0, 0, 0), 2);
            Imgproc.putText(out, ugui.user_info_text, new Point(80, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(255, 255, 255), 1);
        }

        Imgproc.putText(out, ugui.tgt_r.dump()+ugui.tgt_t.dump(), new Point(0, 40), Imgproc.FONT_HERSHEY_SIMPLEX, .6, new Scalar(0, 0, 0), 2);
        Imgproc.putText(out, ugui.tgt_r.dump()+ugui.tgt_t.dump(), new Point(0, 40), Imgproc.FONT_HERSHEY_SIMPLEX, .6, new Scalar(255, 255, 255), 1);
    }
}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     End Main class                                                         */
/*                                     End Main class                                                         */
/*                                     End Main class                                                         */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
// #!/usr/bin/env python3

// """
// This file is part of the "Pose Calib" project.
// It is subject to the license terms in the LICENSE file found
// in the top-level directory of this distribution.

// @author Pavel Rojtberg
// """

// import cv2
// import argparse
// import sys

// from ui import UserGuidance
// from utils import ChArucoDetector
    
// class UVCVideoCapture:
//     def __init__(self, cfg):
//         self.manual_focus = True
//         self.manual_exposure = True

//         imsize = (int(cfg.getNode("image_width").real()), int(cfg.getNode("image_height").real()))
        
//         cam_id = 0
//         if not cfg.getNode("v4l_id").empty():
//             cam_id = "/dev/v4l/by-id/usb-{}-video-index0".format(cfg.getNode("v4l_id").string())
        
//         cap = cv2.VideoCapture(cam_id)
//         cap.set(cv2.CAP_PROP_FRAME_WIDTH, imsize[0])
//         cap.set(cv2.CAP_PROP_FRAME_HEIGHT, imsize[1])
//         cap.set(cv2.CAP_PROP_GAIN, 0.0)
//         cap.set(cv2.CAP_PROP_AUTOFOCUS, not self.manual_focus)
//         cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        
//         val = 1 / 4 if self.manual_exposure else 3 / 4
//         cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, val)
        
//         self.cap = cap
        
//         assert self.cap.isOpened()

//     def set(self, prop, val):
//         self.cap.set(prop, val)
        
//     def read(self):
//         return self.cap.read()

// def add_camera_controls(win_name, cap):
//     cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)

//     if cap.manual_focus:
//         focus = 0
//         cap.set(cv2.CAP_PROP_FOCUS, focus / 100)
//         cv2.createTrackbar("Focus", win_name, focus, 100, lambda v: cap.set(cv2.CAP_PROP_FOCUS, v / 100))

//     if cap.manual_exposure:
//         exposure = 200
//         cap.set(cv2.CAP_PROP_EXPOSURE, exposure / 1000)
//         cv2.createTrackbar("Exposure", win_name, exposure, 1000, lambda v: cap.set(cv2.CAP_PROP_EXPOSURE, v / 1000))    
    
// def main():
//     parser = argparse.ArgumentParser(description="Interactive camera calibration using efficient pose selection")
//     parser.add_argument("-c", "--config", help="path to calibration configuration (e.g. data/calib_config.yml)")
//     parser.add_argument("-o", "--outfile", help="path to calibration output (defaults to calib_<cameraId>.yml)")
//     parser.add_argument("-m", "--mirror", action="store_true", help="horizontally flip the camera image for display")
//     args = parser.parse_args()

//     if args.config is None:
//         print("falling back to "+sys.path[0]+"/data/calib_config.yml")
//         args.config = sys.path[0]+"/data/calib_config.yml"

//     cfg = cv2.FileStorage(args.config, cv2.FILE_STORAGE_READ)
//     assert cfg.isOpened()

//     calib_name = "default"
//     if not cfg.getNode("v4l_id").empty():
//         calib_name = cfg.getNode("v4l_id").string()

//     # Video I/O
//     live = cfg.getNode("images").empty()
//     if live:
//         cap = UVCVideoCapture(cfg)
//         add_camera_controls("PoseCalib", cap)
//         wait = 1
//     else:
//         cv2.namedWindow("PoseCalib")
//         cap = cv2.VideoCapture(cfg.getNode("images").string() + "frame%0d.png", cv2.CAP_IMAGES)
//         wait = 0
//         assert cap.isOpened()

//     tracker = ChArucoDetector(cfg)


//     # user guidance
//     ugui = UserGuidance(tracker, cfg.getNode("terminate_var").real())

//     # runtime variables
//     mirror = False
//     save = False

//     while True:
//         force = not live  # force add frame to calibration

//         status, _img = cap.read()
//         if status:
//             img = _img
//         else:
//             force = False

//         tracker.detect(img)

//         if save:
//             save = False
//             force = True

//         out = img.copy()

//         ugui.draw(out, mirror)

//         ugui.update(force)
        
//         if ugui.converged:
//             if args.outfile is None:
//                 outfile = "calib_{}.yml".format(calib_name)
//             else:
//                 outfile = args.outfile
//             ugui.write(outfile)

//         if ugui.user_info_text:
//             cv2.displayOverlay("PoseCalib", ugui.user_info_text, 1000 // 30)

//         cv2.imshow("PoseCalib", out)
//         k = cv2.waitKey(wait)

//         if k == 27:
//             break
//         elif k == ord('m'):
//             mirror = not mirror
//         elif k == ord('c'):
//             save = True

// if __name__ == "__main__":
//     main()

/////////////////////////////////////////////////////////////////////////

// Logger.getLogger("").setLevel(Level.OFF); // turn off everything - especially java.awt fine, finer, finest spam
// Logger.getLogger("calibrator").setLevel(Level.ALL); // turn on for my package

// System.loadLibrary("opencv_videoio_ffmpeg480_64");

/*

cd C:\Users\RKT\frc\FRC2023\Code\Java
find "repError" CalibrationLog.txt
find "jaccard" CalibrationLog.txt
find "board_warped created" CalibrationLog.txt

 */



 /*
  * The rotvecd( ) function gives a scaled rotation axis.  I.e., what it would take to do a single rotation of an object
   on all three axes simultaneously to get from one orientation to another.  The direction of the vector gives the rotation
    axis, and the magnitude of the vector gives the total rotation angle.
The eulerd( ) function gives the angles for performing three separate sequential rotations (not simultaneous) to get from
 one orientation to another.  E.g., rotate about the X-axis by some amount first, then from that resulting position rotate
  about the Y-axis by some amount, then from that resulting position rotate about the Z-axis by some amount to get to the
   final orientation.
In general, anytime you deal with Euler Angles you are dealing with separate rotations that are chained together sequentially.

All rotations in 3-D can be represented by four elements: a three-element axis of rotation and a rotation angle.
 If the rotation axis is constrained to be unit length, the rotation angle can be distributed over the vector elements to
  reduce the representation to three elements.

https://www.mathworks.com/help/nav/ref/quaternion.rotvecd.html
  */