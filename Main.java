// This project and file are derived in part from the "Pose Calib" project by
// @author Pavel Rojtberg
// It is subject to his license terms in the PoseCalibLICENSE file.

// Calibrate Camera with efficient camera Pose Guidance provided on screen to the user

package Guidance;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.logging.Logger;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import org.apache.commons.lang3.tuple.Pair;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import Guidance.CVPipe.CVPipeResult;
import Guidance.FindBoardCornersGuidancePipe.FindBoardCornersGuidancePipeResult;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.util.PixelFormat;

/*
 * The PhotonVision Calibrate3dPipeline.java class separately calls
 * PV FindBoardCorners.java to find the corners of the board and then calls
 * PV Calibrate3dPipe.java to use those corners to calibrate the camera.
 * 
 * The PV user and camera interfaces are handled by other PV processes.
 *  
 * This Guidance Main class calls FindBoardCornersGuidance.java to both find the corners
 * and continuously calibrate the camera as those corners are found - the two processes
 * are integrated. If convergence criteria for calibration accuracy are met then the last
 * calibration performed is accepted as the result.
 * 
 * This Main, also handles the user and camera interfaces and stands alone from the PV
 * framework but does similarly use some PV structures, WPILib functions, and Apache
 * commons libraries.
 */

/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     Main class                                                  */
/*                                     Main class                                                  */
/*                                     Main class                                                  */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
public class Main {
    private static final String VERSION = "beta-14"; // change this


    private static Logger LOGGER;
    static {
        LOGGER = LoggerSetup.setupLogger();

        LOGGER.finest("Loading");
        LOGGER.config("Pose Guidance Camera Calibration version " + VERSION);

        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private static FindBoardCornersGuidancePipe findBoardCornersGuidancePipe;
    private static CVPipeResult<FindBoardCornersGuidancePipeResult> findBoardCornersGuidancePipeResult;

    private static PrintWriter vnlog = null; // first time switch value is null
    private static VideoCreation video = null; // first time switch value is null

    ///////////////////////////// USER INPUT ARGUMENTS ///////////////////////////////
    // camera ID instructions:
    // Checks for the specified camera - laptop internal or USB external and uses it if present.
    // 0 internal if no external or if external connected after boot-up
    // 0 external if connected at boot-up
    // 1 internal if external connected at boot-up
    // 1 is external if connected after boot-up
    static String camId;

    // camera format specification
    static PixelFormat pixelFormat;
    static int fps;
    // Note that user output display screen size follows the same as the camera size
    static int image_width;
    static int image_height;

    static boolean focus; // specify focus mode (or calibration mode if focus mode is false)
    static boolean logSnapshot; // specify saving images and corners used for calibration

    // output URL port - focus and calibration modes
    static int displayPort;
    ///////////////////////////// END USER INPUT ARGUMENTS ///////////////////////////////
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     main                                                        */
/*                                     main                                                        */
/*                                     main                                                        */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/

    public static void main(String[] args) throws Exception
    {

        // get the parameters for the user provided options
        LOGGER.config("Command Line Args " + Arrays.toString(args));  
        try {
            if ( ! handleArgs(args)) {
                System.exit(0);
            }
        } catch (ParseException e) {
            LOGGER.severe("Failed to parse command-line options! " + e);
            System.exit(0);
        }

        // establish keyboard handler
        Keystroke keystroke;
        Thread keyboardThread;
        keystroke = new Keystroke();
        keyboardThread = new Thread(keystroke, "keys");
        keyboardThread.setDaemon(true);
        keyboardThread.start();

        // image input source - determine if USB attached camera or URL for remote camera
        CvSink capture;

        var remoteCamera = camId.toLowerCase().contains("http://"); // assume it's a URL remote feed or not

        if (remoteCamera) {
            capture = CameraHTTP.getSource(
                camId.substring(0, camId.indexOf(",")),
                camId.substring(camId.indexOf(",") + 1).trim());
        }
        else { // assume it's an integer camera id
            capture = CameraUSB.getSource(Integer.parseInt(camId), image_width, image_height, fps, pixelFormat);            
        }

        Mat img = new Mat();

        // output display for pose guidance board with input image or focus board
        CvSource networkDisplay = null;
        MjpegServer mjpegServer;
        networkDisplay = new CvSource("calibPV", PixelFormat.kMJPEG,
                image_height, image_width, fps);
        mjpegServer = new MjpegServer("GuidanceView", displayPort);
        mjpegServer.setSource(networkDisplay);
        LOGGER.config("View Guidance Board with Camera Image On Port " + displayPort);

        // Grab first good image from camera to set the size.

        // Attached USB camera would already know the size here so this is a bit redundant.
        // For remote camera this sets the size and that must not change from now on.

        // Note that PV changes the stream image size when it enters calibration mode
        // so start PV calibration before starting this pose guidance.

        while ( ! Thread.interrupted())
        {
            long status = capture.grabFrame(img, 0.5);
            if (status != 0)
            {
                break;
            }
            else
            {
                LOGGER.warning("initial grabFrame error " + capture.getError() + " - retrying");
            }
        }
        Size img_size = new Size(img.width(), img.height());
        LOGGER.config("image size " + img_size.toString());
        Size img_size_prev = img_size.clone();        
        Mat out = new Mat(); // user display Mat

        findBoardCornersGuidancePipe = new FindBoardCornersGuidancePipe();

        // initialize the params - "save" and "mirror" can change with each image frame
        var param = new FindBoardCornersGuidancePipe.FindCornersGuidancePipeParams(
            false,
            false,
            Cfg.board_x,
            Cfg.board_y,
            UICalibrationData.BoardType.CHESSBOARD,
            25.4,
            FrameDivisor.NONE
        );

        findBoardCornersGuidancePipe.setParams(param);
        findBoardCornersGuidancePipeResult = null;
        long holdMessageTimer = System.currentTimeMillis();
        var recentSnapshot = false;
        
        // Grab a camera image, process it, and interpret any keyed entry commands.
        // Loop until calibration is automatically complete or user quits (especially focus mode).

        frameGrabLoop:
        while ( ! Thread.interrupted()) {

            // get an image frame and validate it
            long status = capture.grabFrame(img, 0.5);
            if (status != 0)
            {
                img_size = new Size(img.width(), img.height());
                // if image size changes during calibration bail out and get ready to calibrate again with new size
                if ( ! img_size_prev.equals(img_size))               
                {
                    LOGGER.warning("changing image size to " + img_size + " from " + img_size_prev + ", quitting guidance calibration");
                    img_size_prev = img_size.clone();
                    break frameGrabLoop; // quit
                }
            }
            else
            {
                LOGGER.warning("grabFrame error " + capture.getError());
                continue frameGrabLoop; // pretend frame never happened - rkt addition; original reprocessed previous frame
            }

            /* process the good image frame */

            // focus mode processing
            if (focus) {
                
                // get any user keyed input useful for focus mode (ignore the rest)
                switch (keystroke.getKey())
                {
                    case Keystroke.keyNone: // no key press to process
                            break;
                    case Keystroke.keyTerminate:
                            LOGGER.info("Pose Guidance Camera action CANCELLED");
                            break frameGrabLoop; // quit
                    default: // unassigned key
                            break;
                }

                Class.forName("Guidance.Focus"); // force loading the history buffer if it's not already loaded
                out = Focus.drawSiemensStar();
                var sharpness = Focus.sharpnessMetric(img);
                Imgproc.putText(out, Integer.toString(sharpness), new Point(0, 52), Imgproc.FONT_HERSHEY_SIMPLEX, 2., new Scalar(210), 3);
                // focus process loops until user input q(uits)
            }

            // calibrate mode processing
            else {

                // calibration image frame initialization
                param.save = false;
                
                // get any user keyed input useful for calibration mode (ignore the rest)
                switch (keystroke.getKey())
                {
                    case Keystroke.keyNone: // no key press to process
                            break;
                    case Keystroke.keyTerminate:
                            LOGGER.info("Pose Guidance Camera action CANCELLED");
                            break frameGrabLoop; // quit
                    case Keystroke.keyMirrorToggle:
                            LOGGER.info("Toggled mirror guidance");
                            param.mirror = ! param.mirror;
                            break;
                    case Keystroke.keyCapture:
                            LOGGER.info("Forced capture snapshot");
                            param.save = true; // it's one frame behind what user sees but it's usually too fast to notice
                            break;
                    default: // unassigned key
                            break;
                }

                findBoardCornersGuidancePipe.setParams(param); // update find corners params in case they changed
                var images = Pair.of(img,out ); // define the find corners input and output
                findBoardCornersGuidancePipeResult = findBoardCornersGuidancePipe.run(images);

                // act on find corners results
                if (findBoardCornersGuidancePipeResult.output.madeSnapshot)
                {
                    // The guidance program saves elsewhere its own snapshot needed for guidance and notifies us here.
                    // In addition, corners and images could be logged here and saved snapshots could be further
                    // processed by say Mr.Cal.
                    if (logSnapshot) {
                        logSnapshot(img, findBoardCornersGuidancePipeResult);
                    }

                    recentSnapshot = true; // trigger snapshot display message

                    LOGGER.info("Snapshot taken");
                }

                // if calibrating on another program user needs to inform it of this guidance pose snapshot
                if (recentSnapshot && remoteCamera && (holdMessageTimer + Cfg.messageHoldTime) > System.currentTimeMillis()) {
                    Imgproc.putText(out, "HOLD STEADY", new Point(0.,  45.), Imgproc.FONT_HERSHEY_SIMPLEX, 2., new Scalar(255, 255, 255), 5);
                    Imgproc.putText(out, "TAKE SNAPSHOT", new Point(0., 110.), Imgproc.FONT_HERSHEY_SIMPLEX, 2., new Scalar(255, 255, 255), 5);
                    Imgproc.putText(out, "ON REMOTE", new Point(0., 175.), Imgproc.FONT_HERSHEY_SIMPLEX, 2., new Scalar(255, 255, 255), 5);
                }
                else { // message held long enough so reset hold message timing
                    holdMessageTimer = System.currentTimeMillis();
                    recentSnapshot = false;
                }

                if (findBoardCornersGuidancePipeResult.output.haveEnough) // done calibrating
                {
                    //  further processing of the complete set of snapshots could be done here - this is the end   
                    // The guidance program prints elsewhere its calibration results to the LOGGER and notifies us here.
                    if(logSnapshot) {
                        if(vnlog != null) {
                            vnlog.close();
                            vnlog = null; // redundant cleanup just to make sure a logic error doesn't try to use it again
                        }
                        if(video != null) {
                            video.close();
                            video = null; // redundant cleanup just to make sure a logic error doesn't try to use it again
                        }
                    }
                        LOGGER.severe("Pose Guidance Camera action calibrated");

                    // check for holding the last message before quitting
                    if (!recentSnapshot)
                    {
                        break frameGrabLoop; // done with calibration
                    }
                }
                else    
                if (findBoardCornersGuidancePipeResult.output.cancelCalibration)
                {
                    break frameGrabLoop; // quit
                }
            }

            // completed processing image frame - display results
            networkDisplay.putFrame(out);

        } // bottom of frameGrabLoop loop

        // quitting
        mjpegServer.close();
        networkDisplay.close();
        LOGGER.finest("End of running main");
    } // end main method
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     handleArgs                                                  */
/*                                     handleArgs                                                  */
/*                                     handleArgs                                                  */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
private static boolean handleArgs(String[] args) throws ParseException {

        Options options = new Options();

        options.addOption("h", "help", false, "Show this help text and exit");
        options.addOption("W", "width", true, "camera image width (1280)");
        options.addOption("H", "height", true, "camera image height (720)");
        options.addOption("X", "dpmX", true, "print width pixels per meter (9843=250 DPI)");
        options.addOption("Y", "dpmY", true, "print height pixels per meter (9843=250 DPI");
        options.addOption("F", "pxFmt", true, "camera pixel format (kYUYV) " + Arrays.toString(PixelFormat.values()));
        options.addOption("c", "cameraId", true, "camera id (0); two forms: 1. integer or 2. name, http://...");
        options.addOption("R", "fps", true, "camera frames per second (10)");
        options.addOption("x", "sqrWide", true, "ChArUco board squares wide (8)");
        options.addOption("y", "sqrHigh", true, "ChArUco board squares high (8)");
        options.addOption("D", "dictionary", true, "8x8 square ArUco board; dictionary (FourByFour) " + Arrays.toString(Cfg.Dictionary.values()));
        options.addOption("B", "printBoard", false, "print ChArUco Board to file ChArUcoBoard.png");
        options.addOption("f", "focus", false, "sharpness measure mode - no calibration");
        options.addOption("d", "displayPort", true, "output image port (1185)");
        options.addOption("S", "logSnapshot", false, "save snapshots image files and corners");

        CommandLineParser parser = new DefaultParser();
        CommandLine cmd = parser.parse(options, args);

        if(cmd.getArgs().length > 0) {
            LOGGER.warning("Arguments Not Recognized: " + Arrays.toString(cmd.getArgs()));            
        }

        if (cmd.hasOption("h")) {
            // make a string to hold the help and log it
            StringWriter sw = new StringWriter(1000);
            PrintWriter pw = new PrintWriter(sw);
            HelpFormatter helpFormatter = new HelpFormatter.Builder().setPrintWriter(pw).get();
            helpFormatter.printHelp("\n\njava -jar <your jar file>.jar [options]", options);
            LOGGER.config("\n\n" + sw.toString());
            return false; // exit program
        }

        image_width = Integer.parseInt(cmd.getOptionValue("width", "1280"));
        image_height = Integer.parseInt(cmd.getOptionValue("height", "720"));
        Cfg.resXDPM = Integer.parseInt(cmd.getOptionValue("dpmX", "9843"));
        Cfg.resYDPM = Integer.parseInt(cmd.getOptionValue("dpmY", "9843"));
        pixelFormat = PixelFormat.valueOf(cmd.getOptionValue("pxFmt", "kYUYV"));
        Cfg.board_x = Integer.parseInt(cmd.getOptionValue("x", "8"));
        Cfg.board_y = Integer.parseInt(cmd.getOptionValue("y", "8"));
        Cfg.dictionary = Cfg.Dictionary.valueOf(cmd.getOptionValue("dictionary", "FourByFour")).dictionary;
        camId = cmd.getOptionValue("cameraId", "0");
        fps = Integer.parseInt(cmd.getOptionValue("fps", "10"));
        displayPort = Integer.parseInt(cmd.getOptionValue("displayPort", "1185"));

        if (cmd.hasOption("B")) { // must be after the dictionary and number of board squares options
            ChArUcoBoardPrint.print(); // because do the function here rather than pass back a switch to do it
        }

        focus = cmd.hasOption("f");
        logSnapshot = cmd.hasOption("S");

        return true;

    } // end handleArgs method
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     logSnapshot                                                 */
/*                                     logSnapshot                                                 */
/*                                     logSnapshot                                                 */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    private static int captureCount; // image save file name sequence number
    private static String fileTime;
    /**
     * Save Detected Board corner data in mrgingham format plus the board info
     * Save snapshot images individually and as a video (movie)
     * Vnlog (“vanilla-log”) file format (https://github.com/dkogan/vnlog)
     * @param img Camera image to be saved
     * @param findBoardCornersGuidancePipeResult Detection to be saved
     * @throws FileNotFoundException
     */
    public static void logSnapshot(Mat img, CVPipeResult<FindBoardCornersGuidancePipeResult> findBoardCornersGuidancePipeResult) throws FileNotFoundException
    {
        if (vnlog == null || video == null) // first time switch
        {
            fileTime = new SimpleDateFormat("_yyyy-MM-dd_HH-mm").format(System.currentTimeMillis());
            captureCount = 0;

            vnlog = new PrintWriter(Cfg.cornersLog + fileTime + ".vnl");
            vnlog.println("## produced by pose guidance calibration program");
            vnlog.println("# filename x y level cid boardX boardY");
            video = new VideoCreation(Cfg.videoFile + fileTime + ".mp4", img.size());
     }
        
        video.addFrame(img); // save camera image in the video file

        // write the captured frame to a sequenced file name
        String filename = String.format("img_%s_%03d.jpg", fileTime, ++captureCount);
        final MatOfInt writeBoardParams = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, 100); // pair-wise; param1, value1, ...
        Imgcodecs.imwrite(filename, img, writeBoardParams); // save camera image in its own file

        // for efficiency put Mat data in primative arrays

        float[] ChessboardCorners = new float[findBoardCornersGuidancePipeResult.output.objectPoints.rows()
            *findBoardCornersGuidancePipeResult.output.objectPoints.cols()
            *findBoardCornersGuidancePipeResult.output.objectPoints.channels()]; // below assumes x and y in a row
        findBoardCornersGuidancePipeResult.output.objectPoints.get(0, 0, ChessboardCorners);

        int[] DetectedCIDs = new int[findBoardCornersGuidancePipeResult.output.idCorners.rows()]; // get detected corners - assume captured image does have corners
        findBoardCornersGuidancePipeResult.output.idCorners.get(0, 0, DetectedCIDs);

        float[] DetectedCorners = new float[findBoardCornersGuidancePipeResult.output.imagePoints.rows()
            *findBoardCornersGuidancePipeResult.output.imagePoints.cols()
            *findBoardCornersGuidancePipeResult.output.imagePoints.channels()]; // below assumes x and y in a row
        findBoardCornersGuidancePipeResult.output.imagePoints.get(0, 0, DetectedCorners);

        // save vnlog     
        for (int detectedCornerIndex = 0; detectedCornerIndex < DetectedCIDs.length; detectedCornerIndex++)
        {
            int boardCornerId = DetectedCIDs[detectedCornerIndex]; // get board corner that is detected
            StringBuilder logLine = new StringBuilder();
            logLine.append(filename);
            logLine.append(" ");
            logLine.append(DetectedCorners[detectedCornerIndex*2]); // x
            logLine.append(" ");
            logLine.append(DetectedCorners[detectedCornerIndex*2+1]); // y
            logLine.append(" 0 "); // intended to be decimations always 0
            logLine.append(boardCornerId);
            logLine.append(" ");
            logLine.append(ChessboardCorners[detectedCornerIndex*2]); // x
            logLine.append(" ");
            logLine.append(ChessboardCorners[detectedCornerIndex*2+1]); // y
            vnlog.println(logLine.toString());                    
        }
        vnlog.flush();
    }
}
/*
partial sample final calibration output

2024-12-30 14:17:43.977 INFO    [ Guidance.UserGuidance update] image capture number 14 
2024-12-30 14:17:44.008 FINEST  [ Guidance.UserGuidance update] camera matrix
[904.7665459523686, 0, 601.9607576366333;
 0, 904.3826311013136, 414.9059682892109;
 0, 0, 1] 
2024-12-30 14:17:44.008 FINEST  [ Guidance.UserGuidance update] camera distortion [0.0358651354893901, 0.02390999179756997, 0.0005797443857783846, -0.001179131497828185, -0.1237528713726453] 

2024-12-30 14:17:44.035 CONFIG  [ Guidance.UserGuidance write] Camera Calibration Data 
2024-12-30 14:17:44.035 CONFIG  [ Guidance.UserGuidance write] nr_of_frames: 14 
2024-12-30 14:17:44.035 CONFIG  [ Guidance.UserGuidance write] image_width: 1280.0 
2024-12-30 14:17:44.035 CONFIG  [ Guidance.UserGuidance write] image_height: 800.0 
2024-12-30 14:17:44.035 CONFIG  [ Guidance.UserGuidance write] board_width: 8.0 
2024-12-30 14:17:44.035 CONFIG  [ Guidance.UserGuidance write] board_height: 8.0 
2024-12-30 14:17:44.035 CONFIG  [ Guidance.UserGuidance write] square_size: 250 
2024-12-30 14:17:44.035 CONFIG  [ Guidance.UserGuidance write] marker_size: 188 
2024-12-30 14:17:44.035 CONFIG  [ Guidance.UserGuidance write] fisheye_model: 0 
2024-12-30 14:17:44.035 CONFIG  [ Guidance.UserGuidance write] camera_matrix:
[904.7665459523686, 0, 601.9607576366333;
 0, 904.3826311013136, 414.9059682892109;
 0, 0, 1] 
2024-12-30 14:17:44.035 CONFIG  [ Guidance.UserGuidance write] distortion_coefficients:
[0.0358651354893901, 0.02390999179756997, 0.0005797443857783846, -0.001179131497828185, -0.1237528713726453] 
2024-12-30 14:17:44.035 CONFIG  [ Guidance.UserGuidance write] avg_reprojection_error: 0.41918087385668834 
2024-12-30 14:17:44.035 CONFIG  [ Guidance.UserGuidance write] End of Calibration 
 */

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  parking lot for PV instructions from Matt (not this Guidance program instructions)
// https://github.com/mcm001/photonvision/tree/2023-10-30_pose_calib_integration
//  I made this by running 
// gradlew clean
//  then for RPi
// gradlew shadowjar -PArchOverride=linuxarm64
//  or for Windows
// gradlew shadowjar

//  inside the photonvision project's root directory
//  that spits the jar out into photon-server/build/libs
//  you should be able to stop the RPi photonvision service with 
// sudo service photonvision stop
//  and then 
// java -jar photonvision-dev-v2024.1.1-beta-3.1-5-ga99e85a8-linuxarm64.jar
//  is all you should need

// Disable spotless in VSCode extensions or Append "-x spotlessapply" to the
// commands you run to disable it
