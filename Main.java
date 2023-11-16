// This project is derived in part from the "Pose Calib" project by
// @author Pavel Rojtberg
// It is subject to his license terms in the LICENSE file.

//FIXME detector needs at least 6 points to keep solvePnP happy. UserGuidance needs 15 or more. Why not just one min?

package org.photonvision.calibrator;

// changed method name oribital_pose to orbital_pose
// didn't change some other misspellings
// changed a couple of other "bugs" (maybe, I think)
// used current OpenCV methods. Some old ones were removed.
// didn't convert some unused methods and variables
// dry run not implemented
// used OpenCV variances (standard deviations) for intrinsics instead of the Jacobian manipulations
// sampled distortion map with step = 1 (full map - not sampled) not implemented
// added clone to this.get_pts3d()
// changed K and Knew usage in create_maps - likely had misused and trashed the K variable
// modified calibration flags usage for early poses
// changed (corrected?, I am hopeful) pose_from_bounds scaling of rotated

import java.io.BufferedOutputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.lang.invoke.MethodHandles;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Scanner;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;

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
    private static final String VERSION = "beta 9"; // change this
    
    static
    {
        System.out.println("Starting class: " + MethodHandles.lookup().lookupClass().getCanonicalName() + " version " + VERSION);
        System.err.println("Starting class: " + MethodHandles.lookup().lookupClass().getCanonicalName() + " version " + VERSION);
    }
    
    static
    {
        if (Cfg.isPV)
        {
            org.photonvision.common.util.TestUtils.loadLibraries();
        }
        else
        {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME); // Load the native OpenCV library
        }
    }

    // private static PrintWriter pw; // K debugging
    // private static int counter = 0; // K debugging
    static Mat progressInsert = new Mat(); // testing only

    // LOGGER STUFF
    static final Logger LOGGER = Logger.getLogger("");
    private static final String outFormat = "%7$s%4$-7s [%3$s %2$s] %5$s %6$s%n";
    private static final String outTail = "THE END OF THE LOG\n";
    private static final String errFormat = "%7$s%1$tY-%1$tm-%1$td %1$tH:%1$tM:%1$tS.%1$tL %4$-7s [%3$s %2$s] %5$s %6$s%n";
    private static final String errTail = "THE END OF THE LOG\n";

    // normally don't change header but it's here (or below) to use version
    private static final String header = "\n\nStarting Log for Camera Calibration Program Version " + VERSION
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
    private static final String[] classesLog = { 
        "org.photonvision.calibrator.PoseGeneratorDist",
        "org.photonvision.calibrator.BoardPreview",
        "org.photonvision.calibrator.Distortion",
        "org.photonvision.calibrator.ChArucoDetector",
        "org.photonvision.calibrator.UserGuidance",
        "org.photonvision.calibrator.Calibrator",
        "org.photonvision.calibrator.Cfg",
        "org.photonvision.calibrator.Main"
        };
    static final Map<String, Level> classLevels = new HashMap<String, Level>(15);
    static { 
    // set all minimum log levels to display the same easily
    // java.util.logging Levels	ALL FINEST FINER	FINE	INFO	CONFIG  WARNING	SEVERE	OFF
    for (String key : classesLog)
        {
        String value = "ALL";
        classLevels.put(key, Level.parse(value));
        }
    }
    private static int frameNumber = 0;
    static String frame = "00000 ";
    // END LOGGER STUFF

    static boolean fewCorners = true;

    // keyboard mapping returns from waitKey
    private static final int keyTerminate = 81;
    private static final int keyCapture = 67;
    private static final int keyMirrorToggle = 77;
    private static final int keyTerminateScanner = 113;
    private static final int keyCaptureScanner = 99;
    private static final int keyMirrorToggleScanner = 109;
    private static final int timedOut = -1;  // timed out no key pressed
  
    // Java Scanner alternative to OpenCV keyboard usage that is not in PV headless (AWT is missing)
    AtomicInteger dokeystroke = new AtomicInteger(-1);
    class Keystroke implements Runnable
    {
        public void run()
        {
            try (Scanner keyboard = new Scanner(System.in))
            {
                while( ! Thread.interrupted())
                {
                    System.out.println("Pose should auto capture otherwise, press c (capture), m (mirror), q (quit) then the Enter key");
                    String entered = keyboard.next();
                    int keyScanner = entered.charAt(0);
                    // map Scanner character codes to OpenCV character codes
                    if (keyScanner == keyCaptureScanner)
                    {
                        dokeystroke.set(keyCapture);
                    }
                    if (keyScanner == keyMirrorToggleScanner)
                    {
                        dokeystroke.set(keyMirrorToggle);
                    }
                    if (keyScanner == keyTerminateScanner)
                    {
                        dokeystroke.set(keyTerminate);
                    }
                }
            } catch(Exception e) {Main.LOGGER.log(Level.SEVERE,
                "Terminal keyboard closed (Ctrl-c) or doesn't exist (jar file not run from command line)", e);}
            // keyboard.close();
        }
    }

    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}
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

        final var options = new Options();
        options.addOption("h", "help", false, "Show this help text and exit");
        options.addOption("width", true, "camera image width (1280)");
        options.addOption("height", true, "camera image height (720)");
        options.addOption("dpmX", true, "print width pixels per meter (9843=250 DPI)");
        options.addOption("dpmY", true, "print height pixels per meter (9843=250 DPI");
        options.addOption("pxFmt", true, "pixel format (kYUYV) " + Arrays.toString(PixelFormat.values()));
        options.addOption("cameraID", true, "camera id (0)");
        CommandLineParser parser = new DefaultParser();
        CommandLine cmd = parser.parse(options, args);

        if (cmd.hasOption("help")) {
            HelpFormatter formatter = new HelpFormatter();
            formatter.printHelp("java -jar <your jar file>.jar [options]", options);
            return false; // exit program
        }
        else
        if ( cmd.hasOption("width")) {
            Cfg.image_width = Integer.parseInt(cmd.getOptionValue("width"));
        }
        else
        if (cmd.hasOption("height")) {
            Cfg.image_height = Integer.parseInt(cmd.getOptionValue("height"));
        }
        else
        if (cmd.hasOption("dpmX")) {
            Cfg.resXDPM = Integer.parseInt(cmd.getOptionValue("dpmX"));
        }
        else
        if (cmd.hasOption("dpmY")) {
            Cfg.resYDPM = Integer.parseInt(cmd.getOptionValue("dpmY"));
        }
        else
        if (cmd.hasOption("pxFmt")) {
            Cfg.pixelFormat = PixelFormat.valueOf(cmd.getOptionValue("pxFmt"));
        }
        else
        if (cmd.hasOption("cameraId")) {
            Cfg.camId = Integer.parseInt(cmd.getOptionValue("cameraId"));
        }

        return true;
    }
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
        Main MainInstance;
        Keystroke keystroke;
        Thread keyboardThread;

        CvSource networkDisplay;
        MjpegServer mjpegServer;

        OutputStream copySystemErr = System.err; // initialize System.err duplicated stream to just the err
        // add the file that is a running duplicate of System.err
        try { // create a stream with the 2 substreams
            copySystemErr = new BufferedOutputStream(new FileOutputStream(Cfg.logFile, true));
            TeePrintStream errStreamCopied = new TeePrintStream(
                                System.err,
                                copySystemErr,
                                true);
            System.setErr(errStreamCopied); // use the duplicating stream as System.err
        } catch (IOException e) {
            e.printStackTrace();
        }

        Loggers.setupLoggers(copySystemErr, outFormat, outHeader, outTail, outLevel, errFormat, errHeader, errTail, errLevel);
        Main.LOGGER.log(Level.SEVERE, "logs accumulate in file " + Cfg.logFile);

        // pw = new PrintWriter("K.csv");

        try {
            if ( ! handleArgs(args)) {
                System.exit(0);
            }
        } catch (ParseException e) {
            LOGGER.log(Level.SEVERE, "Failed to parse command-line options!", e);
        }

        // keyboard handler for PV using the web interface
        if (Cfg.isPV)
        {
            MainInstance = new Main();
            keystroke = MainInstance.new Keystroke();
            keyboardThread = new Thread(keystroke, "keys");
            keyboardThread.setDaemon(true);
            keyboardThread.start();

            networkDisplay = new CvSource("calibPV", /*VideoMode.*/PixelFormat.kMJPEG,
                    Cfg.image_height, Cfg.image_width, 30);
            mjpegServer = new MjpegServer("serve_DriverView", 1185);
            mjpegServer.setSource(networkDisplay);
            // MjpegServer openCVserver = CameraServer.getInstance().startAutomaticCapture(outputStream);
        }

        if ( ! Cfg.isPV) // PV has its own way to get these libraries */
        {
        NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
        WPIMathJNI.Helper.setExtractOnStaticLoad(false);
        CameraServerJNI.Helper.setExtractOnStaticLoad(false);
        CombinedRuntimeLoader.loadLibraries(Main.class, "wpiutiljni", "wpimathjni", "ntcorejni", "cscorejnicvstatic");
        }
        // var inst = NetworkTableInstance.getDefault(); // not using NT in this program

        /** video image capture setup **/
        // Get the UsbCamera from CameraServer
        final UsbCamera camera = CameraServer.startAutomaticCapture(Cfg.camId);

        camera.setPixelFormat(Cfg.pixelFormat);

        camera.setResolution(Cfg.image_width, Cfg.image_height);
        // camera.setExposureAuto();
        camera.setExposureManual(70);
        camera.setBrightness(70);
        camera.setFPS(30);
        // Get a CvSink. This will capture Mats from the camera
        JavaCvSink cap = new JavaCvSink("sink1");
        cap.setSource(camera);

        Mat _img = new Mat(); // this follows the camera input but ...
        Mat  img = new Mat(Cfg.image_height, Cfg.image_width, CvType.CV_8UC3); // set by user config - need camera to return this size, too
        /** end video image capture setup **/

        Mat out = new Mat(); // user display Mat

        ChArucoDetector tracker = new ChArucoDetector();
        UserGuidance ugui = new UserGuidance(tracker, Cfg.var_terminate);

        // runtime variables
        boolean mirror = false;
        boolean save = false; // indicator for user pressed the "c" key to capture (save) manually

        grabFrameLoop:
        while ( ! Thread.interrupted())
        {
            frameNumber++;
            frame = String.format("%05d ", frameNumber);
            if (frameNumber%Cfg.garbageCollectionFrames == 0) System.gc();

            boolean force = false;  // force add frame to calibration (no support yet still images else (force = !live)

            long status = cap.grabFrame(_img, 0.5);
            if (status != 0)
            {
                if (_img.height() != Cfg.image_height || img.width() != Cfg.image_width) // enforce camera matches user spec for testing and no good camera setup
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
                force = false; // useless now with the continue below
                continue; // pretend frame never happened - rkt addition; original reprocessed previous frame
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
            
            if (ugui.converged()) // are we there yet?
            {
                ugui.write(); // write all the calibration data

                break grabFrameLoop; // the end - rkt addition; the original kept looping somehow
            }

            displayOverlay(out, ugui);

            int k;
            if (Cfg.isPV)
            {                    
            networkDisplay.putFrame(out);

            k = MainInstance.dokeystroke.getAndSet(timedOut);
            }
            else // use HighGuiX for Windows
            {
            HighGuiX.imshow("PoseCalibPV", out); // added PV to name to distinguish Java images from Python
            k = HighGuiX.waitKey(Cfg.wait);
            }

            if (k == timedOut)
            {
                continue; // no key press to process
            }
            
            // have a key
            switch (k)
            {
                case keyTerminate: // terminate key pressed to stop loop immediately
                        break grabFrameLoop;

                case keyMirrorToggle: // mirror/no mirror key pressed
                        mirror = ! mirror;
                        break;

                case keyCapture: // capture frame key pressed
                        save = true;
                        break;

                default: // unassigned key
                        break;
            }
        } // end grabFrameLoop

        Imgproc.putText(out, "CALIBRATED", new Point(50, 250), Imgproc.FONT_HERSHEY_SIMPLEX, 2.8, new Scalar(0, 0, 0), 5);
        Imgproc.putText(out, "CALIBRATED", new Point(50, 250), Imgproc.FONT_HERSHEY_SIMPLEX, 2.8, new Scalar(255, 255, 255), 3);
        Imgproc.putText(out, "CALIBRATED", new Point(50, 250), Imgproc.FONT_HERSHEY_SIMPLEX, 2.8, new Scalar(0, 255, 0), 2);
        if (Cfg.isPV)
        {         
            networkDisplay.putFrame(out);
        }
        else // use HighGuiX for Windows
        {
            HighGuiX.imshow("PoseCalibPV", out); // added PV to name to distinguish Java images from Python
            HighGuiX.waitKey(5000);
        }
        // ugui.write(); // temp just to see what comes out even if we don't make it to the converged end
        // pw.close(); // K debugging

        Main.LOGGER.log(Level.CONFIG,"End of running main");
        System.exit(0);
    }   
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     displayOverlay                                              */
/*                                     displayOverlay                                              */
/*                                     displayOverlay                                              */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    public static void displayOverlay(Mat out, UserGuidance ugui)
    {
        Imgproc.putText(out, Main.frame, new Point(0, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(0, 0, 0), 2);
        Imgproc.putText(out, Main.frame, new Point(0, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(255, 255, 255), 1);

        String message = "";

        if (fewCorners)
        {
            message = "moving or bad aim\n";
            fewCorners = false;
        }
        
        if (ugui.user_info_text().length() > 0) // is there a message to display?
        {
            // if ( ! (ugui.user_info_text().equals("initialization"))) // stop spamming "initialization" to log
            // {
            //Main.LOGGER.log(Level.WARNING,ugui.user_info_text());
            // }
            message += ugui.user_info_text();
        }

        if (message.length() > 0)
        {
            Imgproc.putText(out, message, new Point(80, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(0, 0, 0), 2);
            Imgproc.putText(out, message, new Point(80, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(255, 255, 255), 1);
        } 

        // //FIXME these guidance pose angles aren't right; bad conversion from tgt_r for some reason
        // // maybe save the original angles from pose gen so they are butchered before getting here.
        // Mat dst = new Mat();
        // Calib3d.Rodrigues(ugui.tgt_r(), dst);
        // double[] euler = Calib3d.RQDecomp3x3(dst, new Mat(), new Mat()); // always returns euler.length = 3

        // Imgproc.putText(out, String.format("%4.0f %4.0f %4.0f ", euler[0], euler[1], euler[2]) + ugui.tgt_t().dump(), new Point(0, 40), Imgproc.FONT_HERSHEY_SIMPLEX, .6, new Scalar(0, 0, 0), 2);
        // Imgproc.putText(out, String.format("%4.0f %4.0f %4.0f ", euler[0], euler[1], euler[2]) + ugui.tgt_t().dump(), new Point(0, 40), Imgproc.FONT_HERSHEY_SIMPLEX, .6, new Scalar(255, 255, 255), 1);
        
        // write a frame to a file name java<frame nbr>.jpg
        // final MatOfInt writeBoardParams = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, 100); // debugging - pair-wise; param1, value1, ...
        // Imgcodecs.imwrite("java" + frame + ".jpg", out); // debugging - save image in jpg file
        
        if ( ! progressInsert.empty())
        { // add to the display the board/camera overlap image
            Imgproc.resize(progressInsert, progressInsert, new Size(Cfg.image_width*0.1, Cfg.image_height*0.1), 0, 0, Imgproc.INTER_CUBIC);
            List<Mat> temp1 = new ArrayList<>(3); // make the 1 b&w channel into 3 channels
            temp1.add(progressInsert);
            temp1.add(progressInsert);
            temp1.add(progressInsert);
            Mat temp2 = new Mat();
            Core.merge(temp1, temp2);
            Imgproc.rectangle(temp2, // outline the insert for better visibility
                new Point(0, 0),
                new Point(progressInsert.cols()-1., progressInsert.rows()-1.),
                new Scalar(255., 255., 0.), 1);
            temp2.copyTo(out.submat((int)(Cfg.image_height*0.45), (int)(Cfg.image_height*0.45)+progressInsert.rows(), 0,progressInsert.cols()));
            temp2.release();
        }

        // display intrinsics convergence
        for (int i = 0; i < 9; i++)
        {
            Scalar color;
            if (ugui.pconverged()[i])
            {
                color = new Scalar(0, 190, 0);
            }
            else
            {
                color = new Scalar(0, 0, 255);
            }
            Imgproc.rectangle(out, new Point((double)i*20,Cfg.image_height*0.4), new Point((double)(i+1)*20, Cfg.image_height*0.4+20), color, Imgproc.FILLED);
            Imgproc.putText(out, ugui.INTRINSICS()[i], new Point((double)i*20, Cfg.image_height*0.4+15), Imgproc.FONT_HERSHEY_SIMPLEX, .4, new Scalar(255, 255, 255), 1);
        }
    }
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     Kcsv                                                        */
/*                                     Kcsv                                                        */
/*                                     Kcsv                                                        */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/**
 * Print the K camera matrix Mat to a file defined in variable "pw" in CSV format.
 * [500, 0, 319.75;
 * 0, 666.6666666666666, 239.6666666666667;
 * 0, 0, 1]
 * 00000 , "BoardPreview.java@166", 500, 0, 319.75, 0, 666.6666666666666, 239.6666666666667, 0, 0, 1, 4
 * Used for debugging.
 * Requires class variables:
 *   private static PrintWriter pw;  // print writer file name
 *   private static int counter = 0; // sequence number
 * 
 *   pw = new PrintWriter("K.csv");  // define the file - statement may throw an exception to be handled somehow
 * 
 *   pw.close(); // forces last line to be completed, too
 * @param line Input "comment" line that could be used to identify the location in the program
 * @param K Input 3x3 camera matrix Mat. Doesn't have to be 3x3 but assumptions are made about changing the "[];\n"
 */
    // public static void Kcsv(String line, Mat K)
    // {
    //     counter++;
    //     if (counter == 1) // first time switch for columns' header
    //     {
    //         Main.pw.println("frame, line, fx, 0, cx, 0, fy, cy, row3_1is0, row3_2is0, row3_3is1, sequence"); // K's column names
    //     }
    //     String Kdump = K.dump();
    //     Kdump = Kdump.replace("[", "").replace("]", "").replace(";", ",").replace("\n", "");
    //     Main.pw.println(Main.frame + ", \"" + line + "\", " + Kdump + ", " + counter);
    // }
}
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     End Main class                                              */
/*                                     End Main class                                              */
/*                                     End Main class                                              */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/

// Parking lot

// Logger.getLogger("").setLevel(Level.OFF); // turn off everything - especially java.awt fine, finer, finest spam
// Logger.getLogger("calibrator").setLevel(Level.ALL); // turn on for my package

// System.loadLibrary("opencv_videoio_ffmpeg480_64");

/*

C:\Users\Public\wpilib\2023\jdk\bin\java.exe -jar C:\Users\RKT\frc\FRC2023\Code\Java\Java.jar

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

//   Java Camera Calibrator
// run from a terminal window the photonvision-dev-calib-winx64.jar (Windows PC) or photonvision-dev-calib-linuxarm64.jar (RPi, etc.)
// java -jar [options]
// Options are listed with -help, for example, on a Windows PC: java -jar photonvision-dev-calib-winx64.jar -help
// Options included for changing the numeric camera Id, camera pixel format, wide, height, ChArUcoBoard printing size.
// Run the program to make a ChArUcoBoard.png file that can be printed.
// Run the program and aim the camera at the printed board in the pose that matches the guidance board on the computer
//  screen. It may be easier to align a fixed camera and hold and move the baord.
// // If the guidance board and the camera image match, the program should auto-capture that information. The (lack of
//  always) auto capture leaves something to be desired and the user can force capture by pressing "c" and Enter on the
//   computer terminal window that was used to start the program. I suggest pressing 'c' after the program starts and it's
//    ready for a press Enter when the poses align but failed to auto capture. The black and white insert shows what the
//     poses are of the guidance board (exactly right) and the estimated camera view (not always right - want to help me
//      get this better? It has to do with how the program is dynamically adjusting the estimated camera matrix).
// // The nine red camera intrinsic parameters turn green when the poses provide enough information. Usually after about
//  15 carefully aligned poses.
// // Other terminal (keyboard) input are 'm' for mirror view if that helps you align the camera to the guidance and 'q'
//  to quit.
// // The display of the guidance board and camera view are on a browser's port 1185. For example,
//  127.0.0.1:1185?action=stream or just 127.0.0.1:1185 to see the camera parameters, too. (This is standard WPILib
//   camera server stuff so you can adust your camera parameters there.)
// If you run this on a system with PhotonVision running then stop PhotonVision. (linux command is sudo service
//  photonvision stop)
// References:
// https://arxiv.org/pdf/1907.04096.pdf
// https://www.calibdb.net/#
// https://github.com/paroj/pose_calib

//  https://github.com/mcm001/photonvision/tree/2023-10-30_pose_calib_integration
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
