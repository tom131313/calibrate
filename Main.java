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
import org.opencv.calib3d.Calib3d;
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
    private static final String VERSION = "beta 11.4"; // change this
    
    static
    {
        System.out.println("Starting class: " + MethodHandles.lookup().lookupClass().getCanonicalName() + " version " + VERSION);
        System.err.println("Starting class: " + MethodHandles.lookup().lookupClass().getCanonicalName() + " version " + VERSION);
    }
    
    // private static PrintWriter pw; // K debugging
    // private static int counter = 0; // K debugging
    static Mat progressInsert;

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
        options.addOption("isPV", true, "using PV environment (true)");
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
        else
        if( cmd.hasOption("isPV")) {
            Cfg.isPV = Boolean.parseBoolean(cmd.getOptionValue("isPV"));
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
        Main MainInstance = null;
        Keystroke keystroke;
        Thread keyboardThread;

        CvSource networkDisplay = null;
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

        try {
            if ( ! handleArgs(args)) {
                System.exit(0);
            }
        } catch (ParseException e) {
            LOGGER.log(Level.SEVERE, "Failed to parse command-line options!", e);
        }

        if (Cfg.isPV)
        {
            org.photonvision.common.util.TestUtils.loadLibraries();
        }
        else
        {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME); // Load the native OpenCV library
        }

        progressInsert = new Mat();
        
        // keyboard handler for PV using the web interface
        if (Cfg.isPV)
        {
            MainInstance = new Main();
            keystroke = MainInstance.new Keystroke();
            keyboardThread = new Thread(keystroke, "keys");
            keyboardThread.setDaemon(true);
            keyboardThread.start();

            networkDisplay = new CvSource("calibPV", /*VideoMode.*/PixelFormat.kMJPEG,
                    Cfg.image_height, Cfg.image_width, Cfg.fps);
            mjpegServer = new MjpegServer("GuidanceView", Cfg.displayPort);
            LOGGER.log(Level.SEVERE, "View Guidance Board On Port " + Cfg.displayPort);
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
        LOGGER.log(Level.SEVERE, "camera parameters can be seen or changed on port 1181 or higher");

        try
        {
            camera.setPixelFormat(Cfg.pixelFormat);
            camera.setResolution(Cfg.image_width, Cfg.image_height);
            // camera.setExposureAuto();
            camera.setExposureManual(Cfg.exposureManual);
            camera.setBrightness(Cfg.brightness);
            camera.setFPS(Cfg.fps);
        }
        catch(Exception e)
        {
            Main.LOGGER.log(Level.SEVERE, "camera setup error", e);
        }

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
        String endMessage = "logic error"; // status of calibration at the end

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
            
            displayOverlay(out, ugui);

            int k;
            if (Cfg.isPV)
            {                    
                networkDisplay.putFrame(out);
                k = MainInstance.dokeystroke.getAndSet(timedOut);
            }
            else
            {
                HighGuiX.imshow("PoseCalibPV", out); // added PV to name to distinguish Java images from Python
                k = HighGuiX.waitKey(Cfg.wait);
            }

            if (ugui.converged()) // are we there yet?
            {
                ugui.write(); // write all the calibration data

                endMessage = "CALIBRATED";

                break grabFrameLoop; // the end - rkt addition; the original kept looping somehow
            }

            if (k == timedOut)
            {
                continue; // no key press to process
            }
            
            // have a key
            switch (k)
            {
                case keyTerminate: // terminate key pressed to stop loop immediately
                        endMessage = "CANCELLED";
                        
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

        Imgproc.putText(out, endMessage, new Point(50, 250), Imgproc.FONT_HERSHEY_SIMPLEX, 2.8, new Scalar(0, 0, 0), 5);
        Imgproc.putText(out, endMessage, new Point(50, 250), Imgproc.FONT_HERSHEY_SIMPLEX, 2.8, new Scalar(255, 255, 255), 3);
        Imgproc.putText(out, endMessage, new Point(50, 250), Imgproc.FONT_HERSHEY_SIMPLEX, 2.8, new Scalar(255, 255, 0), 2);
        if (Cfg.isPV)
        {
            for (int runOut = 0; runOut < 10; runOut++) // last frame won't display so repeat it a bunch of times to see it; q lags these 2 seconds
            {
                networkDisplay.putFrame(out);
                Thread.sleep(200L);
            }
            networkDisplay.close();
        }
        else
        {
            HighGuiX.imshow("PoseCalibPV", out); // added PV to name to distinguish Java images from Python
            HighGuiX.waitKey(5000);
        }
        // ugui.write(); // temp just to see what comes out even if we don't make it to the converged end

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

        if (ugui.user_info_text().length() > 0) // is there a message to display?
        {
            // if ( ! (ugui.user_info_text().equals("initialization"))) // stop spamming "initialization" to log
            // {
            // Main.LOGGER.log(Level.WARNING,ugui.user_info_text());
            // }
            String message1 = ugui.user_info_text();
            Imgproc.putText(out, message1, new Point(80, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(0, 0, 0), 2);
            Imgproc.putText(out, message1, new Point(80, 20), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(255, 255, 255), 1);
        } 

        if (fewCorners)
        {
            String message2 = "moving or bad aim\n";
            Imgproc.putText(out, message2, new Point(80, 40), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(0, 0, 0), 2);
            Imgproc.putText(out, message2, new Point(80, 40), Imgproc.FONT_HERSHEY_SIMPLEX, .8, new Scalar(255, 255, 255), 1);
            fewCorners = false;
        }

        // ugui.tgt_r() ugui.tgt_t guidance board target rotation and translation from pose generation
        Mat rotationMatrix = new Mat();
        double[] rotationDegrees;
        double[] translation = new double[3];
        
        // if Guidance Board has a pose then display it (end of program is missing this pose)
        if ( ! ugui.tgt_r().empty() && ! ugui.tgt_t().empty())
        {
            Calib3d.Rodrigues(ugui.tgt_r(), rotationMatrix);
            rotationDegrees = Calib3d.RQDecomp3x3(rotationMatrix, new Mat(), new Mat()); // always returns reuler.length = 3
            rotationDegrees[0] -= 180.;

            ugui.tgt_t().get(0, 0, translation);

            Imgproc.putText(out, String.format("r{%4.0f %4.0f %4.0f} t{%4.0f %4.0f %4.0f}Guidance",
                rotationDegrees[0], rotationDegrees[1], rotationDegrees[2], translation[0], translation[1], translation[2]),
                new Point(250, 60), Imgproc.FONT_HERSHEY_SIMPLEX, .6, new Scalar(0, 0, 0), 2);
            Imgproc.putText(out, String.format("r{%4.0f %4.0f %4.0f} t{%4.0f %4.0f %4.0f}Guidance",
                rotationDegrees[0], rotationDegrees[1], rotationDegrees[2], translation[0], translation[1], translation[2]),
                new Point(250, 60), Imgproc.FONT_HERSHEY_SIMPLEX, .6, new Scalar(255, 255, 255), 1);
        }

        // if camera has a ChArUco Board pose then display it (if camera not on target this pose is missing)
        if ( ! ugui.tracker.rvec().empty() && ! ugui.tracker.tvec().empty())
        {
            Calib3d.Rodrigues(ugui.tracker.rvec(), rotationMatrix);
            rotationDegrees = Calib3d.RQDecomp3x3(rotationMatrix, new Mat(), new Mat()); // always returns reuler.length = 3
            rotationDegrees[1] = -rotationDegrees[1];
            rotationDegrees[2] = -rotationDegrees[2];

            ugui.tracker.tvec().get(0, 0, translation);
            translation[1] = -translation[1];
            translation[0] = (double)(((int)(translation[0])+5)/10*10);
            translation[1] = (double)(((int)(translation[1])+5)/10*10);
            translation[2] = (double)(((int)(translation[2])+5)/10*10);
            Imgproc.putText(out, String.format("r{%4.0f %4.0f %4.0f} t{%4.0f %4.0f %4.0fCamera",
                rotationDegrees[0], rotationDegrees[1], rotationDegrees[2], translation[0], translation[1], translation[2]),
                new Point(250, 80), Imgproc.FONT_HERSHEY_SIMPLEX, .6, new Scalar(0, 0, 0), 2);
            Imgproc.putText(out, String.format("r{%4.0f %4.0f %4.0f} t{%4.0f %4.0f %4.0f}Camera",
                rotationDegrees[0], rotationDegrees[1], rotationDegrees[2], translation[0], translation[1], translation[2]),
                new Point(250, 80), Imgproc.FONT_HERSHEY_SIMPLEX, .6, new Scalar(255, 255, 255), 1);
        }

        // write a frame to a file name java<frame nbr>.jpg
        // final MatOfInt writeBoardParams = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, 100); // debugging - pair-wise; param1, value1, ...
        // Imgcodecs.imwrite("java" + frame + ".jpg", out); // debugging - save image in jpg file
        
        if ( ! progressInsert.empty())
        {
            // add to the display the board/camera overlap image
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

            Imgproc.putText(out,
                String.format("similar%5.2f/%4.2f", ugui.pose_close_to_tgt_get(), Cfg.pose_close_to_tgt_min),
                new Point(0,(int)(Cfg.image_height*0.45)+progressInsert.rows()+20) , Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
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
            Imgproc.putText(out, ugui.INTRINSICS()[i],
                new Point((double)i*20, Cfg.image_height*0.4+15),
                Imgproc.FONT_HERSHEY_SIMPLEX, .4, new Scalar(255, 255, 255), 1);
        }
    }


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

// testit();

//     static void testit()
//     {
//         Mat display = Mat.ones(600, 800, CvType.CV_8UC1);
//         Core.multiply(display, new Scalar(110.), display);
//         Mat object = display.submat(200, 400, 250, 550);
//         Mat.ones(200, 300, CvType.CV_8UC1).copyTo(object);
//         Core.multiply(object, new Scalar(170.), object);

//         //! [find-corners]
//         MatOfPoint2f cornersObject = new MatOfPoint2f(
//             new Point(0, 0),
//             new Point(object.cols(), 0),
//             new Point(object.cols(), object.rows()),
//             new Point(0, object.rows()));
//         System.out.println(cornersObject.dump());

//         MatOfPoint2f cornersWarped = new MatOfPoint2f(
//             new Point(30, 20),
//             new Point(object.cols()-70, 40),
//             new Point(object.cols()-80, object.rows()-50),
//             new Point(10, object.rows()-40));
//         System.out.println(cornersWarped.dump());

//         MatOfPoint3f cube = new MatOfPoint3f(
//             new Point3(0, 0, 0),
//             new Point3(1, 0, 0),
//             new Point3(1, 1, 0),
//             new Point3(0, 1, 0),
//             new Point3(0, 0, 1),
//             new Point3(1, 0, 1),
//             new Point3(1, 1, 1),
//             new Point3(0, 1, 1));

//         MatOfPoint3f  cubeWarped = new MatOfPoint3f();

//         Calib3d.projectPoints(cubeWarped, cornersObject, cube, cubeWarped, null, cornersWarped);

//         //! [estimate-homography]
//         // actual  H = K*R*inv(K) or estimate frompoints
//         Mat H = Mat.eye(4, 4, CvType.CV_64FC1);
//         Mat Htemp = new Mat();
//         Htemp = Calib3d.findHomography(cornersObject, cornersWarped);
//         Htemp.copyTo(H.submat(0, 3, 0, 3));
//         System.out.println(H + " " + H.dump());
//         System.out.println(Htemp + " " + Htemp.dump());
        
// //check for empty Htemp
// // Calib3d.decomposeHomographyMat();
//         // derive rotation angle from homography
//         double theta = - Math.atan2(H.get(0,1)[0], H.get(0,0)[0]) * 180. / Math.PI;
//         System.out.println("theta " + theta);

//         // Calib3d.Rodrigues(r, dst);
//         // double[] reuler = Calib3d.RQDecomp3x3(dst, mtxR, mtxQ); // always returns reuler.length = 3

//         //  getPerspectiveTransform the same answer as findHomography but getPerspectiveTransform doesn't have the extended options so use findHomography
//         // Mat T = new Mat();
//         // T = Imgproc.getPerspectiveTransform(cornersObject, cornersWarped);
//         // System.out.println(T.dump());
//         //! [estimate-homography]

//         //! [warp-chessboard]
//         Mat objectWarped = new Mat();
//         Mat cornersWarpedAlt = new Mat();
//         Imgproc.warpPerspective(object, objectWarped, H.submat(0, 3, 0, 3), object.size()); // images
//         Core.perspectiveTransform(cube, cubeWarped, H); // points
//         System.out.println(cubeWarped.dump());
//         //! [warp-chessboard]

//         // change 3x3 to to 4x4
//         Core.perspectiveTransform(cube, cubeWarped, H); // points
//         /*
//          * perspectiveTransform transforms points
//          *  If you want to transform an image using perspective transformation, use warpPerspective .
//          *  If you have an inverse problem, that is, you want to compute the most probable perspective transformation
//          *  out of several pairs of corresponding points, you can use getPerspectiveTransform or findHomography
//          */

//         MatOfPoint3f cornersWarped3d = new MatOfPoint3f(); // 3d points; Z = 0 added to the 2d to make 3d
        
//         Calib3d.convertPointsToHomogeneous(cornersWarped, cornersWarped3d); // now convert 2d to 3d homogeneous
//         System.out.println(cornersWarped3d.dump());


//         Mat img_draw_matches = new Mat();
//         List<Mat> list1 = new ArrayList<>(), list2 = new ArrayList<>() ;
//         list1.add(object);
//         list1.add(objectWarped);
//         Core.hconcat(list1, img_draw_matches); // side-by-side display
//         // HighGuiX.imshow("Desired view / Warped view", img_draw_warp);


//         // //FIXME these guidance pose angles aren't right; bad conversion from tgt_r for some reason
//         // // maybe save the original angles from pose gen so they are butchered before getting here.
//         // Mat dst = new Mat();
//         // Calib3d.Rodrigues(ugui.tgt_r(), dst);
//         // double[] euler = Calib3d.RQDecomp3x3(dst, new Mat(), new Mat()); // always returns euler.length = 3

//         // Imgproc.putText(out, String.format("%4.0f %4.0f %4.0f ", euler[0], euler[1], euler[2]) + ugui.tgt_t().dump(), new Point(0, 40), Imgproc.FONT_HERSHEY_SIMPLEX, .6, new Scalar(0, 0, 0), 2);
//         // Imgproc.putText(out, String.format("%4.0f %4.0f %4.0f ", euler[0], euler[1], euler[2]) + ugui.tgt_t().dump(), new Point(0, 40), Imgproc.FONT_HERSHEY_SIMPLEX, .6, new Scalar(255, 255, 255), 1);


//         Point []corners1Arr = cornersObject.toArray();

//         for (int i = 0 ; i < corners1Arr.length; i++) {
//             Mat pt1 = new Mat(3, 1, CvType.CV_64FC1);
//             Mat pt2 = new Mat();
//             pt1.put(0, 0, corners1Arr[i].x, corners1Arr[i].y, 1 );

//             Core.gemm(H.submat(0, 3, 0, 3), pt1, 1, new Mat(), 0, pt2);
//             double[] data = pt2.get(2, 0);
//             Core.divide(pt2, new Scalar(data[0]), pt2);

//             double[] data1 =pt2.get(0, 0);
//             double[] data2 = pt2.get(1, 0);
//             Point end = new Point((int)(object.cols()+ data1[0]), (int)data2[0]);
//             Imgproc.line(img_draw_matches, corners1Arr[i], end,  new Scalar(255, 255, 255), 2);
//         }


//         // compute rotation matrix from rotation vector
//         double[] angleZ = {0., 0., Math.PI/4.};
//         double[] angleX = {Math.PI/4., 0., 0.};
//         double[] angleY = {0., Math.PI/4., 0.};
//         Mat angleZVector = new Mat(3, 1, CvType.CV_64FC1);
//         Mat angleXVector = new Mat(3, 1, CvType.CV_64FC1);
//         Mat angleYVector = new Mat(3, 1, CvType.CV_64FC1);
//         angleZVector.put(0, 0, angleZ);       
//         angleXVector.put(0, 0, angleX);
//         angleYVector.put(0, 0, angleY);
//         Mat Rz = new Mat();
//         Mat Rx = new Mat();
//         Mat Ry = new Mat();

//         /**************************************************************************************** */
//         Calib3d.Rodrigues(angleZVector, Rz);
//         Calib3d.Rodrigues(angleXVector, Rx);
//         Calib3d.Rodrigues(angleYVector, Ry);
//         /**************************************************************************************** */

//         Main.LOGGER.log(Level.WARNING, "Rz\n" + Rz.dump());
//         Main.LOGGER.log(Level.WARNING, "Rx\n" + Rx.dump());
//         Main.LOGGER.log(Level.WARNING, "Ry\n" + Ry.dump());

//         // in Python (Ry).dot(Rx).dot(Rz) messed up nomenclature - it's often really matrix multiply Ry times Rx times Rz
//         Mat R = Mat.eye(4, 4, CvType.CV_64FC1);
//         Mat R3x3 = R.submat(0, 3, 0, 3);

//         /**************************************************************************************** */
//         Core.gemm(Ry, Rx, 1., new Mat(), 0, R3x3);
//         Core.gemm(R3x3, Rz, 1., new Mat(), 0., R3x3); // rotation matrix of the input Euler Angles [radians]
//         double[] euler = Calib3d.RQDecomp3x3(R3x3, new Mat(), new Mat());
//         System.out.println(Arrays.toString(euler));
//         /**************************************************************************************** */
        
//         Main.LOGGER.log(Level.SEVERE, "R\n" + R.dump());
//         Main.LOGGER.log(Level.SEVERE, "R3x3\n" + R3x3.dump());
      
//         // Calib3d.undistortPoints(p, p, K, cdist, new Mat(), new Mat(), Cfg.undistortPointsIterCriteria);


//         angleZVector.release();
//         angleXVector.release();
//         angleYVector.release();
//         Rz.release();
//         Rx.release();
//         Ry.release();

//         HighGuiX.imshow("Draw matches", img_draw_matches);

//         // HighGuiX.imshow("testit", objectWarped);
//         HighGuiX.waitKey(000);
//         System.exit(0);
//         // https://www.euclideanspace.com/maths/geometry/affine/matrix4x4/index.htm
//     }
/*
[0, 0;    
 300, 0;  
 300, 200;
 0, 200]  
[30, 20;
 230, 40;
 220, 150;
 10, 160]
Mat [ 4*4*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x22044a29970, dataAddr=0x22044a1f680 ] [0.8733333333333329, -0.1017391304347828, 30.00000000000008, 0;
 0.102608695652174, 0.6721739130434781, 20.00000000000001, 0;
 0.0008985507246376817, -0.0001739130434782618, 1, 0;
 0, 0, 0, 1]
Mat [ 3*3*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x22044a6ca10, dataAddr=0x2204484ef00 ] [0.8733333333333329, -0.1017391304347828, 30.00000000000008;
 0.102608695652174, 0.6721739130434781, 20.00000000000001;
 0.0008985507246376817, -0.0001739130434782618, 1]
[0, 0, 0;
 0.87333333, 0.1026087, 0.00089855073;
 0.77159423, 0.7747826, 0.00072463771;
 -0.10173913, 0.67217392, -0.00017391304;
 30, 20, 1;
 30.873333, 20.10261, 1.0008986;
 30.771595, 20.774782, 1.0007247;
 29.89826, 20.672174, 0.99982607]
[30, 20, 1;
 230, 40, 1;
 220, 150, 1;
 10, 160, 1]
 */

    // void perspectiveCorrection () {
    //     //! [find-corners]
    //     MatOfPoint2f corners1 = new MatOfPoint2f(), corners2 = new MatOfPoint2f();
    //     boolean found1 = Calib3d.findChessboardCorners(img1, new Size(9, 6), corners1 );
    //     boolean found2 = Calib3d.findChessboardCorners(img2, new Size(9, 6), corners2 );
    //     //! [find-corners]

    //     if (!found1 || !found2) {
    //         System.out.println("Error, cannot find the chessboard corners in both images.");
    //         return;
    //     }

    //     //! [estimate-homography]
    //     Mat H = new Mat();
    //     H = Calib3d.findHomography(corners1, corners2);
    //     System.out.println(H.dump());
    //     //! [estimate-homography]

    //     //! [warp-chessboard]
    //     Mat img1_warp = new Mat();
    //     Imgproc.warpPerspective(img1, img1_warp, H, img1.size());
    //     //! [warp-chessboard]

    //     Mat img_draw_warp = new Mat();
    //     List<Mat> list1 = new ArrayList<>(), list2 = new ArrayList<>() ;
    //     list1.add(img2);
    //     list1.add(img1_warp);
    //     Core.hconcat(list1, img_draw_warp);
    //     HighGuiX.imshow("Desired chessboard view / Warped source chessboard view", img_draw_warp);

    //     //! [compute-transformed-corners]
    //     Mat img_draw_matches = new Mat();
    //     list2.add(img1);
    //     list2.add(img2);
    //     Core.hconcat(list2, img_draw_matches);
    //     Point []corners1Arr = corners1.toArray();

    //     for (int i = 0 ; i < corners1Arr.length; i++) {
    //         Mat pt1 = new Mat(3, 1, CvType.CV_64FC1);
    //         Mat pt2 = new Mat();
    //         pt1.put(0, 0, corners1Arr[i].x, corners1Arr[i].y, 1 );

    //         Core.gemm(H, pt1, 1, new Mat(), 0, pt2);
    //         double[] data = pt2.get(2, 0);
    //         Core.divide(pt2, new Scalar(data[0]), pt2);

    //         double[] data1 =pt2.get(0, 0);
    //         double[] data2 = pt2.get(1, 0);
    //         Point end = new Point((int)(img1.cols()+ data1[0]), (int)data2[0]);
    //         Imgproc.line(img_draw_matches, corners1Arr[i], end,  new Scalar(0, 255, 255), 2);
    //     }

    //     HighGuiX.imshow("Draw matches", img_draw_matches);
    //     // HighGuiX.waitKey(0);
    //     //! [compute-transformed-corners]
    // }

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


// def order_points(pts):
// 	# initialzie a list of coordinates that will be ordered
// 	# such that the first entry in the list is the top-left,
// 	# the second entry is the top-right, the third is the
// 	# bottom-right, and the fourth is the bottom-left
// 	rect = np.zeros((4, 2), dtype = "float32")
// 	# the top-left point will have the smallest sum, whereas
// 	# the bottom-right point will have the largest sum
// 	s = pts.sum(axis = 1)
// 	rect[0] = pts[np.argmin(s)]
// 	rect[2] = pts[np.argmax(s)]
// 	# now, compute the difference between the points, the
// 	# top-right point will have the smallest difference,
// 	# whereas the bottom-left will have the largest difference
// 	diff = np.diff(pts, axis = 1)
// 	rect[1] = pts[np.argmin(diff)]
// 	rect[3] = pts[np.argmax(diff)]
// 	# return the ordered coordinates
// 	return rect