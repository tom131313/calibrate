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
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintWriter;
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
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoProperty;
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
    private static final String VERSION = "beta 12.3"; // change this
    
    static
    {
        System.out.println("Starting class: " + MethodHandles.lookup().lookupClass().getCanonicalName() + " version " + VERSION);
        System.err.println("Starting class: " + MethodHandles.lookup().lookupClass().getCanonicalName() + " version " + VERSION);
    }
    
    private static PrintWriter vnlog = null;
    
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
                "Terminal keyboard closed prematurely (Ctrl-c) or doesn't exist (jar file not run from command line; don't double click the jar to start it)", e);}
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
        options.addOption("cameraId", true, "camera id (0)");
        options.addOption("fps", true, "frames per second (10)");
        options.addOption("isPV", true, "using PV environment (true)");
        CommandLineParser parser = new DefaultParser();
        CommandLine cmd = parser.parse(options, args);

        if (cmd.hasOption("help")) {
            HelpFormatter formatter = new HelpFormatter();
            formatter.printHelp("java -jar <your jar file>.jar [options]", options);
            return false; // exit program
        }
        
        if ( cmd.hasOption("width")) {
            Cfg.image_width = Integer.parseInt(cmd.getOptionValue("width"));
        }
        
        if (cmd.hasOption("height")) {
            Cfg.image_height = Integer.parseInt(cmd.getOptionValue("height"));
        }
        
        if (cmd.hasOption("dpmX")) {
            Cfg.resXDPM = Integer.parseInt(cmd.getOptionValue("dpmX"));
        }
        
        if (cmd.hasOption("dpmY")) {
            Cfg.resYDPM = Integer.parseInt(cmd.getOptionValue("dpmY"));
        }
        
        if (cmd.hasOption("pxFmt")) {
            Cfg.pixelFormat = PixelFormat.valueOf(cmd.getOptionValue("pxFmt"));
        }
        
        if (cmd.hasOption("cameraId")) {
            Cfg.camId = Integer.parseInt(cmd.getOptionValue("cameraId"));
        }
        
        if (cmd.hasOption("fps")) {
            Cfg.fps = Integer.parseInt(cmd.getOptionValue("fps"));
        }
        
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
        Main.LOGGER.log(Level.CONFIG, "Command Line Args " + Arrays.toString(args));
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
        
        // keyboard handler for PV environment using the web interface
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
        }

        if ( ! Cfg.isPV) // PV is similar but loads more libraries */
        {
        NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
        WPIMathJNI.Helper.setExtractOnStaticLoad(false);
        CameraServerJNI.Helper.setExtractOnStaticLoad(false);
        CombinedRuntimeLoader.loadLibraries(Main.class, "wpiutiljni", "wpimathjni", "ntcorejni", "cscorejnicvstatic");
        }

        /** video image capture setup **/
        // Get the UsbCamera from CameraServer
        final UsbCamera camera = CameraServer.startAutomaticCapture(Cfg.camId); // gives access to camera parameters on port 181 or above

        // final UsbCamera camera = new UsbCamera("mycamera", Cfg.camId); // same camera as above but no interaction on port 181 or above

        for ( VideoMode vm : camera.enumerateVideoModes())
        {
                 Main.LOGGER.log(Level.CONFIG, "Camera mode choices " + vm.getPixelFormatFromInt(vm.pixelFormat.getValue()) + " " +
                + vm.width + "x" + vm.height + " " + vm.fps + " fps");
        }

        for ( VideoProperty vp : camera.enumerateProperties())
        {
            Main.LOGGER.log(Level.CONFIG, "camera property choices " + vp.get() + " " + vp.getName() + " " + VideoProperty.getKindFromInt(vp.get()));
        }

        VideoMode videoMode = new VideoMode(Cfg.pixelFormat, Cfg.image_width, Cfg.image_height, Cfg.fps);
        
        Main.LOGGER.log(Level.CONFIG, "Setting camera mode " + VideoMode.getPixelFormatFromInt(Cfg.pixelFormat.getValue()) + " " + Cfg.image_width + "x" + Cfg.image_height + " " + Cfg.fps + "fps");
            try {
                if ( ! camera.setVideoMode(videoMode)) throw new IllegalArgumentException("set video mode returned false");
            } catch (Exception e) {
                Main.LOGGER.log(Level.SEVERE, "camera set video mode error; mode is unchanged", e);
            }

        // camera.setExposureAuto();
        // camera.setExposureManual(Cfg.exposureManual);
        // camera.setBrightness(Cfg.brightness);
        // int cameraHandle = camera.getHandle();
        // CameraServerJNI.setProperty(CameraServerJNI.getSourceProperty(cameraHandle, "contrast"), 74);

        LOGGER.log(Level.SEVERE, "camera " + Cfg.camId + " properties can be seen and changed on port 1181 or higher");
        // Get a CvSink. This will capture Mats from the camera
        // CvSink cap = CameraServer.getVideo(camera); // typical CvSink

        JavaCvSink cap = new JavaCvSink("sink1"); // 2023 standalone WPILib way since there was no CvSink in that distribution
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
                if (_img.height() != Cfg.image_height || _img.width() != Cfg.image_width) // enforce camera matches user spec for testing and no good camera setup
                {
                    // Imgproc.resize(_img, _img, new Size(Cfg.image_width, Cfg.image_height), 0, 0, Imgproc.INTER_CUBIC);
                    Main.LOGGER.log(Level.SEVERE, "image incorrect " + _img.width() + "x" + _img.height()
                                                    + " should be " + Cfg.image_width + "x" + Cfg.image_height + " - ignoring it");
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

            boolean capturedPose = ugui.update(force); // calibrate
            
            // showPose(tracker, ugui); // pose cartoon stuff under development

            if (capturedPose && Cfg.isLogDetectedCorners)
            {
                logDetectedCorners(img, ugui);
            }

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
        if (vnlog != null)
        {
            vnlog.close();
        }
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
            // translation[1] = -translation[1];
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
        // Imgcodecs.imwrite("java" + frame + ".jpg", out, writeBoardParams); // debugging - save image in jpg file
        
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
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     logDetectedCorners                                          */
/*                                     logDetectedCorners                                          */
/*                                     logDetectedCorners                                          */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    /**
     * Detected Board data in mrgingham format plus the board info
     * @param img Camera image
     * @param ugui User Guidance Class
     * @throws FileNotFoundException
     */
    public static void logDetectedCorners(Mat img, UserGuidance ugui) throws FileNotFoundException
    {
        if (vnlog == null) // first time switch
        {
            vnlog = new PrintWriter("corners.vnl");
            vnlog.println("## produced by pose guidance calibration program");
            vnlog.println("# filename x y level cid boardX boardY");
        }

        // write the captured frame to a file name
        int captureCount = ugui.calib.keyframes.size();

        String filename = String.format("img%02d.jpg", captureCount);
        final MatOfInt writeBoardParams = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, 100); // pair-wise; param1, value1, ...
        Imgcodecs.imwrite(filename, img, writeBoardParams); // save camera image

        // for efficiency put Mat data in arrays
        Point3[] ChessboardCorners = ugui.tracker.board().getChessboardCorners().toArray(); // 1 col 3 channels x, y, z in a Point3 (float)

        int[] DetectedCIDs = new int[ugui.tracker.cids().rows()]; // get detected corners - assume captured image does have corners
        ugui.tracker.cids().get(0, 0, DetectedCIDs);

        float[] DetectedCorners = new float[ugui.tracker.ccorners().rows()*ugui.tracker.ccorners().cols()*ugui.tracker.ccorners().channels()]; // below assumes x and y in a row
        ugui.tracker.ccorners().get(0, 0, DetectedCorners);

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
            logLine.append(ChessboardCorners[boardCornerId].x); // x
            logLine.append(" ");
            logLine.append(ChessboardCorners[boardCornerId].y); // y
            vnlog.println(logLine.toString());                    
        }
    }
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     showPose                                                    */
/*                                     showPose                                                    */
/*                                     showPose                                                    */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    /**
     * pose cartoon stuff under development
     */
    static void showPose(ChArucoDetector tracker, UserGuidance ugui)
    {
        if ( ugui.tgt_t().empty())
        {
            return;
        }
//////////////// cube (camera cartoon)
        Point3[] cube = new Point3[5*5*5];
        int index = 0;
        for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
        for (int k = 0; k < 5; k++)
            cube[index++] = new Point3(i, j, k);
//////////////// ! cube

//////////////// board
        Mat tinyBoard = new Mat();
        double scale = 0.06;
        Imgproc.resize(tracker.boardImage, tinyBoard, new Size(tracker.boardImage.cols()*scale, tracker.boardImage.rows()*scale), Imgproc.INTER_AREA);
       
        Point3[] boardMeshGrid = new Point3[tinyBoard.rows()*tinyBoard.cols() + cube.length]; // space for the board and the camera icon

        int boardMeshGridIndex = 0;
        for (int row = 0; row < tinyBoard.rows(); row++)
        for (int col = 0; col < tinyBoard.cols(); col++)
        {
            boardMeshGrid[boardMeshGridIndex++] = new Point3(col, 0, row);
        }
        // add the cube (camera) points to the grid after the board points in the right place in space relative to the board
        double[] tvecPose = new double[3];
        // tvecPose[0] = -596.; // test data - 1st pose
        // tvecPose[1] = -1258.; // flipped with -
        // tvecPose[2] = 4628;

        ugui.tgt_t().get(0, 0, tvecPose); // real data here
        for (Point3 cubePoint : cube)
        {
            double[] cubePositioned = {cubePoint.x+tvecPose[0]*scale+tinyBoard.cols()/2, cubePoint.y+tvecPose[1]*scale+tinyBoard.rows()/2, cubePoint.z+tvecPose[2]*scale};
            boardMeshGrid[boardMeshGridIndex++] = new Point3(cubePositioned);
        }

        MatOfPoint3f tinyBoardMeshGridLinear = new MatOfPoint3f(boardMeshGrid);
        // System.out.println("tinyBoardMeshGridLinear " + tinyBoardMeshGridLinear); // 15000x1 3 channels
//////////////// ! board

        // somehow add the camera icon in the right place with the board to the end of tinyBoardMeshGridLinear
        // or merge with another MatOfPoint3f. Put everything in an ArrayList then can add points at will.

        MatOfPoint3f  cubeWarped = new MatOfPoint3f();

        Mat rvecBoard = new Mat(3, 1, CvType.CV_32FC1);
        rvecBoard.put(0, 0, 0.1, 10., 0.3);

        Mat tvecBoard = new Mat(3, 1, CvType.CV_32FC1);
        tvecBoard.put(0, 0, 300, 100, 500);
         
        Mat imagePose = Mat.zeros(1000, 1200, CvType.CV_8UC1);

        // with aspect ratio of 1 and pp at center. Focal length is empirical.
        Mat Kin = Mat.zeros(3, 3, CvType.CV_64FC1);
        Kin.put(0, 0, 800.); // fx
        Kin.put(1, 1, 800.); // fy
        Kin.put(2, 2, 1.);

        Mat K = Calib3d.getDefaultNewCameraMatrix(Kin, new Size(640, 480), true);
        // System.out.println("K\n" + K.dump());

        MatOfDouble cdist = new MatOfDouble(0.1, -0.05, 0, 0, 0);
        // System.out.println("cdist\n" +cdist.dump());

        MatOfPoint2f tinyBoardProjected = new MatOfPoint2f();
        Calib3d.projectPoints(
            tinyBoardMeshGridLinear,
            rvecBoard, tvecBoard,
            K, cdist,
            tinyBoardProjected);
        // System.out.println("cube projected\n" + cubeProjected.dump());
        // System.out.println(ArrayUtils.brief(tinyBoardProjected));
        // put the projected board on the image.  warpPrespective might be better than this warped mesh grid
        int tinyBoardProjectedRowsIndex = 0;
        for (int row = 0; row < tinyBoard.rows(); row++)
        for (int col = 0; col < tinyBoard.cols(); col++)
        {
            // get the value of tinyBoard at the unwarped location x, y and put it in the warped location x',y'
            float[] warpedCoord = new float[2]; // 2 channels x and y
            tinyBoardProjected.get(tinyBoardProjectedRowsIndex++, 0, warpedCoord);
            byte[] data = new byte[1];
            tinyBoard.get(row, col, data);
            imagePose.put((int)warpedCoord[1], (int)warpedCoord[0], data);
        }

        for (int cubeIndex = 0 ; cubeIndex < cube.length; cubeIndex++)
        {
            // make the cube 255 at all its warped locations x',y'
            float[] warpedCoord = new float[2]; // 2 channels x and y
            tinyBoardProjected.get(tinyBoardProjectedRowsIndex++, 0, warpedCoord);
            byte[] data = {-1};
            imagePose.put((int)warpedCoord[1], (int)warpedCoord[0], data); //FIXME  EXCEPTION HERE on 3rd pose I think
        }

        HighGuiX.imshow("Guidance Pose", imagePose);
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
// Append "-x spotlessapply" to the commands you run to disable it

// javac -Xlint:unchecked  xxxxx.java