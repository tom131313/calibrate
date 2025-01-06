package Guidance;

import java.util.Scanner;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Logger;

// Java Scanner alternative to OpenCV keyboard usage that is not in PV headless (AWT is missing)

// Turn Scanner keys into OpenCV keys to ease transition back and forth between PV terminal and OpenCV waitKey
class Keystroke implements Runnable
{
        private static final Logger LOGGER = Logger.getLogger(""); 
        static {
          LOGGER.finest("Loading");     
        }

    // keyboard mapping returns from OpenCV waitKey
    // can't use enum in switches (maybe with the latest java you can)
    static final int keyTerminate = 81; // q
    static final int keyCapture = 67; // c
    static final int keyMirrorToggle = 77; // m
    static final int keyNone = -1;  // timed out, no key pressed

    // keyboard mapping returns from Java Scanner
    private static final int keyTerminateScanner = 113; // q
    private static final int keyCaptureScanner = 99; // c
    private static final int keyMirrorToggleScanner = 109; // m

    AtomicInteger dokeystroke = new AtomicInteger(-1);

    /**
     * Read the terminal for user entered commands.
     * 
     * Scanner blocks waiting for input so this must be run in a different thread
     * if blocking is not tolerable. Thus it is designed to run as a thread and 
     * the returned character command can be accessed in a thread safe manner.
     * 
     * Type a character command and press Enter.
     * 
     * The first letter entered is interpreted as a command.
     * Excess characters before the "Enter" are ignored.
     */
    public void run()
    {
        try (Scanner keyboard = new Scanner(System.in))
        {
            while( ! Thread.interrupted())
            {
                System.out.println("Pose should auto capture otherwise, press c (capture), m (mirror); q (quit calibrating or focusing) then the Enter key");
                String entered = keyboard.next();
                int keyScanner = entered.charAt(0);
                LOGGER.finest("user entered " + entered + ", action is " + keyScanner + " " + String.valueOf(Character.toChars(keyScanner)));
                // map Scanner character codes to OpenCV character codes
                if (keyScanner == keyCaptureScanner)
                {
                    dokeystroke.set(keyCapture);
                }
                else if (keyScanner == keyMirrorToggleScanner)
                {
                    dokeystroke.set(keyMirrorToggle);
                }
                else if (keyScanner == keyTerminateScanner)
                {
                    dokeystroke.set(keyTerminate);
                }
                else // ignore any keys that weren't mapped above
                {
                    LOGGER.info(String.valueOf(String.valueOf(Character.toChars(keyScanner)) + " not a command"));
                }
            }
        } catch(Exception e) {LOGGER.severe(
            "Terminal keyboard closed prematurely (Ctrl-c) or doesn't exist (jar file not run from command line; don't double click the jar to start it)");}
    }

    /**
     * Get the command entered on the terminal
     * 
     * This getter is expected to be called by a different thread than the "run" method
     * above so access to the character command is in a thread safe manner.
     * @return the character command
     */
    public int getKey() {
        return dokeystroke.getAndSet(keyNone); // after getting the previous key, re-initialize to indicate no key pressed so far
    }
}
