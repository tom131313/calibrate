package Guidance;

import java.util.logging  .Logger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;

public final class CameraHTTP {
    private static Logger LOGGER;
    static {
      LOGGER = Logger.getLogger("");
      LOGGER.finer("Loading");     
    }
    
    CameraHTTP(){}

    /**
     * Use a remote camera server URL for image source
     * 
     * @param name to be used (arbitrary)
     * @param url to be used - must start with http://
     * @return source of images
     */
    public static CvSink getSource(String name, String url) {

        LOGGER.config("Using remote camera " + name + " at URL " + url);
        // remote camera input
        HttpCamera remoteFeed = new HttpCamera(name, url, HttpCameraKind.kMJPGStreamer);
        CameraServer.addCamera(remoteFeed);

        // may also use CameraServer.startAutomaticCapture(...) for interaction on 1181 (or above)

        var capture = CameraServer.getVideo(name);

        return capture;
    }
}
