package Guidance;

import java.util.logging.Logger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoProperty;
import edu.wpi.first.util.PixelFormat;

public final class CameraUSB {
    private static Logger LOGGER;
    static {
      LOGGER = Logger.getLogger("");
      LOGGER.finer("Loading");     
    }
    
    private CameraUSB(){}
    
    /**
     * Use a USB cmera for image source
     * 
     * @param camId digit index of USB camera
     * @param image_width
     * @param image_height
     * @param fps
     * @param pixelFormat
     * @return source of images
     */
    public static CvSink getSource(int camId, int image_width, int image_height, int fps, PixelFormat pixelFormat) {

        // camera input
        final UsbCamera camera = CameraServer.startAutomaticCapture(camId); // gives access to camera parameters on port 1181 (or above)
 
        // final UsbCamera camera = new UsbCamera("mycamera", camId);
        // same camera as above but no interaction on port 1181 (or above); would need setSource or addCamera, too

        for ( VideoMode vm : camera.enumerateVideoModes())
        {
            LOGGER.info("Camera mode choices " + vm.pixelFormat + " "
                + vm.width + "x" + vm.height + " " + vm.fps + " fps");
        }

        for ( VideoProperty vp : camera.enumerateProperties())
        {
            LOGGER.info("camera property choices " + vp.get() + " " + vp.getName() + " " + VideoProperty.getKindFromInt(vp.get()));
        }

        VideoMode videoMode = new VideoMode(pixelFormat, image_width, image_height, fps);
        LOGGER.config("Setting camera mode "
                + pixelFormat + " " + image_width + "x" + image_height + " " + fps + "fps");
        try {
            if ( ! camera.setVideoMode(videoMode)) throw new IllegalArgumentException("set video mode returned false");
        } catch (Exception e) {
            LOGGER.severe("camera set video mode error; mode is unchanged " + e);
        }

        LOGGER.info("camera " + camId + " properties can be seen and changed on port 1181 or higher");

        CvSink capture = CameraServer.getVideo(camera); // Get a CvSink. This will capture Mats from the camera
        capture.setSource(camera);

        return capture;
    }
}
