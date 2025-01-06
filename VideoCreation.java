package Guidance;

import java.util.logging.Logger;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.videoio.VideoWriter;

public class VideoCreation {
    
	private static Logger LOGGER;

	static {
	  LOGGER = Logger.getLogger("");
	  LOGGER.finest("Loading");     
	}

        String filename;

        int fourcc = VideoWriter.fourcc('m', 'p', '4', 'v');
        // var fourcc = VideoWriter.fourcc('X', 'V', 'I', 'D'); AVI ?
        // var fourcc = VideoWriter.fourcc('D', 'I', 'V', 'X'); AVI ?
        
        VideoWriter videoWriter;

    public VideoCreation(String filename, Size size)
    {
        this.filename = filename;
        videoWriter = new VideoWriter(filename, fourcc, 2, size);

        // Check if video writer is opened successfully
        if (!videoWriter.isOpened()) {
            LOGGER.severe("Error opening video writer");
        }
    }

    public void addFrame(Mat frame)
    {
        videoWriter.write(frame);
    }

    public void close()
    {
        LOGGER.info("Calibration image video completed " + filename);
        videoWriter.release();        
    }
}
