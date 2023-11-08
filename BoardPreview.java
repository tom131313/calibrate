// projects a 2D object (image) according to parameters - generate styled board image
package calibrator;

import java.util.logging.Level;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     BoardPreview class                                                     */
/*                                     BoardPreview class                                                     */
/*                                     BoardPreview class                                                     */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
class BoardPreview {
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     project_img                                                            */
/*                                     project_img                                                            */
/*                                     project_img                                                            */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/**
 *
 * @param img image to project
 * @param sz size of the final image
 * @param K
 * @param rvec
 * @param t
 * @param flags
 * @return
 */
    private static Mat project_img(Mat img, Size sz, Mat K, Mat rvec, Mat t, int flags)
    // force user to specify flags=cv2.INTER_LINEAR to use default
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        //Main.LOGGER.log(Level.WARNING, "img " + img);
        //Main.LOGGER.log(Level.WARNING, "sz " + sz);
        //Main.LOGGER.log(Level.WARNING, "K " + K + "\n" + K.dump());
        //Main.LOGGER.log(Level.WARNING, "rvec " + rvec + rvec.dump());
        //Main.LOGGER.log(Level.WARNING, "t " + t + t.dump());
        //Main.LOGGER.log(Level.WARNING, "flags " + flags);

        // construct homography
        Mat R = new Mat();
        Calib3d.Rodrigues(rvec, R);

        //Main.LOGGER.log(Level.WARNING, "R " + R + "\n" + R.dump());
        Mat transform = new Mat(3, 3, CvType.CV_64FC1); // rotation matrix R and a translation matrix T (t)
        transform.put(0, 0,
            R.get(0, 0)[0], R.get(0, 1)[0], R.get(0, 2)[0], // 1st row r,second row r, third row t
                R.get(1, 0)[0], R.get(1, 1)[0], R.get(1, 2)[0],
                t.get(0, 0)[0], t.get(0, 1)[0], t.get(0, 2)[0]);
        Core.transpose(transform, transform);
        Mat H = new Mat();
        Core.gemm(K, transform, 1., new Mat(), 0., H);
        Core.divide(H, new Scalar(H.get(2, 2)[0]), H);
   
        //Main.LOGGER.log(Level.WARNING, "transform " + transform + "\n" + transform.dump());
        //Main.LOGGER.log(Level.WARNING, "R " + R + "\n" + R.dump());
        //Main.LOGGER.log(Level.WARNING, "H " + H + "\n" + H.dump());

        Mat imgProjected = new Mat();

        Imgproc.warpPerspective(img, imgProjected, H, sz, flags);

        // draw axes on the warped Guidance Board
        // these are drawn in the wrong corner and drawing before the warpPerspective doesn't work - tiny image in worse spot
        // these axes diagram cover the desired posed (warped) Guidance Board before it is processed. Maybe draw them later and they are in a different position
        Core.flip(imgProjected, imgProjected, 0); // flip to get axes origin in correct corner BUT the estimated origin is reversed from where it belongs
        Calib3d.drawFrameAxes(imgProjected, K, new Mat(), R, t, 300.f); // may need rotation vector rvec instead of R
        Core.flip(imgProjected, imgProjected, 0); // put the img back right

        transform.release();
        R.release();
        H.release();

        // //Main.LOGGER.log(Level.WARNING, "returning imgProjected\n" + brief(imgProjected));

        return imgProjected;
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     BoardPreview constructor                                               */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}

    private Size SIZE = new Size(640., 480.); // different than camera res okay and it runs a little faster if smaller. Resize at end makes images match
    private Size sz;
    private Mat img = new Mat();
    private Mat shadow; // used for overlap score
    private Mat maps; // 2D version used for remap function
    private Mat Knew;
    BoardPreview(Mat img)
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        img.copyTo(this.img);

        // at this point the img appears correctly if displayed on a screen or printed
        Core.flip(this.img, this.img, 0); // flipped when printing
        // at this point the img is flipped upside down. This is needed because the Imgproc.warpPerspective
        // flips the img upside down likely because it's acting like it's projecting through a camera
        // aperture/lens which flips the scene upside down. So after the Imgproc.warpPerspective the img is
        // again upside right.

        // light to much to see low exposure camera images behind it so reduce almost to nothing. Can't set to 0 - messes up other places that check 0 or not 0
        // set black pixels to gray; non-black pixels stay the same

        // process entire Mat for efficiency
        byte[] img1ChannelBuff = new byte[this.img.rows()*this.img.cols()*this.img.channels()]; // temp buffer for more efficient access
        this.img.get(0, 0, img1ChannelBuff); // get the row, all channels
        for (int index = 0; index < img1ChannelBuff.length; index++) // process each element of the row
        {
            // if the camera image is dimmer then the guidance board needs to be dimmer.
            // if the camera image is bright then the guidance board needs to be bright.
            if (img1ChannelBuff[index] == 0) // is it black?
            {
                img1ChannelBuff[index] = 1; // make it gray - I'd like to make this 0 so it's easy to see the camera image
                // behind it but that messes up the shadow board logic that relies on non-zero pixels. Need major surgery to fix
            }
            else
            {
                img1ChannelBuff[index] = 127; // make the bright green subdued.
            }
        }
        this.img.put(0, 0, img1ChannelBuff);

        Imgproc.cvtColor(this.img, this.img, Imgproc.COLOR_GRAY2BGR);

        // set blue and red channels to black (0) so gray/white becomes a shade of green (green channel was not changed)
        byte[] img3ChannelBuff = new byte[this.img.rows()*this.img.cols()*this.img.channels()]; // temp buffers for efficient access
        // process one row at a time for efficiency
        this.img.get(0, 0, img3ChannelBuff); // get the row, all channels
        for (int index = 0; index < img3ChannelBuff.length; index += 3) // process each triplet (channels) of the row
        {
            img3ChannelBuff[index] = 0; // B
            img3ChannelBuff[index + 2] = 0; // R
        }
        this.img.put(0, 0, img3ChannelBuff);

        // used for overlap score
        this.shadow = Mat.ones(this.img.rows(), this.img.cols(), CvType.CV_8UC1);
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     create_maps                                                            */
/*                                     create_maps                                                            */
/*                                     create_maps                                                            */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    void create_maps(Mat K, Mat cdist, Size sz)
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        //Main.LOGGER.log(Level.WARNING, "camera matrix K " + K + "\n" + K.dump());
        //Main.LOGGER.log(Level.WARNING, "cdist " + cdist.dump());
        //Main.LOGGER.log(Level.WARNING, "sz " + sz);

        // cdist initialized in its constructor instead of setting to 0 here if null; did 5 not 4 for consistency with rest of code
        this.sz = sz;
        Mat scale = Mat.zeros(3, 3, K.type()); // assuming it's K.rows(), K.cols()
        scale.put(0, 0, this.SIZE.width/sz.width);
        scale.put(1, 1, this.SIZE.height/sz.height);
        scale.put(2, 2, 1.);
        //Main.Kcsv(Id.__LINE__(), K);
        Core.gemm(scale, K, 1., new Mat(), 0.,K);
        //Main.Kcsv(Id.__LINE__(), K);
        sz = this.SIZE;
        //Main.LOGGER.log(Level.WARNING, "K scaled\n" + K.dump());
        //Main.LOGGER.log(Level.WARNING, "Knew null " + (this.Knew == null)); // matches to here

        this.Knew = Calib3d.getOptimalNewCameraMatrix(K, cdist, sz, 1.); // .2% higher than Python for same input
        //Main.LOGGER.log(Level.WARNING, "Knew " + this.Knew + "\n" + this.Knew.dump());
        //Main.Kcsv(Id.__LINE__(), this.Knew);
        this.maps = Distortion.make_distort_map(K, sz, cdist, this.Knew);
        //Main.Kcsv(Id.__LINE__(), this.Knew);
        //Main.LOGGER.log(Level.WARNING, "Knew " + this.Knew + "\n" + this.Knew.dump()); // same as input to make_distort_map
        //Main.LOGGER.log(Level.WARNING, "maps " + this.maps + "\n" + brief(this.maps));
    }
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     project                                                                */
/*                                     project                                                                */
/*                                     project                                                                */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
    Mat project(Mat r, Mat t, boolean useShadow, int inter)
    // force users to specify useShadow=false and inter=Imgproc.INTER_NEAREST instead of defaulting
    // no default allowed in Java and I don't feel like making a bunch of overloaded methods for this conversion
    {
        //Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        //Main.LOGGER.log(Level.WARNING, "r " + r.dump());
        //Main.LOGGER.log(Level.WARNING, "t " + t.dump());
        //Main.LOGGER.log(Level.WARNING, "useShadow " + useShadow);
        //Main.LOGGER.log(Level.WARNING, "inter " + inter);
        //Main.LOGGER.log(Level.WARNING, "sz " + this.sz);

        Mat img = new Mat();

        img = project_img(useShadow ? this.shadow : this.img, this.SIZE, this.Knew, r, t, Imgproc.INTER_LINEAR);

        //Main.LOGGER.log(Level.WARNING, "maps " + this.maps + "\n" + brief(maps));
        // Can be one map for XY or two maps X and Y. python had 2 and this has 1
        // Imgproc.remap(img, img, maps[0]/*X*/, maps[1]/*Y*/, inter);// maybe X Mat and Y Mat somehow; separate channels?

        Imgproc.remap(img, img, this.maps, new Mat(), inter);// 1st arg can be XY with no 2nd arg (original has separate X and Y arguments)
        // //Main.LOGGER.log(Level.WARNING, "img after remap " + img + "\n" + brief(img));

        // maps (2, 480, 640)
        Imgproc.resize(img, img, this.sz, 0, 0, inter);

        // //Main.LOGGER.log(Level.WARNING, "returning img after resize " + img + "\n" + brief(img));

        return img;
    }
}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     End BoardPreview class                                                 */
/*                                     End BoardPreview class                                                 */
/*                                     End BoardPreview class                                                 */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
