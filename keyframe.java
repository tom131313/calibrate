package calibrator;

import java.util.logging.Level;

import org.opencv.core.Mat;

/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     keyframe class                                                         */
/*                                     keyframe class                                                         */
/*                                     keyframe class                                                         */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
class keyframe
{
  private Mat p2d; // detected ccorners in camera image
  private Mat p3d; // target ChArUcoBoard object - perfect without distortion in 3d space but ours is always flat on a wall so Z = 0

  // getters
  Mat p2d(){return p2d;}
  Mat p3d(){return p3d;}

/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     keyframe constructor                                                   */
/*                                     keyframe constructor                                                   */
/*                                     keyframe constructor                                                   */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
keyframe(Mat p2d, Mat p3d)
  {
    this.p2d = p2d;
    this.p3d = p3d;
    if(this.p2d.rows() != this.p3d.rows() || this.p2d.cols() != p3d.cols())
    {
        Main.LOGGER.log(Level.SEVERE, "size of p2d != p3d");
    }
  }
}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     End keyframe class                                                     */
/*                                     End keyframe class                                                     */
/*                                     End keyframe class                                                     */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
