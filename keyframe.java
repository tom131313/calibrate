package calibrator;

import org.opencv.core.Mat;

class keyframe
{
  private Mat p2d; // detected ccorners in camera image
  private Mat p3d; // target ChArUcoBoard - perfect without distortion in 3d space but ours is always flat on a wall so Z = 0

  // getters
  Mat p2d(){return p2d;}
  Mat p3d(){return p3d;}

  keyframe(Mat p2d, Mat p3d)
  {
    this.p2d = p2d;
    this.p3d = p3d;
  }
}
