package calibrator;

import org.opencv.core.Mat;

class keyframe
{
  Mat p2d; // ccorners
  Mat p3d;
  keyframe(Mat p2d, Mat p3d)
  {
    this.p2d = p2d;
    this.p3d = p3d;
  }
}
