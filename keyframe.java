// This project and file are derived in part from the "Pose Calib" project by
// @author Pavel Rojtberg
// It is subject to his license terms in the PoseCalibLICENSE file.

package org.photonvision.calibrator;

import org.opencv.core.Mat;

import org.photonvision.common.logging.LogGroup;
import org.photonvision.common.logging.Logger;

/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     keyframe class                                              */
/*                                     keyframe class                                              */
/*                                     keyframe class                                              */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
class keyframe
{
  private static final Logger logger = new Logger(keyframe.class, LogGroup.General);

  private Mat p2d; // detected ccorners in camera image
  private Mat p3d; // target ChArUcoBoard object - perfect without distortion in 3d space but ours is always flat on a wall so Z = 0

  // getters
  Mat p2d()
  {
    return p2d;
  }
  Mat p3d()
  {
    return p3d;
  }

/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     keyframe constructor                                        */
/*                                     keyframe constructor                                        */
/*                                     keyframe constructor                                        */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
keyframe(Mat p2d, Mat p3d)
  {
    this.p2d = p2d;
    this.p3d = p3d;
    if (this.p2d.rows() != this.p3d.rows() || this.p2d.cols() != p3d.cols())
    {
        logger.error("size of p2d != p3d\n" + this.p2d.dump() + "\n" + this.p3d.dump());
    }
    logger.debug("calibration image captured");
  }
}
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     End keyframe class                                          */
/*                                     End keyframe class                                          */
/*                                     End keyframe class                                          */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
