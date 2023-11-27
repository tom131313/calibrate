# Java Camera Calibrator
Program guides user to pose the camera to the ChArUcoBoard for most efficient and accurate calibration.

**Run from a terminal window either the Windows jar or Linux arm64 jar (RPi, etc.)**

**don't double click the jar file name**

`java -jar the_right_jar_file [options]`

Logs accumulate in the directory with the jar file.

Options are listed with -help, for example, on a Windows PC:

`java -jar calib-photonvision-dev-beta-X-winx64.jar -help`

```
C:\Users\RKT\frc\FRC2023\Code\photonvision-2023-10-30_pose_calib_integration>java -jar calib-photonvision-dev-beta-11-4-winx64.jar -help
Starting class: org.photonvision.calibrator.Main version beta 11.4
Starting class: org.photonvision.calibrator.Main version beta 11.4
Starting class: org.photonvision.calibrator.Loggers
Starting class: org.photonvision.calibrator.Loggers

Starting Log for Camera Calibration Program Version beta 11.4, current time 2023-11-26T12:28:10.954667300

00000 CONFIG  [ Loggers setupLoggers] Logger configuration done.
00000 SEVERE  [ Main main] logs accumulate in file CalibrationLog.txt
usage: java -jar <your jar file>.jar [options]
 -cameraId <arg>   camera id (0)
 -dpmX <arg>       print width pixels per meter (9843=250 DPI)
 -dpmY <arg>       print height pixels per meter (9843=250 DPI
 -fps <arg>        frames per second (10)
 -h,--help         Show this help text and exit
 -height <arg>     camera image height (720)
 -isPV <arg>       using PV environment (true)
 -pxFmt <arg>      pixel format (kYUYV) [kUnknown, kMJPEG, kYUYV, kRGB565,
                   kBGR, kGray, kY16, kUYVY]
 -width <arg>      camera image width (1280)
THE END OF THE LOG
```

On a computer with an internal camera the camera ids on Windows typically are:
internal and external cameras connected at boot up: external=0, internal=1
internal camera connected at boot up and external plugged in later: internal=0, external=1
It may be convenient to disable the internal camera for extended calibration testing to make the camera id for the external constant.

Start the program (successfully) to make a ChArUcoBoard.png file that can be printed.

Run the program and aim the camera at the printed board in the pose that matches the guidance board on the computer screen. It may be easier to align a fixed camera and hold and move the board.

The first two guidance poses are always the same.

The first pose at a fairly steep angle to the board's left (camera's right) sets the initial camera matrix. I advise getting the angles correct but not at precisely the correct distance so the images do not align yet. Then carefully move closer or further to get the precise size alignment. The capture should be automatic. “Well begun is half done.” - Aristotle. [Rotated poses are good to calibrate the camera matrix.]

Occaisionally the rotated first pose "jumps" to a different size. That is NOT a capture and is the program trying to use the latest estimate of the camera matrix for that first pose. That pose is captured when the second pose - the head-on pose - appears.

The second pose is straight head-on and sets the initial distortion. Similarly, get the straight-on correct, move to align the images but not yet the matching sizes then move closer or further to match. The capture should be automatic.[Straight-on poses are good to calibrate the distortion coefficients.]

If the guidance board and the camera image match, the program should auto-capture that information. Rarely, but if the auto capture is not happening, the user can force capture by pressing "c" and Enter on the computer terminal window that was used to start the program. The black and white insert shows what the poses are of the exact guidance board and the estimated camera view. The similar numbers are the Jaccard index between the poses and the minimum acceptable value for auto capture.

The nine red camera intrinsic parameters turn green when the poses provide enough information. Usually after about 10 carefully aligned poses.

The terminal (keyboard) inputs are 'c' for force capture, 'm' for mirror view if that helps you align the camera to the guidance, and 'q' to quit. Rarely are those needed.

The display of the guidance board and camera view are on a browser's port 1185. For example, 127.0.0.1:1185?action=stream.

The camera server is standard WPILib camera server stuff so you can adust many camera parameters on the automatically assigned port - 1181 or higher. For example 127.0.0.1:1181 to see the camera parameters.

If you run this on a system with PhotonVision running then stop PhotonVision. (linux command is `sudo service photonvision stop`)

Camera calibration data is written to a json file suitable for import into PhotonVision. (Note the camera name and platform are unknown so if you needed those, edit the file.)
References:

https://arxiv.org/pdf/1907.04096.pdf

https://www.calibdb.net/#

https://github.com/paroj/pose_calib
