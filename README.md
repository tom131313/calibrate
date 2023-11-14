# Java Camera Calibrator

run from a terminal window the photonvision-dev-calib-winx64.jar (Windows PC) or photonvision-dev-calib-linuxarm64.jar (RPi, etc.)

java -jar <the right jar file> [options]

Options are listed with -help, for example, on a Windows PC:
java -jar photonvision-dev-calib-winx64.jar -help

Options included for changing the numeric camera Id, camera pixel format, wide, height, ChArUcoBoard printing size.

Run the program to make a ChArUcoBoard.png file that can be printed.

Run the program and aim the camera at the printed board in the pose that matches the guidance board on the computer screen. It may be easier to align a fixed camera and hold and move the baord.

If the guidance board and the camera image match, the program should auto-capture that information. The (lack of always) auto capture leaves something to be desired and the user can force capture by pressing "c" and Enter on the computer terminal window that was used to start the program. I suggest pressing 'c' after the program starts and it's ready for a press Enter when the poses align but failed to auto capture. The black and white insert shows what the poses are of the guidance board (exactly right) and the estimated camera view (not always right - want to help me get this better? It has to do with how the program is dynamically adjusting the estimated camera matrix).

The nine red camera intrinsic parameters turn green when the poses provide enough information. Usually after about 15 carefully aligned poses.

Other terminal (keyboard) input are 'm' for mirror view if that helps you align the camera to the guidance and 'q' to quit.

The display of the guidance board and camera view are on a browser's port 1185. For example, 127.0.0.1:1185?action=stream or just 127.0.0.1:1185 to see the camera parameters, too. (This is standard WPILib camera server stuff so you can adust your camera parameters there.)

If you run this on a system with PhotonVision running then stop PhotonVision. (linux commnad is `sudo service photonvision stop`)

References:

https://arxiv.org/pdf/1907.04096.pdf

https://www.calibdb.net/#

https://github.com/paroj/pose_calib
