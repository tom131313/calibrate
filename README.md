# Java Camera Calibrator
Program guides user to pose the camera to the ChArUcoBoard for most efficient and accurate calibration.

It accepts a USB camera as input for standalone calibration or can use the stream output of another program such as LimeLight to provide 
more efficient pose guidance for those programs. There is no automated connection between this guidance automatically capturing a calibration 
snapshot and the other program. When the Guidance captures a snapshot, the user should hold the camera very steady in that position and then trigger the 
corresponding snapshot in the other program. An on screen message helps to remind the user. The Guidance has moved on to the next guidance pose or ended. 
The user has to hold the camera still and trigger the other program's snapshot despite those distractions. Two people working together may make this a 
lot easier.

**Run the jar with the correct version id file name from a Windows command terminal**

**don't double click the jar file name**

`java -jar CalibrationGuidance.jar [options]`

Examples:

C:\Users\bike1\FRC\2025\CalibrationGuidance>java -jar CalibrationGuidance.jar "-S" "-cameraId=LL, http://10.42.37.22:5802"

C:\Users\bike1\FRC\2025\CalibrationGuidance>java -jar CalibrationGuidance.jar "-S" "-cameraId=1" "-fps=100" "-height=800" "-width=1280" "-pxFmt=kMJPEG"

C:\Users\bike1\FRC\2025\CalibrationGuidance>java -jar CalibrationGuidance.jar "-f"

C:\Users\bike1\FRC\2025\CalibrationGuidance>java -jar CalibrationGuidance.jar "-h"

C:\Users\bike1\FRC\2025\CalibrationGuidance>java -jar CalibrationGuidance.jar "-f"

C:\Users\bike1\FRC\2025\CalibrationGuidance>java -jar CalibrationGuidance.jar "-B"

Run log is in the directory with the jar file.

User specified Runtime Options are listed with `-help`, for example:

`java -jar CalibrationGuidance.jar -help`

This is a minimal set of options from the many options set in the program at compile time.

On a computer with an internal camera the camera ids on Windows typically are:
internal and external cameras connected at boot up: external=0, internal=1
internal camera connected at boot up and external plugged in later: internal=0, external=1
It may be convenient to disable the internal camera for extended calibration testing to make the camera id for the external constant.

This program is setup to use the PhotonVision calibration board 1" squares, 0.75" markers, 4x4 dictionary. LimeLight Vision can also use that calibration. The 5x5 dictionary and other numbers of squares on the board are supported.

If a board isn't available, then start the program with the `-printBoard` option to make a `ChArUcoBoard.png` file that can be printed. Instead of printing, 
the ChArUcoBoard file can be displayed in another window on the computer screen or on a second screen. Use of a computer flat screen display can improve the 
camera calibration, however, there may be restrictions compared to using a paper board or commercail stiff board. Computer displays must be flat and visible 
at steep (nearly parallel) angles. For the smallest guidance target some displays may not have enough resolution or refresh rate to display the guidance 
(although it may look fine to the human eye). The size of the squares must be determined accurately without scratching the display.

Run the program and aim the camera at the printed board in the pose that matches the guidance board on the computer screen. It may be easier to align a fixed 
camera and hold and move the board.

The top line of the guidance display has movement information that isn't very useful. Below that are the guidance rotations and translations 
`r{x, y, z} t{x, y, z}`. The camera should be aimed such that its rotations and translations fairly closely match the guidance. Sometimes a large negative 
rotation (say -345) is actually matched by a small positive rotation (say 15). The small insert of the outline of the guidance and the current camera pose 
indicate how close the camera is to matching the guidance.

The first two guidance poses are always the same.

The first pose at a fairly steep angle to the board's left (camera's right) sets the initial camera matrix. Start with a steep angle to the board (NOT head-on) 
and get the angles correct but not at precisely the correct distance so the images do not align yet. Then carefully move closer or further to get the precise 
size alignment. The capture should be automatic. “Well begun is half done.” - Aristotle. [Rotated poses are good to calibrate the camera matrix.] The guidance 
board is always intended to be able to be duplicated by a camera pose. An occaisonal distorted board that is bent, broken or split indicates previously captured 
poses weren't close enough to the guidance. Start over.

Occaisionally the rotated first pose "jumps" to a different size. That is NOT a capture and is the program trying to use the latest best estimate of the camera 
matrix for that first pose. (The calibration matrix is recalculated with each image frame trying to improve on the previous frame in order to get a good 
boot-strap, initial camera matrix. A jump in the image indicates the program thought that was a better rendition. It might have happened with too much of a head-on 
view that yields poor intial values.) That pose is captured when the second pose - the head-on pose - appears.

The second pose is straight head-on and sets the initial distortion. Similarly, get the straight-on correct, move to align the images but not yet the matching 
sizes then move closer or further to match. The capture should be automatic. [Straight-on poses are good to calibrate the distortion coefficients.]

If the guidance board and the camera image match, the program should auto-capture that information. Rarely, if the auto capture is not happening, the user without 
a steady hand and patience can force capture by pressing `c` and `Enter` on the computer terminal window that was used to start the program. The black and white 
insert shows what the poses are of the exact guidance board and the estimated camera view. The similar numbers are the Jaccard index between the poses and the 
minimum acceptable value for auto capture.

The nine red camera intrinsic parameters turn green when the poses provide enough information. Usually after about 10 poses. That information might not be good 
unless you have carefully aligned to the guidance and especially autocapture yields good information.

Be especially careful to align to the first two guidance poses. They are always the same and easy to align to once the user gets the hang of it.

(You can point incorrectly at the target and randomly hit the `c` and `Enter` several times to complete a bad calibration; it doesn't care, although it has been known to throw an "unknown exception" and keep going as if nothing bad happened.)

The terminal (keyboard) inputs are `c` for force capture, `m` for mirror view if that helps you align the camera to the guidance, and `q` to quit. Rarely if ever, 
are those needed. `q` wil stop the current calibration or focus session (less graceful stop is hit `Ctrl-c` in the terminal window).

The display of the guidance board and camera view are on a browser's port 1185 (default can be changed by the user) on the computer running the program. For example, `127.0.0.1:1185?action=stream` or `localhost:1185?action=stream`.

The camera server is standard WPILib camera server stuff so you can adjust many camera parameters on the automatically assigned port - 1181 or higher. For example 
`127.0.0.1:1181` to see the camera parameters.

If you run this on a system with PhotonVision running and aren't using the PV output as input to this Guidance Porgram, then stop PhotonVision.

Camera calibration data is logged to the console (terminal) and file.

The program has an option to display a Siemens Star for aid camera focusing.

[Usage observations:
LimeLight alternate output stream (5802) runs at full resolution. For a responsive camera image use the "Viewfinder" pipeline to take snapshots.

PhotonVision has no pure raw camera stream output and that slightly damaged "raw" stream is disabled during calibration so this guidance doesn't work with PV.]

References:

https://arxiv.org/pdf/1907.04096.pdf

https://www.calibdb.net/#

https://github.com/paroj/pose_calib