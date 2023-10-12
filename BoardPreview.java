package calibrator;
import static calibrator.ArrayUtils.brief;

import java.util.logging.Level;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class BoardPreview {
// generate styled board image
    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}

    // class BoardPreview:
//     SIZE = (640, 480) # different than camera res okay and it runs a little faster if smaller. Resize at end makes images match
    Size SIZE = new Size(640., 480.);
    Size sz;
    Mat img = new Mat();
    Mat shadow; // used for overlap score
    Mat maps; // 2D version used for remap function
    Mat Knew;

// BoardPreview 1><(1680, 2520)><(1680, 2520) input img 1 channel
// BoardPreview 2><(1680, 2520, 3)><(1680, 2520)><(1680, 2520) converted to 3 channel color; shadow is 1 channel
//imggray  Mat [ 1680*2520*CV_8UC1, isCont=true, isSubmat=false, nativeObj=0x1bfc11b59e0, dataAddr=0x1bfc1c6f080 ] 
//imgcolor Mat [ 1680*2520*CV_8UC3, isCont=true, isSubmat=false, nativeObj=0x1bfc11b59e0, dataAddr=0x1bfc302a080 ] 
    public BoardPreview(Mat img)
    {
        Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        img.copyTo(this.img);

        Core.flip(this.img, this.img, 0);

        // set black to gray
        byte[] img1BuffRow = new byte[this.img.cols()*this.img.channels()]; // temp buffers for efficient access to a row
        // process one row at a time for efficiency
        for(int row = 0; row < img.rows(); row++)
        {
            this.img.get(row, 0, img1BuffRow); // get the row, all channels
            for(int col = 0; col < img1BuffRow.length; col++) // process each element of the row
            {
                // if there is a non-black pixel then use it in img
                if(img1BuffRow[col] == 0)
                {
                    img1BuffRow[col] = 64;
                }
            }
            this.img.put(row, 0, img1BuffRow);
        }

        Imgproc.cvtColor(this.img, this.img, Imgproc.COLOR_GRAY2BGR);

        // set blue and red to black so white becomes green
        byte[] img3BuffRow = new byte[this.img.cols()*this.img.channels()]; // temp buffers for efficient access to a row
        // process one row at a time for efficiency
        for(int row = 0; row < this.img.rows(); row++)
        {
            this.img.get(row, 0, img3BuffRow); // get the row, all channels
            for(int col = 0; col < img3BuffRow.length; col+=3) // process each triplet (channels) of the row
            {
                img3BuffRow[col] = 0; // B
                img3BuffRow[col+2] = 0; // R
            }
            this.img.put(row, 0, img3BuffRow);
        }

        // used for overlap score
        this.shadow = Mat.ones(this.img.rows(), this.img.cols(), CvType.CV_8UC1);
    }
/*
create_maps cdist is None
create_maps K [[1000.0000 0.0000 639.5000]
 [0.0000 1000.0000 359.5000]
 [0.0000 0.0000 1.0000]]
create_maps><(640, 480)><(1280, 720)><(1280, 720)><(3, 3)><(3, 3)>
[[0.5000 0.0000 0.0000]
 [0.0000 0.6667 0.0000]
 [0.0000 0.0000 1.0000]]><(3, 3)><
 [[500.0000 0.0000 319.7500]
 [0.0000 666.6667 239.6667]
 [0.0000 0.0000 1.0000]]
create_maps Knew
 [[499.2188 0.0000 319.2504]
 [0.0000 665.2778 239.1674]
 [0.0000 0.0000 1.0000]]
create_maps (maps)><(2, 480, 640)><(2, 480, 640)><[[[0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
  [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
  [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
  ...
  [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
  [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
  [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]]

 [[-0.0000 -0.0000 -0.0000 ... -0.0000 -0.0000 -0.0000]
  [0.9979 0.9979 0.9979 ... 0.9979 0.9979 0.9979]
  [1.9958 1.9958 1.9958 ... 1.9958 1.9958 1.9958]
  ...
  [476.0062 476.0062 476.0062 ... 476.0062 476.0062 476.0062]
  [477.0042 477.0042 477.0042 ... 477.0042 477.0042 477.0042]
  [478.0021 478.0021 478.0021 ... 478.0021 478.0021 478.0021]]]><[[0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
 [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
 [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
 ...
 [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
 [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
 [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]]><[[-0.0000 -0.0000 -0.0000 ... -0.0000 -0.0000 -0.0000]
 [0.9979 0.9979 0.9979 ... 0.9979 0.9979 0.9979]
 [1.9958 1.9958 1.9958 ... 1.9958 1.9958 1.9958]
 ...
 [476.0062 476.0062 476.0062 ... 476.0062 476.0062 476.0062]
 [477.0042 477.0042 477.0042 ... 477.0042 477.0042 477.0042]
 [478.0021 478.0021 478.0021 ... 478.0021 478.0021 478.0021]]
set_next_pose self.img_size cdist create_maps out (1280, 720) None
project_img
img (1680, 2520, 3)
sz (640, 480)
K [[499.2188 0.0000 319.2504]
 [0.0000 665.2778 239.1674]
 [0.0000 0.0000 1.0000]]
rvec [2.7188 -0.5408 -1.1262]
t [-595.8323 1258.2399 4627.8323]
flags 1
R [[0.6533 -0.2706 -0.7071]
 [-0.3827 -0.9239 -0.0000]
 [-0.6533 0.2706 -0.7071]]
H (3, 3) [[0.0254 -0.0105 254.9761]
 [-0.0888 -0.1188 420.0467]
 [-0.0001 0.0001 1.0000]]
imgProjected [[[0 0 0]
  [0 0 0]
  [0 0 0]
  ...
  [0 0 0]
  [0 0 0]
  [0 0 0]]

 [[0 0 0]
  [0 0 0]
  [0 0 0]
  ...
  [0 0 0]
  [0 0 0]
  [0 0 0]]

 [[0 0 0]
  [0 0 0]
  [0 0 0]
  ...
  [0 0 0]
  [0 0 0]
  [0 0 0]]

 ...

 [[0 0 0]
  [0 0 0]
  [0 0 0]
  ...
  [0 0 0]
  [0 0 0]
  [0 0 0]]

 [[0 0 0]
  [0 0 0]
  [0 0 0]
  ...
  [0 0 0]
  [0 0 0]
  [0 0 0]]

 [[0 0 0]
  [0 0 0]
  [0 0 0]
  ...
  [0 0 0]
  [0 0 0]
  [0 0 0]]]
shadow flag><False><[[1 1 1 ... 1 1 1]
 [1 1 1 ... 1 1 1]
 [1 1 1 ... 1 1 1]
 ...
 [1 1 1 ... 1 1 1]
 [1 1 1 ... 1 1 1]
 [1 1 1 ... 1 1 1]]><(1680, 2520, 3)><(640, 480)><[[499.2188 0.0000 319.2504]
 [0.0000 665.2778 239.1674]
 [0.0000 0.0000 1.0000]]><[2.7188 -0.5408 -1.1262]><[-595.8323 1258.2399 4627.8323]
maps (2, 480, 640) [[[0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
  [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
  [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
  ...
  [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
  [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
  [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]]

 [[-0.0000 -0.0000 -0.0000 ... -0.0000 -0.0000 -0.0000]
  [0.9979 0.9979 0.9979 ... 0.9979 0.9979 0.9979]
  [1.9958 1.9958 1.9958 ... 1.9958 1.9958 1.9958]
  ...
  [476.0062 476.0062 476.0062 ... 476.0062 476.0062 476.0062]
  [477.0042 477.0042 477.0042 ... 477.0042 477.0042 477.0042]
  [478.0021 478.0021 478.0021 ... 478.0021 478.0021 478.0021]]]
maps elements 0 and 1 [[0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
 [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
 [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
 ...
 [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
 [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]
 [0.0000 0.9984 1.9969 ... 636.0047 637.0031 638.0016]] [[-0.0000 -0.0000 -0.0000 ... -0.0000 -0.0000 -0.0000]
 [0.9979 0.9979 0.9979 ... 0.9979 0.9979 0.9979]
 [1.9958 1.9958 1.9958 ... 1.9958 1.9958 1.9958]
 ...
 [476.0062 476.0062 476.0062 ... 476.0062 476.0062 476.0062]
 [477.0042 477.0042 477.0042 ... 477.0042 477.0042 477.0042]
 [478.0021 478.0021 478.0021 ... 478.0021 478.0021 478.0021]] (480, 640, 3)
img after1 (480, 640, 3)
img after2 (720, 1280, 3)
board warped 2764800 (720, 1280, 3)
done
/////////////////////////////////
2023-10-11 10:40:27.213 SEVERE  [ calibrator.BoardPreview create_maps] method entered  . . . . . . . . . . . . . . . . . . . . . . . . 
2023-10-11 10:40:27.216 SEVERE  [ calibrator.BoardPreview create_maps] K
[1000, 0, 639.5;
 0, 1000, 359.5;
 0, 0, 1] 
2023-10-11 10:40:27.219 SEVERE  [ calibrator.BoardPreview create_maps] cdist [0, 0, 0, 0, 0] 
2023-10-11 10:40:27.226 SEVERE  [ calibrator.BoardPreview create_maps] sz 1280x720 
2023-10-11 10:40:27.228 SEVERE  [ calibrator.BoardPreview create_maps] K scaled
[500, 0, 319.75;
 0, 666.6666666666666, 239.6666666666667;
 0, 0, 1] 
2023-10-11 10:40:27.233 SEVERE  [ calibrator.BoardPreview create_maps] Knew null true 
2023-10-11 10:40:27.242 SEVERE  [ calibrator.BoardPreview create_maps] Knew
Mat [ 3*3*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x202dc89b740, dataAddr=0x202dca89080 ]
 [[499.2188 0.0000 319.2504]
 [0.0000 665.2778 239.1674]
 [0.0000 0.0000 1.0000]]

 [[499.2188 0.0000 319.2504]
 [0.0000 665.2778 239.1674]
 [0.0000 0.0000 1.0000]]
2023-10-11 10:40:27.246 CONFIG  [ calibrator.Distortion <clinit>] Starting ---------------------------------------- 
2023-10-11 10:40:27.249 SEVERE  [ calibrator.Distortion make_distort_map] method entered  . . . . . . . . . . . . . . . . . . . . . . . . 
2023-10-11 10:40:27.251 SEVERE  [ calibrator.Distortion make_distort_map] camera matrix K Mat [ 3*3*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x202dc8f73b0, dataAddr=0x202dc820a00 ]
[500, 0, 319.75;
 0, 666.6666666666666, 239.6666666666667;
 0, 0, 1] 
2023-10-11 10:40:27.253 SEVERE  [ calibrator.Distortion make_distort_map] sz 640x480 
2023-10-11 10:40:27.254 SEVERE  [ calibrator.Distortion make_distort_map] distortion coefficients dist Mat [ 1*5*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x202dc8f68c0, dataAddr=0x202dc82c040 ]
[0, 0, 0, 0, 0] 
2023-10-11 10:40:27.256 SEVERE  [ calibrator.Distortion make_distort_map] Knew [500, 0, 319.7500000000001;
 0, 666.6666666666666, 239.6666666666667;
 0, 0, 1] 
2023-10-11 10:40:27.472 SEVERE  [ calibrator.Distortion make_distort_map] pts Mat [ 307200*1*CV_32FC2, isCont=true, isSubmat=false, nativeObj=0x202dc89b970, dataAddr=0x202de406080 ]
Mat [ 307200*1*CV_32FC2, isCont=true, isSubmat=false, nativeObj=0x202dc89b970, dataAddr=0x202de406080 ]
0.0 0.0 
1.0 0.0 
2.0 0.0 
3.0 0.0 
........................................................................................................................................

*/
    public void create_maps(Mat K, Mat cdist, Size sz)
    {
        Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        Main.LOGGER.log(Level.SEVERE, "camera matrix K " + K + "\n" + K.dump());
        Main.LOGGER.log(Level.SEVERE, "cdist " + cdist.dump());
        Main.LOGGER.log(Level.SEVERE, "sz " + sz);

        // cdist initialized in its constructor instead of setting to 0 here if null; did 5 not 4 for consistency with rest of code
        this.sz = sz;
        Mat scale = Mat.zeros(3, 3, K.type()); // assuming it's K.rows(), K.cols()
        scale.put(0, 0, SIZE.width/sz.width);
        scale.put(1, 1, SIZE.height/sz.height);
        scale.put(2, 2, 1.);
        Core.gemm(scale, K, 1., new Mat(), 0.,K);
        sz = this.SIZE;
        Main.LOGGER.log(Level.SEVERE, "K scaled\n" + K.dump());
        Main.LOGGER.log(Level.SEVERE, "Knew null " + (this.Knew == null)); // matches to here

        this.Knew = Calib3d.getOptimalNewCameraMatrix(K, cdist, sz, 1.); // .2% higher than Python for same input
        Main.LOGGER.log(Level.SEVERE, "Knew\n" + this.Knew + this.Knew.dump());
        /*[500, 0, 319.7500000000001;
 0, 666.6666666666666, 239.6666666666667;
 0, 0, 1] */
/*2023-10-11 10:40:27.242 SEVERE  [ calibrator.BoardPreview create_maps] Knew
Mat [ 3*3*CV_64FC1, isCont=true, isSubmat=false, nativeObj=0x202dc89b740, dataAddr=0x202dca89080 ]
[500, 0, 319.7500000000001;
 0, 666.6666666666666, 239.6666666666667;
 0, 0, 1]
 
 create_maps Knew
 [[499.2188 0.0000 319.2504]
 [0.0000 665.2778 239.1674]
 [0.0000 0.0000 1.0000]]
 */
        this.maps = Distortion.make_distort_map(K, sz, cdist, this.Knew);

        Main.LOGGER.log(Level.SEVERE, "Knew " + this.Knew + "\n" + this.Knew.dump()); // same as input to make_distort_map
        Main.LOGGER.log(Level.SEVERE, "maps " + this.maps + "\n" + brief(this.maps));
    }

    public Mat project(Mat r, Mat t, boolean useShadow, int inter)
    // force users to specify useShadow=false and inter=Imgproc.INTER_NEAREST instead of defaulting
    // no default allowed in Java and I don't feel like making a bunch of overloaded methods for this conversion
    {
        Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        Main.LOGGER.log(Level.SEVERE, "r " + r.dump());
        Main.LOGGER.log(Level.SEVERE, "t " + t.dump());
        Main.LOGGER.log(Level.SEVERE, "useShadow " + useShadow);
        Main.LOGGER.log(Level.SEVERE, "inter " + inter);
        Main.LOGGER.log(Level.SEVERE, "sz " + this.sz);

        Mat img = new Mat();

        img = project_img(useShadow ? this.shadow : this.img, this.SIZE, this.Knew, r, t, Imgproc.INTER_LINEAR);

        Main.LOGGER.log(Level.SEVERE, "maps " + this.maps + "\n" + brief(maps));
        // Can be one map for XY or two maps X and Y. python had 2 and this has 1
        // Imgproc.remap(img, img, maps[0]/*X*/, maps[1]/*Y*/, inter);// maybe X Mat and Y Mat somehow; separate channels?

        Imgproc.remap(img, img, this.maps, new Mat(), inter);// 1st arg can be XY with no 2nd arg
        Main.LOGGER.log(Level.SEVERE, "img after remap " + img + "\n" + brief(img));

        // maps (2, 480, 640)
        Imgproc.resize(img, img, this.sz, 0, 0, inter);

        Main.LOGGER.log(Level.SEVERE, "returning img after resize " + img + "\n" + brief(img));

        return img;
    }

    public static Mat project_img(Mat img, Size sz, Mat K, Mat rvec, Mat t, int flags)
    // force user to specify flags=cv2.INTER_LINEAR to use default
    {
        Main.LOGGER.log(Level.SEVERE, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        Main.LOGGER.log(Level.SEVERE, "img " + img);
        Main.LOGGER.log(Level.SEVERE, "sz " + sz);
        Main.LOGGER.log(Level.SEVERE, "K " + K + "\n" + K.dump());
        Main.LOGGER.log(Level.SEVERE, "rvec " + rvec + rvec.dump());
        Main.LOGGER.log(Level.SEVERE, "t " + t + t.dump());
        Main.LOGGER.log(Level.SEVERE, "flags " + flags);

        // construct homography
        Mat R = new Mat();
        Calib3d.Rodrigues(rvec, R);

        // H = K.dot(np.array([R[:, 0], R[:, 1], t]).T)
        // H /= H[2, 2]
        Main.LOGGER.log(Level.SEVERE, "R " + R + R.dump());
        Mat transform = new Mat(3, 3, CvType.CV_64FC1); // rotation matrix R and a translation matrix T (t)
        transform.put(0, 0,
            R.get(0, 0)[0], R.get(0, 1)[0], R.get(0, 2)[0],  // 1st row r,second row r, third row t
                R.get(1, 0)[0], R.get(1, 1)[0], R.get(1, 2)[0],
                t.get(0, 0)[0], t.get(0, 1)[0], t.get(0, 2)[0]);
        // transform.put(0, 0,
        // R.get(0, 0)[0], R.get(0, 1)[0], R.get(0, 2)[0],  // 1st row r,second row r, third row t
        //     R.get(1, 0)[0], R.get(1, 1)[0], R.get(1, 2)[0],
        //     t.get(0, 0)[0], t.get(0, 1)[0], t.get(0, 2)[0]);
        Core.transpose(transform, transform); // then transpose since I didn't enter it the other way
        Mat H = new Mat();
        Core.gemm(K, transform, 1., new Mat(), 0., H);
        Core.divide(H, new Scalar(H.get(2, 2)[0]), H);
        transform.release();
        // double[] patchH = { // test data
        //     0.0959, -0.0397, 190.4522,
        //     -0.1163, -0.1852, 510.3200,
        //     -0.0001, 0.0001, 1.0000 };
        // H.put(0, 0, patchH);    
        Main.LOGGER.log(Level.SEVERE, "transform " + transform + "\n" + transform.dump());
        Main.LOGGER.log(Level.SEVERE, "R " + R + "\n" + R.dump());
        Main.LOGGER.log(Level.SEVERE, "H " + H + "\n" +H.dump());

        Mat imgProjected = new Mat();

        Imgproc.warpPerspective(img, imgProjected, H, sz, flags);

        // Calib3d.drawFrameAxes(imgProjected, K, cdist, R, t, 300.f); //FIXME may need rotation vector rvec instead of R

        R.release();
        H.release();

        Main.LOGGER.log(Level.SEVERE, "returning imgProjected\n" + brief(imgProjected));

        return imgProjected;
    }
}
/*
java:

python:
project_img
ok img (1680, 2520, 3)
ok sz (640, 480)
ok K [[998.4375 0.0000 319.0008]
 [0.0000 997.9166 239.0010]
 [0.0000 0.0000 1.0000]]
ok rvec [2.7188 -0.5408 -1.1262]
ok t [-595.8323 1258.2399 4627.8323]
ok flags 1
ok R [[0.6533 -0.2706 -0.7071]
 [-0.3827 -0.9239 -0.0000]
 [-0.6533 0.2706 -0.7071]]
ok H (3, 3) [[0.0959 -0.0397 190.4522]
 [-0.1163 -0.1852 510.3200]
 [-0.0001 0.0001 1.0000]]
         */
// import cv2
// import numpy as np

// from distvis import make_distort_map

// class BoardPreview:
//     SIZE = (640, 480) # different than camera res okay and it runs a little faster if smaller. Resize at end makes images match

//     def __init__(self, img):
//         # generate styled board image
//         self.img = img
//         self.img = cv2.flip(self.img, 0)  # flipped when printing
//         self.img[self.img == 0] = 64  # set black to gray
//         self.img = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR) # guidance board becomes 3 channel black and white
//         self.img[:, :, 0::2] = 0  # set blue (0) and red (2) to zero and the white becomes green (1)

//         self.shadow = np.ones(self.img.shape[:2], dtype=np.uint8)  # used for overlap score
    
//     def create_maps(self, K, cdist, sz):
//         if cdist is None:
//             cdist = np.array([0., 0., 0., 0.])
        
//         self.sz = sz
//         scale = np.diag((self.SIZE[0]/sz[0], self.SIZE[1]/sz[1], 1))
//         K = scale.dot(K)
        
//         sz = self.SIZE
//         self.Knew = cv2.getOptimalNewCameraMatrix(K, cdist, sz, 1)[0]
//         self.maps = make_distort_map(K, sz, cdist, self.Knew)

//     def project(self, r, t, shadow=False, inter=cv2.INTER_NEAREST):
//         img = project_img(self.shadow if shadow else self.img, self.SIZE, self.Knew, r, t)
//         img = cv2.remap(img, self.maps[0], self.maps[1], inter)
//         img = cv2.resize(img, self.sz, interpolation=inter)
//         return img

//     project_img(img, sz, K, rvec, t, flags=cv2.INTER_LINEAR)
//     {
//         // """
//         // projects a 2D object (image) according to parameters
//         // @param img: image to project
//         // @param sz: size of the final image  
//         // """
//         // # construct homography
//         R = cv2.Rodrigues(rvec)[0]
//         H = K.dot(np.array([R[:, 0], R[:, 1], t]).T)
//         H /= H[2, 2]

//         imgProjected = cv2.warpPerspective(img, H, sz, flags=flags)
//         cv2.drawFrameAxes(imgProjected, K, None, R, t, 300)
//         return imgProjected
//         public static void warpPerspective​(Mat src, Mat dst, Mat M, Size dsize, int flags)
//         Imgproc.warpPerspective();
//         return cv2.warpPerspective(img, H, sz, flags=flags)
//     }
