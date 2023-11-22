package org.photonvision.calibrator;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.zip.CRC32;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.objdetect.CharucoBoard;
import org.opencv.objdetect.CharucoDetector;
import org.opencv.objdetect.CharucoParameters;
import org.opencv.objdetect.DetectorParameters;
import org.opencv.objdetect.Dictionary;
import org.opencv.objdetect.Objdetect;
import org.opencv.objdetect.RefineParameters;

/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     ChArucoDetector class                                       */
/*                                     ChArucoDetector class                                       */
/*                                     ChArucoDetector class                                       */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
public class ChArucoDetector {
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     ChArucoDetector constructor                                 */
/*                                     ChArucoDetector constructor                                 */
/*                                     ChArucoDetector constructor                                 */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    static {Main.LOGGER.log(Level.CONFIG, "Starting ----------------------------------------");}
 
    // configuration
    private Size board_sz = new Size(Cfg.board_x, Cfg.board_y);
    private double square_len = Cfg.square_len;
    private double marker_len = Cfg.marker_len;
    Size img_size = new Size(Cfg.image_width, Cfg.image_height);

    // per frame data

    // p3d is the object coordinates of the perfect undistorted ChArUco Board corners that the camera is pointing at.
    // In this case the board is flat at its Z origin (say, the wall upon which it is mounted) so the Z coordinate is always 0.
    // p2d is the coordinates of the corresponding board corners as they are located in the camera image,
    // distorted by perspective (pose) and camera intrinsic parameters and camera distortion.
    private final Mat p3d = new Mat(); // 3 dimensional currentObjectPoints, the physical target ChArUco Board
    private final Mat p2d = new Mat(); // 2 dimensional currentImagePoints, the likely distorted board on the flat camera sensor frame posed relative to the target
    private int N_pts = 0;
    private boolean pose_valid = false;
    // private Mat raw_img = null; // not used

    /// Charuco Board
    private final Dictionary dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50);
    private final Size boardImageSize = new Size(Cfg.board_x*Cfg.square_len, Cfg.board_y*Cfg.square_len);
    final Mat boardImage = new Mat();
    private final CharucoBoard board = new CharucoBoard(this.board_sz, Cfg.square_len, Cfg.marker_len, this.dictionary);
    private CharucoDetector detector; // the OpenCV detector spelled almost the same - fooled me too many times!!!!!

    private Mat rvec = new Mat();
    private Mat tvec = new Mat();

    private boolean intrinsic_valid = false;
    private Mat K;
    private Mat cdist;

    private final Mat ccorners = new Mat(); // currentCharucoCorners
    private final Mat cids = new Mat(); // currentCharucoIds

    // optical flow calculation
    private Mat last_ccorners = new Mat(); // previous ChArUcoBoard corners
    private Mat last_cids = new Mat(); // previous ChArUcoBoard ids
    private double mean_flow = Double.MAX_VALUE; // mean flow of the same corners that are detected in consecutive frames (relaxed from original)

    // getters
    int N_pts()
    {
        return N_pts;
    }
    Size board_sz()
    {
        return board_sz;
    }
    boolean pose_valid()
    {
        return this.pose_valid;
    }
    Mat rvec()
    {
        return rvec;
    }
    Mat tvec()
    {
        return tvec;
    }
    double mean_flow()
    {
        return this.mean_flow;
    }

    public ChArucoDetector() throws FileNotFoundException, IOException
    {
        // Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        /// create board
        this.board.generateImage(this.boardImageSize, this.boardImage);

        if (Cfg.writeBoard)
        {
            // write ChArUco Board to file for print to use for calibration
            final String boardFile = Cfg.boardFile;
            
            // /* jpg */
            // final MatOfInt writeBoardParamsJpg = new MatOfInt( // pair-wise; param1, value1, ...
            //     Imgcodecs.IMWRITE_JPEG_QUALITY, 100); // no compression

            // Imgcodecs.imwrite(
            // file + ".jpg",
            // this.boardImage,
            // writeBoardParamsJpg);
            // Main.LOGGER.log(Level.SEVERE, "ChArUcoBoard to be printed is in file " + file + ".jpg");

            // if ( ! Cfg.isPV) // no tiff writer in PV RPi OpenCV
            // {
            //     /* tiff */
            //     final MatOfInt writeBoardParamsTiff = new MatOfInt( // pair-wise; param1, value1, ...
            //     Imgcodecs.IMWRITE_TIFF_COMPRESSION, 1, // no compression
            //     Imgcodecs.IMWRITE_TIFF_XDPI, Cfg.guidanceTiffDPIx,
            //     Imgcodecs.IMWRITE_TIFF_YDPI, Cfg.guidanceTiffDPIy);

            //     Imgcodecs.imwrite(
            //     file + ".tiff",
            //     this.boardImage,
            //     writeBoardParamsTiff);
            //     Main.LOGGER.log(Level.SEVERE, "ChArUcoBoard to be printed is in file " + file + ".tiff");
            // }

            /* PNG */
            final String boardFilePNG = boardFile + ".png";
            FileOutputStream outputStreamPNG = new FileOutputStream(new File(boardFilePNG));
            Main.LOGGER.log(Level.SEVERE, "ChArUcoBoard to be printed is in file " + boardFilePNG);

            byte[] boardByte = new byte[this.boardImage.rows()*this.boardImage.cols()]; // assumes 1 channel Mat [ 1680*2520*CV_8UC1, isCont=true, isSubmat=false, nativeObj=0x294e475cc20, dataAddr=0x294e55f7080 ]

            CRC32 crc32 = new CRC32();

            // SIGNATURE
            final byte[] signaturePNG =
                {
                (byte)0x89, (byte)0x50, (byte)0x4e, (byte)0x47, (byte)0x0d, (byte)0x0a, (byte)0x1a, (byte)0x0a // PNG magic number
                };
            outputStreamPNG.write(signaturePNG);

            // HEADER
            byte[] IHDR =
            {
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x0d, // length
                (byte)0x49, (byte)0x48, (byte)0x44, (byte)0x52, // IHDR
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00, // data width place holder
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00, // data height place holder
                (byte)0x08,                                     // bit depth
                (byte)0x00,                                     // color type - grey scale
                (byte)0x00,                                     // compression method
                (byte)0x00,                                     // filter method (default/only one?)
                (byte)0x00,                                     // interlace method
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00  // crc place holder
            };
            // fetch the length data for the IHDR
            int ihdrWidthOffset = 8;
            int ihdrHeightOffset = 12;
            ArrayUtils.intToByteArray(boardImage.cols(), IHDR, ihdrWidthOffset);
            ArrayUtils.intToByteArray(boardImage.rows(), IHDR, ihdrHeightOffset);
    
            crc32.reset();
            crc32.update(IHDR, 4, IHDR.length-8); // skip the beginning 4 for length and ending 4 for crc
            ArrayUtils.intToByteArray((int)crc32.getValue(), IHDR, IHDR.length-4);
            outputStreamPNG.write(IHDR);

            // PHYSICAL RESOLUTION
            byte[] PHYS = // varies with the requested resolution [pixels per meter]
                {
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x09, // length
                (byte)0x70, (byte)0x48, (byte)0x59, (byte)0x73, // pHYs
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00, // x res [pixels per unit] place holder
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00, // y res [pixels per unit] place holder
                (byte)0x01,                                     // units [unit is meter]
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00  // crc place holder
                };
            int physXresOffset = 8;
            int physYresOffset = 12;
            ArrayUtils.intToByteArray(Cfg.resXDPM, PHYS, physXresOffset);
            ArrayUtils.intToByteArray(Cfg.resYDPM, PHYS, physYresOffset);

            crc32.reset();
            crc32.update(PHYS, 4, PHYS.length-8); // skip the beginning 4 for length and ending 4 for crc
            ArrayUtils.intToByteArray((int)crc32.getValue(), PHYS, PHYS.length - 4);
            outputStreamPNG.write(PHYS);

            // DATA
            //The complete filtered PNG image is represented by a single zlib datastream that is stored in a number of IDAT chunks.

            // create the filtered, compressed datastream

            boardImage.get(0, 0, boardByte); // board from OpenCV Mat

            // filter type begins each row so step through all the rows adding the filter type to each row
            byte[] boardByteFilter = new byte[boardImage.rows() + boardByte.length];
            int flatIndex = 0;
            int flatIndexFilter = 0;
            for (int row = 0; row < boardImage.rows(); row++)
            {
                boardByteFilter[flatIndexFilter++] = 0x00; // filter type none begins each row          
                for (int col = 0; col < boardImage.cols(); col++)
                {
                    boardByteFilter[flatIndexFilter++] = boardByte[flatIndex++];
                }
            }
            // complete filtered PNG image is represented by a single zlib compression datastream
            byte[] boardCompressed = ArrayUtils.compress(boardByteFilter);

            // chunk the compressed datastream
            // chunking not necessary for the ChArUcoBoard but it's potentially good for other uses
            int chunkSize = 0;
            int chunkSizeMax = 100_000; // arbitrary "small" number
            int dataWritten = 0;

            while (dataWritten < boardCompressed.length) // chunk until done
            {
                chunkSize = Math.min(chunkSizeMax, boardCompressed.length - dataWritten); // max or what's left in the last chunk

                byte[] IDAT = new byte[4 + 4 + chunkSize + 4]; // 4 length + 4 "IDAT" + chunk length + 4 CRC

                ArrayUtils.intToByteArray(chunkSize, IDAT, 0); // stash length of the chunk data in first 4 bytes
                IDAT[4] = (byte)("IDAT".charAt(0));
                IDAT[5] = (byte)("IDAT".charAt(1));
                IDAT[6] = (byte)("IDAT".charAt(2));
                IDAT[7] = (byte)("IDAT".charAt(3));
                for(int i=0; i < chunkSize; i++)
                {
                    IDAT[8 + i] = boardCompressed[dataWritten + i]; // stash data from where we left off to its place in the chunk
                }

                crc32.reset();
                crc32.update(IDAT, 4, IDAT.length - 8); // skip the beginning 4 for length and ending 4 for crc
                ArrayUtils.intToByteArray((int)crc32.getValue(), IDAT, IDAT.length - 4); // crc in last 4 bytes  

                outputStreamPNG.write(IDAT);
                dataWritten += chunkSize;
           }

            // END
            final byte[] IEND =
                {
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00, // length
                (byte)0x49, (byte)0x45, (byte)0x4e, (byte)0x44, // IEND
                (byte)0xae, (byte)0x42, (byte)0x60, (byte)0x82  // crc
                };
            
            outputStreamPNG.write(IEND);

            outputStreamPNG.close();
        }
        /// end create board

        /// board detector
        final DetectorParameters detectParams = new DetectorParameters();
        final RefineParameters refineParams = new RefineParameters();
        final CharucoParameters charucoParams = new CharucoParameters();

        charucoParams.set_minMarkers(Cfg.pt_min_markers); // 2 default
        charucoParams.set_tryRefineMarkers(Cfg.tryRefineMarkers); // false default
        // charucoParams.set_cameraMatrix();
        // charucoParams.set_distCoeffs();
        detectParams.set_cornerRefinementMaxIterations(Cfg.cornerRefinementMaxIterations); // 30 default
        detectParams.set_cornerRefinementMethod(Cfg.cornerRefinementMethod); // 0 default
        refineParams.set_checkAllOrders(Cfg.checkAllOrders); // true default
        refineParams.set_errorCorrectionRate(Cfg.errorCorrectionRate); // 3.0 default
        refineParams.set_minRepDistance(Cfg.minRepDistance); // 10.0 default

        detector = new CharucoDetector(this.board, charucoParams, detectParams, refineParams);
    }
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     set_intrinsics                                              */
/*                                     set_intrinsics                                              */
/*                                     set_intrinsics                                              */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    public void set_intrinsics(Calibrator calib)
    {
        // Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        this.intrinsic_valid = true;
        this.K = calib.K();
        this.cdist = calib.cdist();
        // Main.LOGGER.log(Level.WARNING, "K\n" + this.K.dump() + "\n" + calib.K().dump());
    }
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     draw_axis                                                   */
/*                                     draw_axis                                                   */
/*                                     draw_axis                                                   */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    /**
     * Draw axes on the detected ChArUcoBoard from the camera image
     * @param img
     */
    public void draw_axis(Mat img)
    {
        // Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        Calib3d.drawFrameAxes(
            img, this.K, this.cdist, this.rvec, this.tvec, (float)this.square_len*2.5f, 2);
    }   
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     detect_pts                                                  */
/*                                     detect_pts                                                  */
/*                                     detect_pts                                                  */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    public void detect_pts(Mat img) throws Exception
    {
        // Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        final List<Mat> markerCorners = new ArrayList();
        final Mat markerIds = new Mat();

        try
        {
            detector.detectBoard
                ( img, this.ccorners, this.cids, markerCorners, markerIds );
        }
        catch(Exception e) {Main.LOGGER.log(Level.SEVERE, img + " " + this.ccorners + " " + this.cids + "\n" + e);}

        this.N_pts = 0;
        this.mean_flow = Double.MAX_VALUE;

        if ( ! this.cids.empty() && this.cids.rows() > 0) // double check since there was some unknown failure to get the N_pts set right
        {
            this.N_pts = this.cids.rows();
        }

        // Main.LOGGER.log(Level.WARNING, "N_pts " + this.N_pts);
    
        if (this.N_pts == 0) // maybe use the min N_pts from Cfg
        {
            return;
        }

        // Main.LOGGER.log(Level.WARNING, "detected ccorners\n" + this.ccorners.dump());
        // Main.LOGGER.log(Level.WARNING, "detected cids\n" + this.cids.dump());
        
        // reformat the Mat to a List<Mat> for matchImagePoints
        final List<Mat> ccornersList = new ArrayList<>();
        for (int i = 0; i < this.ccorners.total(); i++) {
          ccornersList.add(this.ccorners.row(i));
        }

        // display the detected cids on the board (debugging)
        // Objdetect.drawDetectedCornersCharuco(img, ccorners, cids);

        board.matchImagePoints(ccornersList, this.cids,this.p3d, this.p2d); // p2d same data as ccornersList
        // oddly this method returns 3 channels instead of 2 for imgPoints and there isn't much to do about it and it works in solvePnP
        // after copying to MatOfPoint2f. A waste of cpu and memory.

        // Main.LOGGER.log(Level.WARNING, "p3d\n" + this.p3d.dump()); // data okay here
        // Main.LOGGER.log(Level.WARNING, "p2d\n" + this.p2d.dump()); // data okay here

        if (this.p3d.empty() || this.p2d.empty()) throw new Exception("p3d or p2d empty"); // shouldn't happen

        // compute mean flow of the image for the check for stillness elsewhere
        computeMeanFlow();

        this.ccorners.copyTo(this.last_ccorners);
        this.cids.copyTo(this.last_cids);
    }
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     computeMeanFlow                                             */
/*                                     computeMeanFlow                                             */
/*                                     computeMeanFlow                                             */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    /**
     * Compare current and previous corner detections to see how far they are displaced from each other - flow
     * The lists of current and previous corners must be the same and then the displacement is computed
     */
    public void computeMeanFlow()
    {
        // cids: int 1 col, 1 channel (cid); ccorners: float 1 col, 2 channels (x, y)

        this.mean_flow = Double.MAX_VALUE; // first assume default flow is big

        if (cids.rows() <= 0 || cids.rows() != last_cids.rows()) // handle first time or number of rows differ
        {
            return; // cids lists lengths differ so can't compute flow so assume it's big
        }

        // lists' lengths are the same so check the individual elements (cids) for equality

        // get all the last cids and ccorners and all the current cids and ccorners in arrays.
        // do all the computations in the arrays.

        // check that the lists of current and previous cids match
        // assume the cids and last_cids are in the same order (it's ascending but that doesn't matter)
        
        int[] last_cidsArray = new int[last_cids.rows()]; // assume 1 col 1 channel
        int[]      cidsArray = new int[     cids.rows()];

        this.last_cids.get(0, 0, last_cidsArray);
        this.     cids.get(0, 0,      cidsArray );

        for (int row = 0; row < cidsArray.length; row++)
        {
            if (cidsArray[row] != last_cidsArray[row])
            {
                return; // cids differ so can't compute flow so assume it's big
            }
        }

        // previous and current cids lists match so compute flow of each corner
  
        float[] last_ccornersArray = new float[last_ccorners.rows() * last_ccorners.channels()]; // assume 1 col
        float[]      ccornersArray = new float[     ccorners.rows() *      ccorners.channels()];

        this.last_ccorners.get(0,0, last_ccornersArray);
        this.     ccorners.get(0, 0,     ccornersArray);

        this.mean_flow = 0.; // starting at 0 for a summation process but that will change to flow or max value
        for (int rowChannel = 0; rowChannel < ccornersArray.length; rowChannel += 2) // step by 2 assumes 2 channels (x, y) per point
        {
            double diffX = ccornersArray[rowChannel    ] - last_ccornersArray[rowChannel    ]; // X channel (current - last)
            double diffY = ccornersArray[rowChannel + 1] - last_ccornersArray[rowChannel + 1]; // Y channel (current - last)

            this.mean_flow += Math.sqrt(Math.pow(diffX, 2) + Math.pow(diffY, 2)); // sum the L2 norm (Frobenious)
        }

        this.mean_flow /= ccornersArray.length; // mean of the sum of the norms
    }
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     detect                                                      */
/*                                     detect                                                      */
/*                                     detect                                                      */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    public void detect(Mat img) throws Exception
    {
        // Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");
        // raw_img never used - not converted
        this.detect_pts(img);

        if (this.intrinsic_valid)
        {
            this.update_pose();
        }
    } 
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                    get_pts3d                                                    */
/*                                    get_pts3d                                                    */
/*                                    get_pts3d                                                    */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    public Mat get_pts3d()
    {
        // Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        return this.p3d;
    }
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     get_calib_pts                                               */
/*                                     get_calib_pts                                               */
/*                                     get_calib_pts                                               */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    public keyframe get_calib_pts()
    {
        // Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        return new keyframe(this.ccorners.clone(), this.get_pts3d().clone());
    }
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     update_pose                                                 */
/*                                     update_pose                                                 */
/*                                     update_pose                                                 */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
    public void update_pose() throws Exception
    {
        // Main.LOGGER.log(Level.WARNING, "method entered  . . . . . . . . . . . . . . . . . . . . . . . .");

        if (this.N_pts < Cfg.minCorners) // original had 4; solvePnp wants 6 sometimes, and UserGuidance wants many more
        {
            // Main.LOGGER.log(Level.WARNING, "too few corners " + (this.N_pts == 0 ? "- possibly blurred by movement or bad aim" : this.N_pts));
            Main.fewCorners = true;
            this.pose_valid = false;
            return;
        }

        MatOfPoint3f p3dReTyped = new MatOfPoint3f(this.p3d);
        MatOfPoint2f p2dReTyped = new MatOfPoint2f(this.p2d);
        MatOfDouble distReTyped = new MatOfDouble(this.cdist);

        // Main.LOGGER.log(Level.WARNING, "p3d\n" + p3dReTyped.dump());
        // Main.LOGGER.log(Level.WARNING, "p2d\n" + p2dReTyped.dump());
        
        Mat rvec = new Mat(); // neither previous pose nor guidance board pose helped the solvePnP (made pose estimate worse)
        Mat tvec = new Mat(); // so don't give solvePnP a starting pose estimate

        Mat inLiers = new Mat();

        this.pose_valid = Calib3d.solvePnPRansac(
            p3dReTyped, p2dReTyped,
            this.K, distReTyped,
            rvec, tvec,
            false, 100, 8.0f, 0.99, inLiers, Calib3d.SOLVEPNP_ITERATIVE);

        // Main.LOGGER.log(Level.WARNING, "inliers " + inLiers.rows() + " of " + p3dReTyped.rows() + " " + inLiers);
        
        if ( ! this.pose_valid)
        {
            // Main.LOGGER.log(Level.WARNING, "pose not valid");
            return;            
        }

        // compress the object and image mats with only the in liers
        // if the same use the original mats if inliers < all then Compression
        
        // if (inLiers.rows() == p3dReTyped.rows())
        // {
            Calib3d.solvePnPRefineVVS(
                p3dReTyped, p2dReTyped,
                this.K, distReTyped,
                rvec, tvec,
                Cfg.solvePnPRefineVVSCriteria, Cfg.solvePnPRefineVVSLambda
                );
        // }
        // else
        // {
        //     MatOfPoint3f p3dInLiers = new MatOfPoint3f();
        //     p3dInLiers.alloc(inLiers.rows());
        //     MatOfPoint2f p2dInLiers = new MatOfPoint2f();
        //     p2dInLiers.alloc(inLiers.rows());

        //     float[] p3dArray = new float[p3dReTyped.rows()*p3dReTyped.channels()]; 
        //     float[] p2dArray = new float[p2dReTyped.rows()*p2dReTyped.channels()];
        //     float[] p3dInLiersArray = new float[inLiers.rows()*p3dInLiers.channels()]; 
        //     float[] p2dInLiersArray = new float[inLiers.rows()*p2dInLiers.channels()];
        //     int[] inLiersArray = new int[inLiers.rows()];

        //     p3dReTyped.get(0, 0, p3dArray);
        //     p2dReTyped.get(0, 0, p2dArray);
        //     inLiers.get(0, 0, inLiersArray);

        // int detectedCornerIndex;
        // for (int inLierIndex = 0; inLierIndex < inLiers.rows()*p3dReTyped.channels(); inLierIndex += p3dReTyped.channels())
        // {
        //     detectedCornerIndex = inLiersArray[inLierIndex/p3dReTyped.channels()]*p3dReTyped.channels();
        //     p3dInLiersArray[inLierIndex    ] = p3dArray[detectedCornerIndex  ];
        //     p3dInLiersArray[inLierIndex + 1] = p3dArray[detectedCornerIndex+1];
        //     p3dInLiersArray[inLierIndex + 2] = p3dArray[detectedCornerIndex+2];
        // }
        // for (int inLierIndex = 0; inLierIndex < inLiers.rows()*p2dReTyped.channels(); inLierIndex += p2dReTyped.channels())
        // {
        //     detectedCornerIndex = inLiersArray[inLierIndex/p2dReTyped.channels()]*p2dReTyped.channels();
        //     p2dInLiersArray[inLierIndex    ] = p2dArray[detectedCornerIndex  ];
        //     p2dInLiersArray[inLierIndex + 1] = p2dArray[detectedCornerIndex + 1];
        // }
        // p3dInLiers.put(0, 0, p3dInLiersArray);
        // p2dInLiers.put(0, 0, p2dInLiersArray);

        // Calib3d.solvePnPRefineVVS(
        //     p3dInLiers, p2dInLiers,
        //     this.K, distReTyped,
        //     rvec, tvec,
        //     criteria, 1.
        //     );
        // }

        //FIXME negating "x" makes the shadow for jaccard the right orientation for some unknown reason! Python doesn't need this.
        // I thought it was related to not having the "flip()" as the BoardPreview needs because the "warpPerspective" flips
        // the image, but I tried that flip and it was the wrong axis. Still a mystery
        // removing this and flipping 
        Core.multiply(rvec, new Scalar(-1., 1., 1.), rvec);

        this.rvec = rvec.t(); // t() like ravel(), solvePnp returns r and t as Mat(3, 1, )
        this.tvec = tvec.t(); // and the rest of the program uses Mat(1, 3, )

        // Main.LOGGER.log(Level.WARNING, "out rvec\n" + this.rvec.dump());
        // Main.LOGGER.log(Level.WARNING, "out tvec\n" + this.tvec.dump());
    }
}
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*                                                                                                 */
/*                                     End ChArucoDetector class                                   */
/*                                     End ChArucoDetector class                                   */
/*                                     End ChArucoDetector class                                   */
/*                                                                                                 */
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*
Python & Java
pose gen orbital_pose
Pose Gen Rod Rf r returned [2.7188 -0.5408 -1.1262]
Pose Gen Rod Rf t returned><[-595.8323 1258.2399 4627.8323]

Python
ret = estimatePoseCharucoBoard(self.ccorners, self.cids, self.board, self.K, self.cdist)
update_pose rvec [[-2.6629] [0.6387] [1.1686]]
update_pose tvec [[1930.3167] [1424.4072] [7285.3986]]

Java
solvePnP
*/
// Parking lot

// https://stackoverflow.com/questions/12794876/how-to-verify-the-correctness-of-calibration-of-a-webcam/12821056#12821056
// The correct way to check calibration accuracy is to use the reprojection error provided by OpenCV. I'm not sure why this
// wasn't mentioned anywhere in the answer or comments, you don't need to calculate this by hand - it's the return value of
// calibrateCamera. In Python it's the first return value (followed by the camera matrix, etc).
// The reprojection error is the RMS error between where the points would be projected using the intrinsic coefficients and
// where they are in the real image. Typically you should expect an RMS error of less than 0.5px - I can routinely get around 
// 0.1px with machine vision cameras.

// read or write specific byte in a file
// try (RandomAccessFile raf = new RandomAccessFile(file, "rw")) {
//     raf.seek(5); // Go to byte at offset position 5.
//     byte b = raf.readByte();
//     raf.write(70); // Write byte 70 (overwrites original byte at this offset).
// }

// import java.io.*;
// import java.util.*;
// import java.util.zip.*;
// public class CompressionTest
// {
// 	public static void main(String[] args)
// 	{
// 		Compressor compressor = new Compressor();
// 		String stringToCompress = "This is a test!";
// 		//String stringToCompress = "When in the course of human events, it becomes necessary for one people to dissolve the bands that bind them...";
// 		System.out.println("stringToCompress'" + stringToCompress + "'");
// 		byte[] bytesCompressed = compressor.compress(stringToCompress);
// 		System.out.println("bytesCompressed");
// 		Console.writeBytesAsHexadecimal(bytesCompressed);
// 		String stringDecompressed = compressor.decompressToString(bytesCompressed);
// 		System.out.println("stringDecompressed'" + stringDecompressed + "'");
// 	}
// }