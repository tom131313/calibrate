package Guidance;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.logging.Logger;
import java.util.zip.CRC32;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.objdetect.CharucoBoard;
import org.opencv.objdetect.Dictionary;
import org.opencv.objdetect.Objdetect;

public class ChArUcoBoardPrint {
	private static Logger LOGGER;
	static {
	  LOGGER = Logger.getLogger("");
	  LOGGER.finer("Loading");     
	}

    private ChArUcoBoardPrint() {}

    // Charuco Board configuration (duplicates ChArucoDetector)
    private static Size board_sz = new Size(Cfg.board_x, Cfg.board_y);
    private static final Dictionary dictionary = Objdetect.getPredefinedDictionary(Cfg.dictionary);
    private static final Size boardImageSize = new Size(Cfg.board_x*Cfg.square_len, Cfg.board_y*Cfg.square_len);
    static final Mat boardImage = new Mat();
    private static final CharucoBoard board = new CharucoBoard(board_sz, Cfg.square_len, Cfg.marker_len, dictionary);

    public static void print()
    {
        LOGGER.finer("Instantiating");

        /// create board
        board.generateImage(boardImageSize, boardImage);

        // write ChArUco Board to file for print to use for calibration
        
        /* PNG */
        final String boardFilePNG = Cfg.boardFile + ".png";
        try (FileOutputStream outputStreamPNG = new FileOutputStream(new File(boardFilePNG))) {
            LOGGER.info("ChArUcoBoard to be printed is in file " + boardFilePNG);

            byte[] boardByte = new byte[boardImage.rows()*boardImage.cols()]; // assumes 1 channel Mat [ 1680*2520*CV_8UC1, isCont=true, isSubmat=false, nativeObj=0x294e475cc20, dataAddr=0x294e55f7080 ]

            CRC32 crc32 = new CRC32();

            // SIGNATURE
            final byte[] signaturePNG =
                { //     ‰           P           N           G          CR          LF         SUB          LF
                (byte)0x89, (byte)0x50, (byte)0x4e, (byte)0x47, (byte)0x0d, (byte)0x0a, (byte)0x1a, (byte)0x0a // PNG magic number
                };
            outputStreamPNG.write(signaturePNG);

            // HEADER
            byte[] IHDR =
            {
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x0d, // chunk length (not including length, "IHDR", crc)
                (byte)0x49, (byte)0x48, (byte)0x44, (byte)0x52, // "IHDR"
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
            int ihdrDataWidthOffset = 8;
            int ihdrDataHeightOffset = 12;
            ArrayUtils.intToByteArray(boardImage.cols(), IHDR, ihdrDataWidthOffset);
            ArrayUtils.intToByteArray(boardImage.rows(), IHDR, ihdrDataHeightOffset);

            crc32.reset();
            crc32.update(IHDR, 4, IHDR.length-8); // skip the beginning 4 for length and ending 4 for crc
            ArrayUtils.intToByteArray((int)crc32.getValue(), IHDR, IHDR.length-4);
            outputStreamPNG.write(IHDR);

            // PHYSICAL RESOLUTION
            byte[] pHYs = // varies with the requested resolution [pixels per meter]
                {
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x09, // chunk length (not including length, "pHYs", crc)
                (byte)0x70, (byte)0x48, (byte)0x59, (byte)0x73, // "pHYs"
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00, // x res [pixels per unit] place holder
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00, // y res [pixels per unit] place holder
                (byte)0x01,                                     // units [unit is meter]
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00  // crc place holder
                };
            int physXresOffset = 8;
            int physYresOffset = 12;
            ArrayUtils.intToByteArray(Cfg.resXDPM, pHYs, physXresOffset);
            ArrayUtils.intToByteArray(Cfg.resYDPM, pHYs, physYresOffset);

            crc32.reset();
            crc32.update(pHYs, 4, pHYs.length-8); // skip the beginning 4 for length and ending 4 for crc
            ArrayUtils.intToByteArray((int)crc32.getValue(), pHYs, pHYs.length - 4);
            outputStreamPNG.write(pHYs);

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
                (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00, // chunk length (not including length, "IEND", crc)
                (byte)0x49, (byte)0x45, (byte)0x4e, (byte)0x44, // "IEND"
                (byte)0xae, (byte)0x42, (byte)0x60, (byte)0x82  // crc
                };
            
            outputStreamPNG.write(IEND);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
