package calibrator;

import java.util.logging.Level;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     ArrayUtils class                                                       */
/*                                     ArrayUtils class                                                       */
/*                                     ArrayUtils class                                                       */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
public class ArrayUtils
{
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     argmax                                                                 */
/*                                     argmax                                                                 */
/*                                     argmax                                                                 */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
	/**
	 * Find index of the maximum value in an array
	 * @param array an array
	 * @return the argmax (lowest index in case of duplicate values)
	 */
	static int argmax(double[] array)
	{
		int locationOfExtreme = 0;
		double extreme = array[locationOfExtreme];

		for (int i = 1; i < array.length; i++)
		{
			if (array[i] > extreme)
			{
				extreme = array[i];
				locationOfExtreme = i;
			}
		}
		return locationOfExtreme;
	}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     argmin                                                                 */
/*                                     argmin                                                                 */
/*                                     argmin                                                                 */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
	/**
	 * Find index of the minimum value in an array
	 * @param array an array
	 * @return the argmin (lowest index in case of duplicate values)
	 */
	static int argmin(double[] array)
	{
		int locationOfExtreme = 0;
		double extreme = array[locationOfExtreme];

		for (int i = 1; i < array.length; i++)
		{
			if (array[i] < extreme)
			{
				extreme = array[i];
				locationOfExtreme = i;
			}
		}
		return locationOfExtreme;
	}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     isAllTrue                                                              */
/*                                     isAllTrue                                                              */
/*                                     isAllTrue                                                              */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/**
 * Test for all values in a boolean array are true
 * @param array
 * @return true if all true or false if any false
 */
	static boolean isAllTrue(boolean[] array)
	{
		for(boolean element : array)
		{
			if(!element) return false;
		}
		return true;
	}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                     brief                                                                  */
/*                                     brief                                                                  */
/*                                     brief                                                                  */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/**
 * 
 * @param mat OpenCV Mat to be partially printed
 * @return String of the values in the corners of the Mat if large or entire Mat if small
 */
	static String brief(Mat mat)
	{
		StringBuilder sb = new StringBuilder();
		sb.append(mat);
		sb.append("\n");

		double[] matRowD = null;
		float[] matRowF = null;
		byte[] matRowB = null;
		int[] matRowI = null;
		short[] matRowS = null;

		switch(CvType.depth(mat.type()))
		{
			case CvType.CV_64F: // double
				matRowD = new double[mat.channels()*mat.cols()];
				break;

			case CvType.CV_32F: // float
				matRowF = new float[mat.channels()*mat.cols()];
				break;

			case CvType.CV_8U: // byte
			case CvType.CV_8S:
				matRowB = new byte[mat.channels()*mat.cols()];
				break;

			case CvType.CV_32S: // int
				matRowI = new int[mat.channels()*mat.cols()];
				break;

			case CvType.CV_16U: // short
			case CvType.CV_16S:
				matRowS = new short[mat.channels()*mat.cols()];
				break;

			default:
				Main.LOGGER.log(Level.SEVERE, "Print Mat Error - Unknown OpenCV Mat depth. Not printing requested data. " + CvType.depth(mat.type()));
				return "Print Mat Error";
		}

		int printCountRow = 0;
		int printCountCol = 0;
		boolean skippedRow = false;

		for(int row = 0; row < mat.rows(); row++)
		{
			if(row > 3 && row < Math.max(row, mat.rows()-4))
			{
				skippedRow = true;
				if(printCountRow % 2000 == 0)
				{
					printCountRow = 0;
					sb.append(".r.");
				}
				printCountRow++;
				continue;
			}
			if(skippedRow)
			{
				sb.append("\n");
				skippedRow = false;
			}

			switch(CvType.depth(mat.type()))
			{
				case CvType.CV_64F: // double
					mat.get(row, 0, matRowD);
					break;
	
				case CvType.CV_32F: // float
					mat.get(row, 0, matRowF);
					break;

				case CvType.CV_8U: // byte
				case CvType.CV_8S:
					mat.get(row, 0, matRowB);
					break;
	
				case CvType.CV_32S: // int
					mat.get(row, 0, matRowI);
					break;

				case CvType.CV_16U: // short
				case CvType.CV_16S:
					mat.get(row, 0, matRowS);
					break;
	
				default:
					return "ArrayUtils.brief(Mat) should not be here.";
			}
			printCountCol = 0;
			for(int colC = 0; colC < mat.cols()*mat.channels(); colC++)
			{
				if(colC > 10 && colC < Math.max(colC, mat.cols()*mat.channels()-10))
				{
					if(printCountCol % 2000 == 0)
					{
						printCountCol = 0;
						sb.append(".c. ");
					}
					printCountCol++;
					continue;
				}
				switch(CvType.depth(mat.type()))
				{
					case CvType.CV_64F: // double
						sb.append(matRowD[colC]);
						break;

					case CvType.CV_32F: // float
						sb.append(matRowF[colC]);
						break;
		
					case CvType.CV_8U: // byte
					case CvType.CV_8S:
						sb.append(matRowB[colC]);
						break;
		
					case CvType.CV_32S: // int
						sb.append(matRowI[colC]);
						break;

						case CvType.CV_16U: // short
					case CvType.CV_16S:
						sb.append(matRowS[colC]);
						break;

					default:
						return "ArrayUtils.brief(Mat) should not be here.";
				}
				sb.append(" ");
			}
			sb.append("\n");
		}
		return sb.toString();
	}
}
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
/*                                                                                                            */
/*                                   End ArrayUtils Class                                                                         */
/*                                                                                                            */
/*----------------------------------------------------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------- */
