/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package Guidance;

import edu.wpi.first.math.util.Units;

import java.util.logging.Logger;

import org.opencv.core.Size;

public class Calibration3dPipelineSettings {
        private static Logger LOGGER;
    static {
      LOGGER = Logger.getLogger("");
      LOGGER.finer("Loading");     
    }
    public int boardHeight = 8;
    public int boardWidth = 8;
    public UICalibrationData.BoardType boardType = UICalibrationData.BoardType.CHESSBOARD;
    public double gridSize = Units.inchesToMeters(1.0);

    public Size resolution = new Size(640, 480);
    public boolean useMrCal = true;

    public Calibration3dPipelineSettings() {
    }
}
