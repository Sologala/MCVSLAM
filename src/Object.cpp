#include "Object.hpp"
#include <opencv2/core/types.hpp>
using namespace std;

namespace MCVSLAM{
    Object::Object(MCVSLAM::BaseCamera *_cam, cv::Mat _img)
    : mpCam(_cam), img(_img)
    {
        SetPose(cv::Mat::eye(4, 4, CV_32F));
		bounddingbox = cv::Rect(0, 0, img.cols, img.rows);
		grid_width_inv = static_cast<double>(FRAME_GRID_COLS) / img.cols;
		grid_width_inv = static_cast<double>(FRAME_GRID_ROWS) / img.rows;
    }

    void Object::UpdatePoseMatrix()
	{
		mRcw = mTcw.rowRange(0, 3).colRange(0, 3);
		mRwc = mRcw.t();
		mtcw = mTcw.rowRange(0, 3).col(3);
		mOw = -mRwc * mtcw;

		mTwc = cv::Mat::eye(4, 4, mTcw.type());
		mRwc.copyTo(mTwc.rowRange(0, 3).colRange(0, 3));
		mOw.copyTo(mTwc.rowRange(0, 3).col(3));
	}

	void Object::AssignFeaturesToGrid(){
		// Fill matrix with points
		const int nCells = FRAME_GRID_COLS * FRAME_GRID_ROWS;

		int nReserve = 0.5f * N / (nCells);

		for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
			for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
			{
				grid[i][j].reserve(nReserve);
			}

		for (int i = 0; i < N; i++)
		{
			const cv::KeyPoint &kp = kps[i];

			int nGridPosX, nGridPosY;
			if (PosInGrid(kp, nGridPosX, nGridPosY))
			{
				grid[nGridPosX][nGridPosY].push_back(i);
			}
		}

	}

	bool Object::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
	{
		posX = round((kp.pt.x - bounddingbox.tl().x) * grid_width_inv);
		posY = round((kp.pt.y - bounddingbox.tl().y) * grid_height_inv);

		// Keypoint's coordinates are undistorted, which could cause to go out of the
		// image
		if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 ||
		    posY >= FRAME_GRID_ROWS)
			return false;
		return true;
	}

	vector<size_t> Object::GetFeaturesInArea(const float &x, const float &y,
	                                      const float &r, const int minLevel,
	                                      const int maxLevel)
	    const
	{
		vector<size_t> vIndices;
		vIndices.reserve(N);

		float factorX = r;
		float factorY = r;

		const int nMinCellX = max(0, (int)floor((x - bounddingbox.tl().x - factorX) *
		                                        grid_width_inv));
		if (nMinCellX >= FRAME_GRID_COLS)
			return vIndices;
		const int nMaxCellX = min(
		    (int)FRAME_GRID_COLS - 1,
		    (int)ceil((x - bounddingbox.tl().x + factorX) * grid_width_inv));
		if (nMaxCellX < 0)
			return vIndices;

		const int nMinCellY = max(0, (int)floor((y - bounddingbox.tl().y - factorY) *
		                                        grid_height_inv));
		if (nMinCellY >= FRAME_GRID_ROWS)
			return vIndices;

		const int nMaxCellY = min(
		    (int)FRAME_GRID_ROWS - 1,
		    (int)ceil((y - bounddingbox.tl().y + factorY) * grid_height_inv));
		if (nMaxCellY < 0)
			return vIndices;

		const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

		for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
		{
			for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
			{
				const vector<size_t> vCell = grid[ix][iy];
				if (vCell.empty())
					continue;
				// cout << "\t " << ix << " " << iy << endl;
				for (size_t j = 0, jend = vCell.size(); j < jend; j++)
				{
					// cout << vCell[j] << " ";
					const cv::KeyPoint &kp = kps[vCell[j]];
					if (bCheckLevels)
					{
						if (kp.octave < minLevel)
							continue;
						if (maxLevel >= 0)
							if (kp.octave > maxLevel)
								continue;
					}

					const float distx = kp.pt.x - x;
					const float disty = kp.pt.y - y;

					if (fabs(distx) < factorX && fabs(disty) < factorY)
						vIndices.push_back(vCell[j]);
				}
				// cout << endl;
			}
		}
		return vIndices;
	}

}