#include "Frame.hpp"
#include "Matcher.hpp"
#include "Object.hpp"
#include "thread_pool.hpp"
#include "pyp/timer/timer.hpp"
#include <memory>
#include <opencv2/core/types.hpp>
using namespace std;
namespace MCVSLAM {

    ThreadPool pool(3);
    ORB Frame::extractor_left = ORB("/home/wen/SLAM/MCVSLAM/config/extractor.yaml");
    ORB Frame::extractor_right = ORB("/home/wen/SLAM/MCVSLAM/config/extractor.yaml");
    ORB Frame::extractor_wide= ORB("/home/wen/SLAM/MCVSLAM/config/extractor.yaml");
    
    int extractORB(std::shared_ptr<Object>& obj, ORB& extractor)
	{
        return extractor.Extract(obj->img, obj->kps,obj->desps);
	}

    Frame::Frame(cv::Mat imgleft, cv::Mat imgright, cv::Mat imgwide, double time_stamp, BaseCamera* cam_left, BaseCamera* cam_right, BaseCamera* cam_wide, float _b, float _bf)
	: b(_b), bf(_bf) 
    {
        LEFT = std::make_shared<Object>(cam_left, imgleft);
        RIGHT = std::make_shared<Object>(cam_left, imgright);
        WIDE = std::make_shared<Object>(cam_left, imgwide);

        {
            MyTimer::Timer _("ORB_EXTRACT");
            auto res_left = pool.enqueue(extractORB, LEFT, extractor_left);
            auto res_right = pool.enqueue(extractORB, RIGHT, extractor_right);
            auto res_wide = pool.enqueue(extractORB, WIDE,extractor_wide);
            res_left.get();
            res_right.get();
            res_wide.get();
        }
    }

    Frame::~Frame() {}
    
    void Frame::ComputeStereoMatch(ObjectRef left, ObjectRef right) 
    {
		u_right = vector<float>(left->size(), -1.0f);
		
		const int nRows = left->bounddingbox.height;
		// Assign keypoints to row table
		vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());
		for (int i = 0; i < nRows; i++)
			vRowIndices[i].reserve(200);

		const int Nr = right->size();

		for (int iR = 0; iR < Nr; iR++)
		{
			const cv::KeyPoint &kp =right->kps[iR];
			const float &kpY = kp.pt.y;
			const float r = 2.0f * extractor_left.mvScaleFactor[right->kps[iR].octave];
			const int maxr = ceil(kpY + r);
			const int minr = floor(kpY - r);

			for (int yi = minr; yi <= maxr; yi++)
				vRowIndices[yi].push_back(iR);
				
		}

		// Set limits for search
		const float minZ = b;
		const float minD = 1;

		float maxD = min(bf / minZ, float(1000.0));

		// For each left keypoint search a match in the right image
		vector<pair<int, int>> vDistIdx;
		vDistIdx.reserve(left->N);

		for (int iL = 0; iL < left->N; iL++)
		{
			const cv::KeyPoint &kpL = left->kps[iL];
			const int &levelL = kpL.octave;
			const float &vL = kpL.pt.y;
			const float &uL = kpL.pt.x;

			const vector<size_t> &vCandidates = vRowIndices[vL];

			if (vCandidates.size() == 0)
				continue;

			const float minU = uL - maxD;
			const float maxU = uL - minD;

			if (maxU < 0)
				continue;

			// int bestDist = Matcher::TH_HIGH;
			size_t bestIdxR = 0;

			const cv::Mat &dL = left->desps.row(iL);
			std::vector<cv::Mat> left_desps = {dL};
			std::vector<cv::Mat> right_desps;
			// Compare descriptor to right keypoints
			for (size_t iC = 0; iC < vCandidates.size(); iC++)
			{

				const size_t iR = vCandidates[iC];
				const cv::KeyPoint &kpR =right->kps[iR];

				if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
					continue;

				
				const float &uR = kpR.pt.x;

				if (uR >= minU && uR <= maxU)
				{
					const cv::Mat &dR =right->desps.row(iR);
					right_desps.push_back(dR);
				}
			}
			MatchRes res = Matcher::GetInstance(MATCH_DISTANCE::HAMMING).Match(left_desps, right_desps).FilterThreshold(64);
			// Subpixel match by correlation
			for (cv::DMatch& m : res){
				// coordinates in image pyramid at keypoint scale
				const uint bestIdxR = m.trainIdx;
				const float uR0 =right->kps[bestIdxR].pt.x;
				const float scaleFactor = extractor_left.mvInvScaleFactor[kpL.octave];
				const float scaleduL = round(kpL.pt.x * scaleFactor);
				const float scaledvL = round(kpL.pt.y * scaleFactor);
				const float scaleduR0 = round(uR0 * scaleFactor);

				// sliding window search
				const int w = 5;
				if (scaledvL - w < 0 || scaledvL + w + 1 >= extractor_left.mvImagePyramid[kpL.octave].rows || scaleduL - w < 0 || scaleduL + w + 1 >= extractor_left.mvImagePyramid[kpL.octave].cols)
					continue;
				cv::Mat IL = extractor_left.mvImagePyramid[kpL.octave]
				                 .rowRange(scaledvL - w, scaledvL + w + 1)
				                 .colRange(scaleduL - w, scaleduL + w + 1);
				// IL.convertTo(IL,CV_32F);
				// IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);
				IL.convertTo(IL, CV_16S);
				IL = IL - IL.at<short>(w, w);

				int bestDist = INT_MAX;
				int bestincR = 0;
				const int L = 5;
				vector<float> vDists;
				vDists.resize(2 * L + 1);

				const float iniu = scaleduR0 + L - w;
				const float endu = scaleduR0 + L + w + 1;
				if (iniu < 0 || endu >=extractor_right.mvImagePyramid[kpL.octave].cols)
					continue;

				for (int incR = -L; incR <= +L; incR++)
				{
					cv::Mat IR =
					   extractor_right.mvImagePyramid[kpL.octave]
					        .rowRange(scaledvL - w, scaledvL + w + 1)
					        .colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
					// IR.convertTo(IR,CV_32F);
					// IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);
					IR.convertTo(IR, CV_16S);
					IR = IR - IR.at<short>(w, w);

					float dist = cv::norm(IL, IR, cv::NORM_L1);
					if (dist < bestDist)
					{
						bestDist = dist;
						bestincR = incR;
					}

					vDists[L + incR] = dist;
				}

				if (bestincR == -L || bestincR == L)
					continue;

				// Sub-pixel match (Parabola fitting)
				const float dist1 = vDists[L + bestincR - 1];
				const float dist2 = vDists[L + bestincR];
				const float dist3 = vDists[L + bestincR + 1];

				const float deltaR =
				    (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

				if (deltaR < -1 || deltaR > 1)
					continue;

				// Re-scaled coordinate
				float bestuR = extractor_left.mvScaleFactor[kpL.octave] *
				               ((float)scaleduR0 + (float)bestincR + deltaR);

				float disparity = (uL - bestuR);

				if (disparity >= minD && disparity < maxD)
				{
					// if (disparity <= 0)
					// {
					// 	disparity = 0.01;
					// 	bestuR = uL - 0.01;
					// }
					depth_left[iL] = bf / disparity;
					u_right[iL] = bestuR;

					vDistIdx.push_back(pair<int, int>(bestDist, iL));
				}
			}
		}


		// 中值滤波
		sort(vDistIdx.begin(), vDistIdx.end());
		const float median = vDistIdx[vDistIdx.size() * 1.0 / 2].first;
		// 过滤掉了 离群点。
		// 实验如果不加滤波会怎么样？ 滤掉的都是那些点？ 是否有场景中本来应该是好的点被滤波滤掉了。
		const float thDist = 1.5f * 1.4f * median;

		for (int i = vDistIdx.size() - 1; i >= 0; i--)
		{
			if (vDistIdx[i].first < thDist)
				break;
			else
			{
				u_right[vDistIdx[i].second] = -1;
				depth_left[vDistIdx[i].second] = -1;
			}
		}
    }
}  // namespace MCVSLAM
