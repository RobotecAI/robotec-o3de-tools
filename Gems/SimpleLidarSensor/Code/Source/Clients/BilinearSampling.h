#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>

namespace SimpleLidarSensor

{
    enum class BorderMode { Clamp, Zero }; // Clamp -> nearest edge, Zero -> 0 outside

    //! x = column (u), y = row (v).
    //! If normalized == true: x,y are in [0,1] mapped to image width/height.
    //! Expects input.type() == CV_32FC1.
    float sampleBilinear(const cv::Mat& img, float x, float y,
                         bool normalized = false,
                         BorderMode border = BorderMode::Clamp)
    {
        CV_Assert(img.type() == CV_32FC1);

        // map normalized coords to image space
        float fx = x, fy = y;
        if (normalized) {
            fx = x * (img.cols  - 1); // so u=0 -> col 0, u=1 -> col cols-1
            fy = y * (img.rows  - 1);
        }

        // integer top-left
        int x0 = static_cast<int>(std::floor(fx));
        int y0 = static_cast<int>(std::floor(fy));
        int x1 = x0 + 1;
        int y1 = y0 + 1;

        // fractional parts
        float tx = fx - float(x0);
        float ty = fy - float(y0);

        auto getPixel = [&](int xi, int yi) -> float {
            // outside check
            if (xi < 0 || xi >= img.cols || yi < 0 || yi >= img.rows) {
                if (border == BorderMode::Zero) return 0.0f;
                // Clamp to nearest edge:
                xi = std::clamp(xi, 0, img.cols - 1);
                yi = std::clamp(yi, 0, img.rows - 1);
            }
            // Fast access with ptr
            return img.ptr<float>(yi)[xi];
        };

        // fetch four neighbours
        float v00 = getPixel(x0, y0);
        float v10 = getPixel(x1, y0);
        float v01 = getPixel(x0, y1);
        float v11 = getPixel(x1, y1);

        // edge handling: if large difference between neighbours we skip interpolation
        if (std::abs(v00 - v10) > 1.0f || std::abs(v00 - v01) > 1.0f || std::abs(v00 - v11) > 1.0f) {
            return v00; // return nearest neighbour
        }
        // bilinear interpolation
        float a = v00 * (1 - tx) + v10 * tx;
        float b = v01 * (1 - tx) + v11 * tx;
        float value = a * (1 - ty) + b * ty;
        return value;
    }
}