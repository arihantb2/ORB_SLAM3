#include <utils/vo_utils.h>

#include <cassert>
#include <iostream>

using visual_odometry::utils::convert_image;

static void test_grayscale_input_converts_to_bgr()
{
    // Regression test: convert_image() used to pass an already-grayscale
    // (CV_8UC1) image straight through via clone(), but every caller
    // downstream (dispatch_mono_with_context / dispatch_stereo_with_context)
    // unconditionally runs cv::cvtColor(result, ..., COLOR_BGR2GRAY), which
    // throws on single-channel input. Any log containing pre-debayered mono8
    // imagery would abort the whole dispatch path.
    cv::Mat gray(4, 4, CV_8UC1, cv::Scalar(120));

    cv::Mat out;
    convert_image(gray, out);

    assert(out.type() == CV_8UC3);
    assert(out.rows == gray.rows && out.cols == gray.cols);

    // COLOR_GRAY2BGR must replicate the intensity into all three channels.
    const cv::Vec3b px = out.at<cv::Vec3b>(0, 0);
    assert(px[0] == 120 && px[1] == 120 && px[2] == 120);

    // Downstream call that previously crashed must now succeed.
    cv::Mat regray;
    cv::cvtColor(out, regray, cv::COLOR_BGR2GRAY);
    assert(regray.type() == CV_8UC1);
}

static void test_bgr_input_passes_through_unchanged()
{
    cv::Mat bgr(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));

    cv::Mat out;
    convert_image(bgr, out);

    assert(out.type() == CV_8UC3);
    const cv::Vec3b px = out.at<cv::Vec3b>(0, 0);
    assert(px[0] == 10 && px[1] == 20 && px[2] == 30);
}

static void test_bayer16_input_converts_to_8bit_bgr()
{
    cv::Mat bayer(8, 8, CV_16UC1, cv::Scalar(1000));

    cv::Mat out;
    convert_image(bayer, out);

    assert(out.type() == CV_8UC3);
    assert(out.rows == bayer.rows && out.cols == bayer.cols);
}

static void test_16bit_bgr_input_converts_to_8bit_bgr()
{
    cv::Mat bgr16(4, 4, CV_16UC3, cv::Scalar(2560, 5120, 7680));

    cv::Mat out;
    convert_image(bgr16, out);

    assert(out.type() == CV_8UC3);
    const cv::Vec3b px = out.at<cv::Vec3b>(0, 0);
    assert(px[0] == 10 && px[1] == 20 && px[2] == 30);
}

int main()
{
    test_grayscale_input_converts_to_bgr();
    test_bgr_input_passes_through_unchanged();
    test_bayer16_input_converts_to_8bit_bgr();
    test_16bit_bgr_input_converts_to_8bit_bgr();

    std::cout << "All vo_utils tests passed." << std::endl;
    return 0;
}
