#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

#include "helpers_vision/converters.hpp"

TEST(binaryMaskToString, testEqual)
{
    //////////////////////////////////////////////////////////////////////////////////////////
    // Reference data
    // // Old data
    // const cv::Mat mask = cv::imread("/root/ros2_ws/src/avena_ros2/helpers/data/test_orange_mask.png", cv::IMREAD_GRAYSCALE);
    // const std::string mask_rle = R"({"counts":"\\ad68Vf05K4N1N2N2N2N2O001O001O001O000000000001O001O0O2O0O2O0O3M2N2N2Menhd0","size":[720,1280]})";
    //////////////////////////////////////////////////////////////////////////////////////////
    // New data
    const cv::Mat mask = cv::imread("/root/ros2_ws/src/avena_ros2/helpers/data/test_orange_mask.png", cv::IMREAD_GRAYSCALE);
    const std::string mask_rle = R"({"size":[1280,720],"counts":"eYe7=bW13M3M3L4N0O2O1N2O1O000000000001O00001O001N2O1N2N2N3M3L4LW^Zc0"})";
    //////////////////////////////////////////////////////////////////////////////////////////

    std::string encoded_mask_rle;
    const size_t iter = 1000;
    std::vector<float> times;
    for (size_t i = 0; i < iter; ++i)
    {
        cv::Mat temp_mask_current = mask.clone();
        std::string temp_encoded_mask_rle;

        auto s0 = std::chrono::steady_clock::now();
        helpers::converters::binaryMaskToString(temp_mask_current, temp_encoded_mask_rle);
        auto s1 = std::chrono::steady_clock::now();

        auto duration = std::chrono::duration<float, std::milli>(s1 - s0);
        times.push_back(duration.count());
        encoded_mask_rle = temp_encoded_mask_rle;
    }
    const auto [min_time, max_time] = std::minmax_element(times.begin(), times.end());
    auto avg = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    std::cout << "avg: " << avg << " ms" << std::endl;
    std::cout << "min: " << *min_time << " ms" << std::endl;
    std::cout << "max: " << *max_time << " ms" << std::endl;

    nlohmann::json mask_rle_json = nlohmann::json::parse(mask_rle);
    nlohmann::json encoded_mask_rle_json = nlohmann::json::parse(encoded_mask_rle);
    EXPECT_EQ(mask_rle_json.dump(), encoded_mask_rle_json.dump());
    
    // cv::imshow("mask", mask);
    // cv::waitKey();
    // cv::destroyAllWindows();
}

TEST(stringToBinaryMask, testEqual)
{
    //////////////////////////////////////////////////////////////////////////////////////////
    // Reference data
    // // Old data
    // const cv::Mat mask = cv::imread("/root/ros2_ws/src/avena_ros2/helpers/data/test_orange_mask.png", cv::IMREAD_GRAYSCALE);
    // const std::string mask_rle = R"({"counts":"\\ad68Vf05K4N1N2N2N2N2O001O001O001O000000000001O001O0O2O0O2O0O3M2N2N2Menhd0","size":[720,1280]})";
    // New data
    const cv::Mat mask = cv::imread("/root/ros2_ws/src/avena_ros2/helpers/data/test_orange_mask.png", cv::IMREAD_GRAYSCALE);
    const std::string mask_rle = R"({"size":[1280,720],"counts":"eYe7=bW13M3M3L4N0O2O1N2O1O000000000001O00001O001N2O1N2N2N3M3L4LW^Zc0"})";
    //////////////////////////////////////////////////////////////////////////////////////////

    const size_t iter = 1000;
    cv::Mat decoded_mask;
    std::vector<float> times;
    for (size_t i = 0; i < iter; ++i)
    {
        std::string mask_rle_temp = mask_rle;
        cv::Mat decoded_mask_temp;

        auto s0 = std::chrono::steady_clock::now();
        helpers::converters::stringToBinaryMask(mask_rle_temp, decoded_mask_temp, false);
        auto s1 = std::chrono::steady_clock::now();

        auto duration = std::chrono::duration<float, std::milli>(s1 - s0);
        times.push_back(duration.count());
        decoded_mask = decoded_mask_temp.clone();
    }

    const auto [min_time, max_time] = std::minmax_element(times.begin(), times.end());
    auto avg = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    std::cout << "avg: " << avg << " ms" << std::endl;
    std::cout << "min: " << *min_time << " ms" << std::endl;
    std::cout << "max: " << *max_time << " ms" << std::endl;

    cv::Mat diff;
    cv::compare(mask, decoded_mask, diff, cv::CMP_NE);
    int nr_non_zeros = cv::countNonZero(diff);

    EXPECT_EQ(nr_non_zeros, 0);

    // cv::imshow("decoded_mask", decoded_mask);
    // cv::imshow("ref_mask", mask);
    // cv::waitKey();
    // cv::destroyAllWindows();
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    // initialize ROS
    rclcpp::init(argc, argv);

    bool all_successful = RUN_ALL_TESTS();

    // shutdown ROS
    rclcpp::shutdown();

    return all_successful;
}
