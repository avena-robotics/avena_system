#include "helpers_vision/converters.hpp"

namespace helpers
{
    namespace converters
    {
        std::string getPointTypes(const pcl::PCLPointCloud2 &blob)
        {
            std::stringstream oss;
            for (const auto &field : blob.fields)
            {
                if (field.name == "normal_x")
                {
                    oss << "nx ";
                }
                else if (field.name == "normal_y")
                {
                    oss << "ny ";
                }
                else if (field.name == "normal_z")
                {
                    oss << "nz ";
                }
                else if (field.name == "rgb")
                {
                    oss << "r g b ";
                }
                else if (field.name == "rgba")
                {
                    oss << "r g b a ";
                }
                else
                {
                    oss << field.name << " ";
                }
            }
            std::string out_str = oss.str();
            return out_str;
        }

        int imageToStream(const cv::Mat &image, std::shared_ptr<std::iostream> &out_stream, bool debug)
        {
            helpers::Timer timer("helpers::converters::imageToStream", debug);
            if (image.empty())
                return -1;
            std::vector<uint8_t> serialized_img;
            if (image.type() == CV_32FC1)
            {
                cv::Mat temp_image(image.rows, image.cols, CV_8UC4, image.data);
                cv::imencode(".png", temp_image, serialized_img);
            }
            else
            {
                cv::imencode(".png", image, serialized_img);
            }
            std::shared_ptr<std::stringstream> ss(new std::stringstream);
            ss->write(reinterpret_cast<const char *>(serialized_img.data()), serialized_img.size());
            out_stream = ss;
            return 0;
        }

        int imageToStream(const std::shared_ptr<cv::Mat> &image, std::shared_ptr<std::iostream> &out_stream, bool debug)
        {
            return imageToStream(*image, out_stream, debug);
        }

        int streamToImage(const std::shared_ptr<std::iostream> &stream, cv::Mat &out_image, bool debug)
        {
            helpers::Timer timer("helpers::converters::streamToImage", debug);
            if (stream == nullptr)
                return -1;
            stream->seekg(0, stream->end);
            int length = stream->tellg();
            stream->seekg(0, stream->beg);
            std::vector<uint8_t> deserialized_img(length);
            stream->read(reinterpret_cast<char *>(deserialized_img.data()), length);
            out_image = cv::imdecode(deserialized_img, cv::IMREAD_UNCHANGED);
            if (out_image.type() == CV_8UC4)
            {
                cv::Mat temp_out_image(out_image.rows, out_image.cols, CV_32FC1, out_image.data);
                out_image = temp_out_image.clone();
            }
            stream->clear();
            stream->seekg(0);
            return 0;
        }

        int streamToImage(const std::shared_ptr<std::iostream> &stream, std::shared_ptr<cv::Mat> &out_image, bool debug)
        {
            return streamToImage(stream, *out_image, debug);
        }

        int binaryMaskToStream(const cv::Mat &mask, std::shared_ptr<std::iostream> &out_stream, bool debug)
        {
            helpers::Timer timer("helpers::converters::binaryMaskToStream", debug);
            if (mask.empty())
                return -1;

            std::string out_mask_str;
            int return_code = binaryMaskToString(mask, out_mask_str, false);
            if (return_code)
                return return_code;

            std::shared_ptr<std::stringstream> ss(new std::stringstream);
            (*ss) << out_mask_str;
            out_stream = ss;
            return 0;
        }

        int binaryMaskToStream(const std::shared_ptr<cv::Mat> &mask, std::shared_ptr<std::iostream> &out_stream, bool debug)
        {
            helpers::Timer timer("helpers::converters::binaryMaskToStream", debug);
            if (mask == nullptr)
                return -1;
            return binaryMaskToStream(*mask, out_stream, false);
        }

        int binaryMaskToString(const cv::Mat &mask, std::string &out_string, bool debug)
        {
            helpers::Timer timer("helpers::converters::binaryMaskToString", debug);
            if (mask.empty())
                return -1;

            // Initialize values
            const std::string mask_str(reinterpret_cast<const char *>(mask.data), mask.total() * mask.elemSize());
            char val = 255;
            const char MAX_VALUE = val;
            size_t cnt;
            std::vector<uint> cnts;
            size_t prev_pos = 0;

            // Find first 255 value of mask
            size_t pos = mask_str.find(val); // first occurrence
            cnt = pos - prev_pos;
            cnts.push_back(cnt);
            prev_pos = pos;
            while (pos != std::string::npos)
            {
                val = val == 0 ? MAX_VALUE : 0;
                pos = mask_str.find(val, pos + 1);
                if (pos == std::string::npos)
                {
                    // Other change was not found, so the last number of pixels
                    // with the same value is distance from previous change to the end of string.
                    cnts.push_back(mask_str.size() - prev_pos);
                    break;
                }
                cnt = pos - prev_pos;
                cnts.push_back(cnt);
                prev_pos = pos;
            }

            /* Similar to LEB128 but using 6 bits/char and ascii chars 48-111. */
            uint p = 0;
            long x;
            int more;
            std::string out_s;
            out_s.resize(cnts.size() * 6);
            for (size_t i = 0; i < cnts.size(); i++)
            {
                x = static_cast<long>(cnts[i]);
                if (i > 2)
                    x -= static_cast<long>(cnts[i - 2]);
                more = 1;
                while (more)
                {
                    char c = x & 0x1f;
                    x >>= 5;
                    more = (c & 0x10) ? x != -1 : x != 0;
                    if (more)
                        c |= 0x20;
                    c += 48;
                    out_s[p++] = c;
                }
            }
            out_s[p] = 0;
            out_s.resize(p);

            // Postprocessing
            json mask_json;
            mask_json["counts"] = out_s;
            mask_json["size"][0] = mask.cols;
            mask_json["size"][1] = mask.rows;
            std::shared_ptr<std::stringstream> ss(new std::stringstream);
            out_string = mask_json.dump();
            return 0;
        }

        int binaryMaskToString(const std::shared_ptr<cv::Mat> &mask, std::string &out_string, bool debug)
        {
            if (!mask)
                return -1;
            return binaryMaskToString(*mask, out_string, debug);
        }

        int streamToBinaryMask(const std::shared_ptr<std::iostream> &stream, cv::Mat &out_mask, bool debug)
        {
            helpers::Timer timer("helpers::converters::streamToBinaryMask", debug);
            if (stream == nullptr)
                return -1;
            stream->seekg(0, stream->end);
            int length = stream->tellg();
            stream->seekg(0, stream->beg);
            std::string str(length, '\0');
            stream->read(&str[0], length);

            int return_code = stringToBinaryMask(str, out_mask, false);

            stream->clear();
            stream->seekg(0);
            return return_code;
        }

        int streamToBinaryMask(const std::shared_ptr<std::iostream> &stream, std::shared_ptr<cv::Mat> &out_mask, bool debug)
        {
            helpers::Timer timer("helpers::converters::streamToBinaryMask", debug);
            if (!stream)
                return -1;
            if (!out_mask)
                out_mask = std::make_shared<cv::Mat>();
            return streamToBinaryMask(stream, *out_mask, false);
        }

        int stringToBinaryMask(const std::string &mask_str, cv::Mat &out_mask, bool debug)
        {
            helpers::Timer timer("helpers::converters::stringToBinaryMask", debug);
            if (mask_str.empty())
                return -1;
            nlohmann::json json_handler = nlohmann::json::parse(mask_str);
            const size_t w = json_handler["size"][0];
            const size_t h = json_handler["size"][1];
            const std::string s = json_handler["counts"];

            unsigned long m = 0, p = 0, k;
            long x;
            int more;
            std::vector<uint> cnts(s.size());
            m = 0;
            while (s[p])
            {
                x = 0;
                k = 0;
                more = 1;
                while (more)
                {
                    char c = s[p] - 48;
                    x |= (c & 0x1f) << 5 * k;
                    more = c & 0x20;
                    p++;
                    k++;
                    if (!more && (c & 0x10))
                        x |= -1 << 5 * k;
                }
                if (m > 2)
                    x += (long)cnts[m - 2];
                cnts[m++] = static_cast<uint>(x);
            }
            cnts.resize(m);

            out_mask.create(h, w, CV_8UC1);
            uint8_t mask_val = 0;
            size_t idx = 0;
            for (auto &cnt : cnts)
            {
                std::fill(out_mask.data + idx, out_mask.data + idx + cnt, mask_val);
                mask_val ^= 255;
                idx += cnt;
            }
            return 0;
        }

        int stringToBinaryMask(const std::string &mask_str, std::shared_ptr<cv::Mat> &out_mask, bool debug)
        {
            helpers::Timer timer("helpers::converters::stringToBinaryMask", debug);
            if (mask_str == "")
                return -1;
            if (out_mask == nullptr)
                out_mask = std::make_shared<cv::Mat>();
            return stringToBinaryMask(mask_str, *out_mask, false);
        }

        std::string mat_type2encoding(int mat_type)
        {
            switch (mat_type)
            {
            case CV_8UC1:
                return sensor_msgs::image_encodings::MONO8;
            case CV_8UC3:
                return sensor_msgs::image_encodings::BGR8;
            case CV_32FC1:
                return sensor_msgs::image_encodings::TYPE_32FC1;
            default:
                throw std::runtime_error("Unsupported encoding type");
            }
            return "";
        }

        int rosImageToCV(const sensor_msgs::msg::Image &ros_image, cv::Mat &out_image)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(ros_image, ros_image.encoding);
            }
            catch (cv_bridge::Exception &e)
            {
                // throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
                return -1;
            }
            out_image = cv_ptr->image;
            return 0;
        }

        int cvMatToRos(const cv::Mat &cv_image, sensor_msgs::msg::Image &out_ros_image)
        {
            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
            cv_ptr->image = cv_image;
            out_ros_image = *(cv_ptr->toImageMsg());
            out_ros_image.encoding = mat_type2encoding(cv_image.type());
            return 0;
        }

        int eigenAffineToGeometry(const Eigen::Affine3f &eigen_aff, geometry_msgs::msg::Pose &out_ros_pose)
        {
            Eigen::Vector3f position = eigen_aff.translation();
            Eigen::Quaternionf orientation(eigen_aff.rotation());
            out_ros_pose.position.x = position.x();
            out_ros_pose.position.y = position.y();
            out_ros_pose.position.z = position.z();
            out_ros_pose.orientation.w = orientation.w();
            out_ros_pose.orientation.x = orientation.x();
            out_ros_pose.orientation.y = orientation.y();
            out_ros_pose.orientation.z = orientation.z();
            return 0;
        }

        int eigenAffineToGeometry(const Eigen::Affine3f &eigen_aff, geometry_msgs::msg::Transform &out_ros_pose)
        {
            Eigen::Vector3f position = eigen_aff.translation();
            Eigen::Quaternionf orientation(eigen_aff.rotation());
            out_ros_pose.translation.x = position.x();
            out_ros_pose.translation.y = position.y();
            out_ros_pose.translation.z = position.z();
            out_ros_pose.rotation.w = orientation.w();
            out_ros_pose.rotation.x = orientation.x();
            out_ros_pose.rotation.y = orientation.y();
            out_ros_pose.rotation.z = orientation.z();
            return 0;
        }

        int geometryToEigenAffine(const geometry_msgs::msg::Transform &ros_pose, Eigen::Affine3f &out_eigen_aff)
        {
            geometry_msgs::msg::Pose temp_pose;
            temp_pose.position.x = ros_pose.translation.x;
            temp_pose.position.y = ros_pose.translation.y;
            temp_pose.position.z = ros_pose.translation.z;
            temp_pose.orientation = ros_pose.rotation;
            geometryToEigenAffine(temp_pose, out_eigen_aff);
            return 0;
        }

        int geometryToEigenAffine(const geometry_msgs::msg::Pose &ros_pose, Eigen::Affine3f &out_eigen_aff)
        {
            Eigen::Vector3f position(ros_pose.position.x, ros_pose.position.y, ros_pose.position.z);
            Eigen::Quaternionf orientation(ros_pose.orientation.w, ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z);
            out_eigen_aff = Eigen::Translation3f(position) * orientation;
            return 0;
        }

        int iostreamToString(std::shared_ptr<std::iostream> &stream, std::string &out_string)
        {
            if (!stream)
                return -1;
            std::stringstream ss;
            ss << stream->rdbuf();
            out_string = ss.str();
            return 0;
        }

    } // namespace converters

} // namespace helpers
