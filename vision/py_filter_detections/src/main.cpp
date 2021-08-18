#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <nlohmann/json.hpp>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

struct LabeledMasks {
    std::string label;
    std::vector<cv::Mat> masks;
};

int binaryMaskToString(const cv::Mat &mask, std::string &out_string, bool debug) {
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
    while (pos != std::string::npos) {
        val = val == 0 ? MAX_VALUE : 0;
        pos = mask_str.find(val, pos + 1);
        if (pos == std::string::npos) {
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
    for (size_t i = 0; i < cnts.size(); i++) {
        x = static_cast<long>(cnts[i]);
        if (i > 2)
            x -= static_cast<long>(cnts[i - 2]);
        more = 1;
        while (more) {
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
    nlohmann::json mask_json;
    mask_json["counts"] = out_s;
    mask_json["size"][0] = mask.cols;
    mask_json["size"][1] = mask.rows;
    std::shared_ptr<std::stringstream> ss(new std::stringstream);
    out_string = mask_json.dump();
    return 0;
}


bool _checkOverlay(cv::Mat &mask1, cv::Mat &mask2, float overlay_margin) {
    if (mask1.empty() || mask2.empty())
        return false;
    std::vector<int> sizes{cv::countNonZero(mask1), cv::countNonZero(mask2)};

    auto[min, max] = std::minmax_element(sizes.begin(), sizes.end());
    cv::Mat merged;
    cv::bitwise_and(mask1, mask2, merged);
    int size_merged = cv::countNonZero(merged);
    if (size_merged >= (1.0 - overlay_margin) * *min && size_merged <= (1.0 + overlay_margin) * *max)
        return true;
    return false;
}


int handleMultipleDetections(std::vector<LabeledMasks> &_detections_cam) {
    //    helpers::Timer timer("Removing duplicated masks.", true);
    auto merge_overlaping_masks = [](std::vector<LabeledMasks> &labeled_masks) mutable {
        for (auto &detections : labeled_masks) {
            for (auto it = detections.masks.begin(); it != detections.masks.end(); ) {
                if (std::distance(it, detections.masks.end()) <= 0)
                    break;
                auto overlayed_mask_it = std::find_if(it+1, detections.masks.end(),
                                                      [it](cv::Mat &mask) {
                                                          return _checkOverlay(*it, mask,
                                                                               0.1);
                                                      });
                if (overlayed_mask_it == detections.masks.end()){
                    it++;
                    continue;
                }

                cv::bitwise_or(*it, *overlayed_mask_it, *overlayed_mask_it);
                it = detections.masks.erase(it);
            }
        }
    };

    merge_overlaping_masks(_detections_cam);
    return 0;
}


std::map<std::string, std::vector<std::string>> filter_detections(pybind11::dict &detectron_output_dict) {

    std::vector<cv::Mat> masks;
    std::vector<std::string> classes;

    for (auto item : detectron_output_dict) {
        std::string msk_key = std::string(pybind11::str(item.first)); //masks

        if (msk_key == std::string("masks")) {
            auto masks_arr = pybind11::cast<pybind11::list>((item.second).ptr());
            for (pybind11::handle h : masks_arr) {
                auto img = pybind11::cast<pybind11::array_t<uint8_t>>(h);
                pybind11::buffer_info buf = img.request();
//                std::cout<<"shape 0: "<<buf.shape[0]<<" shape 1: "<<buf.shape[1]<<std::endl;
                cv::Mat ig(buf.shape[0], buf.shape[1], CV_8UC1, (unsigned char *) buf.ptr);
                masks.push_back(ig);
            }
        } else if (msk_key == std::string("classes")) {
            auto classes_arr = pybind11::cast<pybind11::list>((item.second).ptr());
            for (pybind11::handle h : classes_arr) {
                auto class_name = std::string(pybind11::cast<pybind11::str>(h));
                classes.push_back(class_name);
            }
        }
    }

    std::vector<std::string> containers_labels{"plate", "bowl", "cuttin_board"};
    std::map<std::string, std::vector<cv::Mat>> labeled_containers_masks_cam;
    std::vector<LabeledMasks> _detections_cam;

    assert(masks.size() == classes.size());
    std::set<std::string> unq_classes;

    for (const auto &cls : classes) {
        unq_classes.insert(cls);
    }

//    for (auto &el: unq_classes) {
//        std::cout << el << std::endl;
//    }

    for (const auto &unq_cls : unq_classes) {
        LabeledMasks lm;
        lm.label = unq_cls;
        for (size_t i = 0; i < classes.size(); i++) {
            if (classes[i] == unq_cls) {
                lm.masks.push_back(masks[i]);
            }
        }
        _detections_cam.push_back(lm);
    }
//    std::cout << "Labeled mask obj count: " << _detections_cam.size() << std::endl;

//    for (size_t j = 0; j < _detections_cam.size(); j++) {
//        std::cout << "class: " << _detections_cam[j].label << " no. of masks: " << _detections_cam[j].masks.size()
//                  << std::endl;
//    }
//
//    int l = 100;
//    for(auto &el: _detections_cam){
//
//        for (auto& ee: el.masks){
//            std::string pth = "img"+std::to_string(l)+".png";
//            cv::imwrite(pth, ee*255);
//            l++;
//        }
//    }
    handleMultipleDetections(_detections_cam);

//    for (size_t j = 0; j < _detections_cam.size(); j++) {
//        std::cout << "class: " << _detections_cam[j].label << " no. of masks: " << _detections_cam[j].masks.size()
//        << std::endl;
//    }

    //************************Containers code***********************************//
    // extract container masks from all detection masks
    auto findContainerMasks = [](const std::vector<LabeledMasks> &detection_cam, const std::string &container_label,
                                 std::vector<cv::Mat> &out_labeled_container_masks) -> bool {
        auto labeled_container_masks = std::find_if(detection_cam.begin(), detection_cam.end(),
                                                    [container_label](const LabeledMasks &labeled_masks) {
                                                        return container_label == labeled_masks.label;
                                                    });
        if (labeled_container_masks != detection_cam.end()) {
            out_labeled_container_masks = labeled_container_masks->masks;
            return true;
        } else
            return false;
    };

    auto removeIntersection = [](cv::Mat &overlayed_container_mask, cv::Mat &label_mask) -> void {
        // exists overlaying
        cv::Mat label_mask_intersection;
        // find intersection b/w container and label mask
        cv::bitwise_and(label_mask, overlayed_container_mask, label_mask_intersection);
        // remove label mask intersection from container label
        overlayed_container_mask = overlayed_container_mask - label_mask_intersection;
    };
    auto filterContainersMasksFromOverlayingMasks = [&removeIntersection](
            std::vector<LabeledMasks> &detections_cam, std::vector<std::string> &containers_labels,
            std::map<std::string, std::vector<cv::Mat>> &out_labeled_containers_masks) -> void {
        // for each label masks
        for (auto &detection_cam : detections_cam) {
            // skip the container
            if (out_labeled_containers_masks.find(detection_cam.label) != out_labeled_containers_masks.end())
                continue;
            // check against all containers
            for (auto &container_label : containers_labels) {
                // RCLCPP_INFO_STREAM(this->get_logger(), "checking " << detection_cam.label << " against " << container_label);
                // one label masks
                for (auto label_mask_it = detection_cam.masks.begin();
                     label_mask_it != detection_cam.masks.end(); label_mask_it++) {
                    // showImage(*label_mask_it, std::to_string(label_mask_it - detection_cam.masks.begin()));
                    auto overlayed_container_mask_it = std::find_if(
                            out_labeled_containers_masks[container_label].begin(),
                            out_labeled_containers_masks[container_label].end(),
                            [label_mask_it](cv::Mat &container_mask) {
                                return _checkOverlay(*label_mask_it, container_mask, 0.1);
                            });
                    if (overlayed_container_mask_it != out_labeled_containers_masks[container_label].end()) {
                        //                        RCLCPP_INFO(this->get_logger(), "found intersection");
                        removeIntersection(*overlayed_container_mask_it, *label_mask_it);
                    }

                }
            }
        };
    };
    auto modifyDetectionMasks = [](std::vector<LabeledMasks> &detections_cam,
                                   std::vector<std::string> &containers_labels,
                                   std::map<std::string, std::vector<cv::Mat>> &labeled_containers_masks) -> void {
        for (auto &container_label : containers_labels) {
            for (auto &detection_cam : detections_cam) {
                // modify the container
                if (detection_cam.label == container_label) {
                    //                        RCLCPP_INFO_STREAM(this->get_logger(), "modity container " << container_label);
                    detection_cam.masks = labeled_containers_masks[container_label];
                } else // ignore all other masks
                    continue;
            }
        }
    };


    for (auto container_label_iter = containers_labels.begin(); container_label_iter != containers_labels.end();) {
        bool container_cam1 = findContainerMasks(_detections_cam, *container_label_iter,
                                                 labeled_containers_masks_cam[*container_label_iter]);

        if (!container_cam1) {
            container_label_iter = containers_labels.erase(container_label_iter);
        } else
            ++container_label_iter;
    }
    if (!labeled_containers_masks_cam.empty()) {
        filterContainersMasksFromOverlayingMasks(_detections_cam, containers_labels,
                                                 labeled_containers_masks_cam);
        modifyDetectionMasks(_detections_cam, containers_labels, labeled_containers_masks_cam);
    }
    std::map<std::string, std::vector<std::string>> res;
//    int k = 0;
//    for(auto &el: _detections_cam){
//
//        for (auto& ee: el.masks){
//            std::string pth = "img"+std::to_string(k)+".png";
//            cv::imwrite(pth, ee*255);
//            k++;
//        }
//    }

    for (auto &el: _detections_cam) {
        std::vector<std::string> str_masks;
        for (auto &msk : el.masks) {
            std::string str_msk;
            msk *= 255;
            binaryMaskToString(msk, str_msk, false);
            std::cout<<str_msk<<std::endl;
            str_masks.push_back(str_msk);
        }


        res[el.label] = str_masks;
    }
    return res;

}

namespace py = pybind11;

PYBIND11_MODULE(py_filter_detections, m) {

    m.def("filter_detections", &filter_detections, py::return_value_policy::move, R"pbdoc(
    Execute filter_detections
    )pbdoc");


#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}