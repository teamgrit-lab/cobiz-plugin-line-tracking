#include "line_tracking/segmenter.hpp"

#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numeric>
#include <span>
#include <stdexcept>

namespace line_tracking {
namespace {

constexpr std::array<int, 7> kRoadIds{7, 8, 10, 13, 14, 23, 24};
constexpr std::array<int, 3> kSidewalkIds{9, 11, 15};

bool contains(const std::span<const int> values, const int value) {
  return std::ranges::find(values, value) != values.end();
}

double seconds_between(const std::chrono::steady_clock::time_point start,
                       const std::chrono::steady_clock::time_point end) {
  return std::chrono::duration<double>(end - start).count();
}

std::vector<cv::Mat> resize_scores(const std::vector<cv::Mat> &scores,
                                   const cv::Size size) {
  std::vector<cv::Mat> resized;
  resized.reserve(scores.size());
  for (const auto &score : scores) {
    cv::Mat channel;
    cv::resize(score, channel, size, 0.0, 0.0, cv::INTER_LINEAR);
    resized.push_back(std::move(channel));
  }
  return resized;
}

} // namespace

SurfaceSegmenter::SurfaceSegmenter(SegmenterConfig config)
    : config_(std::move(config)), profile_(resolve_profile(config_.profile)) {
  config_.validate();
  if (!std::filesystem::is_regular_file(config_.model_path)) {
    throw std::invalid_argument("ONNX model does not exist: " +
                                config_.model_path.string());
  }
  temporal_alpha_ = config_.temporal_alpha.value_or(profile_.temporal_alpha);
  hysteresis_margin_ = config_.temporal_hysteresis_margin.value_or(
      profile_.temporal_hysteresis_margin);
  network_ = cv::dnn::readNetFromONNX(config_.model_path.string());
  if (network_.empty()) {
    throw std::runtime_error("OpenCV DNN could not load the ONNX model");
  }
  if (config_.prefer_cuda && cv::cuda::getCudaEnabledDeviceCount() > 0) {
    network_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    network_.setPreferableTarget(profile_.precision == "fp16"
                                     ? cv::dnn::DNN_TARGET_CUDA_FP16
                                     : cv::dnn::DNN_TARGET_CUDA);
    using_cuda_ = true;
  } else {
    network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    network_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  }
}

void SurfaceSegmenter::reset() {
  previous_scores_.clear();
  previous_selected_.release();
}

std::vector<cv::Mat>
SurfaceSegmenter::decode_outputs(const std::vector<cv::Mat> &outputs) const {
  if (outputs.empty()) {
    throw std::runtime_error("model produced no output tensors");
  }
  const cv::Size evaluation_size{config_.evaluation_width,
                                 config_.evaluation_height};

  const cv::Mat *class_logits = nullptr;
  const cv::Mat *mask_logits = nullptr;
  for (const auto &output : outputs) {
    if (output.dims == 3) {
      class_logits = &output;
    }
  }
  if (class_logits != nullptr) {
    for (const auto &output : outputs) {
      if (output.dims == 4 && output.size[0] == 1 &&
          output.size[1] == class_logits->size[1]) {
        mask_logits = &output;
        break;
      }
    }
  }
  if (class_logits != nullptr && mask_logits != nullptr &&
      class_logits->type() == CV_32F && mask_logits->type() == CV_32F) {
    const int queries = class_logits->size[1];
    const int classes_with_null = class_logits->size[2];
    const int classes = classes_with_null - 1;
    if (queries != mask_logits->size[1] || classes <= 0) {
      throw std::runtime_error(
          "Mask2Former ONNX output shapes are inconsistent");
    }
    const int height = mask_logits->size[2];
    const int width = mask_logits->size[3];
    const int pixels = height * width;
    cv::Mat class_probabilities(classes, queries, CV_32F);
    cv::Mat mask_probabilities(queries, pixels, CV_32F);
    for (int query = 0; query < queries; ++query) {
      const float *logits = class_logits->ptr<float>(0, query);
      const float maximum =
          *std::max_element(logits, logits + classes_with_null);
      double denominator = 0.0;
      for (int category = 0; category < classes_with_null; ++category) {
        denominator +=
            std::exp(static_cast<double>(logits[category] - maximum));
      }
      for (int category = 0; category < classes; ++category) {
        class_probabilities.at<float>(category, query) = static_cast<float>(
            std::exp(static_cast<double>(logits[category] - maximum)) /
            denominator);
      }
      const float *masks = mask_logits->ptr<float>(0, query);
      float *probabilities = mask_probabilities.ptr<float>(query);
      for (int pixel = 0; pixel < pixels; ++pixel) {
        probabilities[pixel] = 1.0F / (1.0F + std::exp(-masks[pixel]));
      }
    }
    cv::Mat flat_scores;
    cv::gemm(class_probabilities, mask_probabilities, 1.0, cv::noArray(), 0.0,
             flat_scores);
    std::vector<cv::Mat> scores;
    scores.reserve(static_cast<std::size_t>(classes));
    for (int category = 0; category < classes; ++category) {
      cv::Mat view(height, width, CV_32F, flat_scores.ptr<float>(category));
      cv::Mat resized;
      cv::resize(view, resized, evaluation_size, 0.0, 0.0, cv::INTER_LINEAR);
      scores.push_back(std::move(resized));
    }
    return scores;
  }

  for (const auto &output : outputs) {
    if (output.dims != 4 || output.size[0] != 1 || output.type() != CV_32F) {
      continue;
    }
    const int classes = output.size[1];
    const int height = output.size[2];
    const int width = output.size[3];
    if (classes < 3 || classes > 512) {
      continue;
    }
    std::vector<cv::Mat> scores;
    scores.reserve(static_cast<std::size_t>(classes));
    for (int category = 0; category < classes; ++category) {
      cv::Mat view(height, width, CV_32F,
                   const_cast<float *>(output.ptr<float>(0, category)));
      scores.push_back(view.clone());
    }
    return resize_scores(scores, evaluation_size);
  }
  throw std::runtime_error("unsupported ONNX outputs; expected semantic logits "
                           "or Mask2Former class/mask logits");
}

std::vector<cv::Mat> SurfaceSegmenter::infer_scores(const cv::Mat &frame_bgr) {
  cv::Mat blob = cv::dnn::blobFromImage(
      frame_bgr, 1.0 / 255.0,
      cv::Size{profile_.input_width, profile_.input_height}, cv::Scalar{}, true,
      false, CV_32F);
  constexpr std::array<float, 3> mean{0.485F, 0.456F, 0.406F};
  constexpr std::array<float, 3> standard_deviation{0.229F, 0.224F, 0.225F};
  const int pixels = profile_.input_height * profile_.input_width;
  for (int channel = 0; channel < 3; ++channel) {
    float *values = blob.ptr<float>(0, channel);
    for (int pixel = 0; pixel < pixels; ++pixel) {
      values[pixel] =
          (values[pixel] - mean[static_cast<std::size_t>(channel)]) /
          standard_deviation[static_cast<std::size_t>(channel)];
    }
  }
  network_.setInput(blob);
  std::vector<cv::Mat> outputs;
  network_.forward(outputs, network_.getUnconnectedOutLayersNames());
  return decode_outputs(outputs);
}

SegmentationResult SurfaceSegmenter::segment(const cv::Mat &frame_bgr) {
  if (frame_bgr.empty() || frame_bgr.type() != CV_8UC3) {
    throw std::invalid_argument("frame_bgr must be a non-empty CV_8UC3 image");
  }
  const auto started = std::chrono::steady_clock::now();
  std::vector<cv::Mat> scores = infer_scores(frame_bgr);
  const auto inference_finished = std::chrono::steady_clock::now();
  if (scores.size() <= 24) {
    throw std::runtime_error(
        "model output has too few classes for the Mapillary label contract");
  }
  if (!previous_scores_.empty()) {
    if (previous_scores_.size() != scores.size()) {
      throw std::runtime_error("model class count changed between frames");
    }
    for (std::size_t category = 0; category < scores.size(); ++category) {
      cv::addWeighted(scores[category], temporal_alpha_,
                      previous_scores_[category], 1.0 - temporal_alpha_, 0.0,
                      scores[category]);
    }
  }
  previous_scores_.clear();
  previous_scores_.reserve(scores.size());
  for (const auto &score : scores) {
    previous_scores_.push_back(score.clone());
  }

  cv::Mat selected(config_.evaluation_height, config_.evaluation_width, CV_8U,
                   cv::Scalar{0});
  std::uint64_t held_pixels = 0;
  for (int row = 0; row < selected.rows; ++row) {
    for (int column = 0; column < selected.cols; ++column) {
      int best_index = 0;
      float best = -std::numeric_limits<float>::infinity();
      float second = -std::numeric_limits<float>::infinity();
      for (std::size_t category = 0; category < scores.size(); ++category) {
        const float score = scores[category].at<float>(row, column);
        if (score > best) {
          second = best;
          best = score;
          best_index = static_cast<int>(category);
        } else if (score > second) {
          second = score;
        }
      }
      std::uint8_t value = 0;
      if (contains(kRoadIds, best_index)) {
        value = 1;
      } else if (contains(kSidewalkIds, best_index)) {
        value = 2;
      }
      if (!previous_selected_.empty()) {
        const auto previous = previous_selected_.at<std::uint8_t>(row, column);
        if (value != previous && best - second < hysteresis_margin_) {
          value = previous;
          ++held_pixels;
        }
      }
      selected.at<std::uint8_t>(row, column) = value;
    }
  }

  const int minimum_area =
      std::max(48, static_cast<int>(selected.total() * 0.00035));
  cv::Mat road;
  cv::Mat sidewalk;
  cv::compare(selected, 1, road, cv::CMP_EQ);
  cv::compare(selected, 2, sidewalk, cv::CMP_EQ);
  road = remove_small_components(road, minimum_area);
  sidewalk = remove_small_components(sidewalk, minimum_area);
  selected.setTo(0);
  selected.setTo(1, road);
  selected.setTo(2, sidewalk);
  previous_selected_ = selected.clone();

  const auto finished = std::chrono::steady_clock::now();
  const double pixels = static_cast<double>(selected.total());
  return {std::move(selected),
          seconds_between(started, inference_finished),
          seconds_between(inference_finished, finished),
          seconds_between(started, finished),
          static_cast<double>(held_pixels) / pixels,
          cv::countNonZero(road) / pixels,
          cv::countNonZero(sidewalk) / pixels};
}

} // namespace line_tracking
