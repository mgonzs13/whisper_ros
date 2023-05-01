
#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"

#include "whisper.h"
#include "whisper_ros/whisper_node.hpp"

using namespace whisper_ros;
using std::placeholders::_1;

WhisperNode::WhisperNode() : rclcpp::Node("whisper_node") {

  std::string model;
  std::string language;
  auto wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);

  this->declare_parameters<int32_t>("", {
                                            {"n_threads", 4},
                                            {"n_max_text_ctx", 16384},
                                            {"offset_ms", 0},
                                            {"duration_ms", 0},
                                            {"max_len", 0},
                                            {"max_tokens", 0},
                                            {"audio_ctx", 0},
                                        });
  this->declare_parameters<std::string>("", {
                                                {"model", ""},
                                                {"language", "en"},
                                            });
  this->declare_parameters<float>("", {
                                          {"thold_pt", 0.01f},
                                          {"thold_ptsum", 0.01f},
                                          {"temperature", 0.00f},
                                          {"max_initial_ts", 1.00f},
                                          {"length_penalty", -1.00f},
                                          {"temperature_inc", 0.40f},
                                          {"entropy_thold", 2.40f},
                                          {"logprob_thold", -1.00f},
                                          {"no_speech_thold", 0.60f},
                                      });
  this->declare_parameters<bool>("", {
                                         {"translate", false},
                                         {"no_context", true},
                                         {"single_segment", true},
                                         {"print_special", false},
                                         {"print_progress", false},
                                         {"print_realtime", false},
                                         {"print_timestamps", false},
                                         {"token_timestamps", false},
                                         {"split_on_word", false},
                                         {"speed_up", false},
                                         {"suppress_blank", true},
                                         {"suppress_non_speech_tokens", false},
                                         {"diarize", false},
                                     });

  this->get_parameter("model", model);

  this->get_parameter("n_threads", wparams.n_threads);
  this->get_parameter("n_max_text_ctx", wparams.n_max_text_ctx);
  this->get_parameter("offset_ms", wparams.offset_ms);
  this->get_parameter("duration_ms", wparams.duration_ms);

  this->get_parameter("translate", wparams.translate);
  this->get_parameter("no_context", wparams.no_context);
  this->get_parameter("single_segment", wparams.single_segment);
  this->get_parameter("print_special", wparams.print_special);
  this->get_parameter("print_progress", wparams.print_progress);
  this->get_parameter("print_realtime", wparams.print_realtime);
  this->get_parameter("print_timestamps", wparams.print_timestamps);

  this->get_parameter("token_timestamps", wparams.token_timestamps);
  this->get_parameter("thold_pt", wparams.thold_pt);
  this->get_parameter("thold_ptsum", wparams.thold_ptsum);
  this->get_parameter("max_len", wparams.max_len);
  this->get_parameter("split_on_word", wparams.split_on_word);
  this->get_parameter("max_tokens", wparams.max_tokens);

  this->get_parameter("speed_up", wparams.speed_up);
  this->get_parameter("audio_ctx", wparams.audio_ctx);

  this->get_parameter("language", language);
  wparams.language = language.c_str();

  this->get_parameter("suppress_blank", wparams.suppress_blank);
  this->get_parameter("suppress_non_speech_tokens",
                      wparams.suppress_non_speech_tokens);

  this->whisper = std::make_shared<Whisper>(wparams, model);
  this->publisher_ = this->create_publisher<std_msgs::msg::String>("stt", 10);
  this->subscription_ =
      this->create_subscription<audio_common_msgs::msg::AudioData>(
          "/audio/audio", 10,
          std::bind(&WhisperNode::audio_callback, this, _1));
}

void WhisperNode::audio_callback(
    const audio_common_msgs::msg::AudioData::SharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "AudioData (%d data bytes)",
              int(msg->data.size()));

  std::vector<float> pcmf32;
  if (!this->msg_to_wav(msg->data, &pcmf32)) {
    return;
  }

  std::string result = this->whisper->transcribe(pcmf32);
  std_msgs::msg::String result_msg;
  result_msg.data = result;
  this->publisher_->publish(result_msg);
}

bool WhisperNode::msg_to_wav(std::vector<uint8_t> data,
                             std::vector<float> *pcmf32) {

  drwav wav;

  if (!drwav_init_memory(&wav, data.data(), data.size(), NULL)) {
    RCLCPP_ERROR(this->get_logger(), "dr_wav failed to init wav");
    return false;
  }

  if (wav.channels != 1 && wav.channels != 2) {
    RCLCPP_ERROR(this->get_logger(), "WAV must be mono or stereo\n");
    return false;
  }

  if (wav.sampleRate != WHISPER_SAMPLE_RATE) {
    RCLCPP_ERROR(this->get_logger(), "WAV must be 16 kHz\n");
    return false;
  }

  if (wav.bitsPerSample != 16) {
    RCLCPP_ERROR(this->get_logger(), "WAV must be 16-bit\n");
    return false;
  }

  const uint64_t n = wav.totalPCMFrameCount;

  std::vector<int16_t> pcm16;
  pcm16.resize(n * wav.channels);
  drwav_read_pcm_frames_s16(&wav, n, pcm16.data());

  // convert to mono, float
  pcmf32->resize(n);
  if (wav.channels == 1) {
    for (uint64_t i = 0; i < n; i++)
      (*pcmf32)[i] = float(pcm16[i]) / 32768.0f;
  } else {
    for (uint64_t i = 0; i < n; i++)
      (*pcmf32)[i] = float(pcm16[2 * i] + pcm16[2 * i + 1]) / 65536.0f;
  }

  RCLCPP_INFO(this->get_logger(),
              "Processing %d samples, %.1f sec, %d threads, "
              "lang = %s, task = %s, timestamps = %d",
              int(pcmf32->size()), float(pcmf32->size()) / WHISPER_SAMPLE_RATE,
              this->whisper->wparams.n_threads, this->whisper->wparams.language,
              this->whisper->wparams.translate ? "translate" : "transcribe",
              this->whisper->wparams.print_timestamps ? 0 : 1);

  drwav_uninit(&wav);
  return true;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WhisperNode>());
  rclcpp::shutdown();
  return 0;
}