
#include "whisper_ros/whisper_node.hpp"
#include "common.h"
#include "whisper.h"

using namespace whisper_ros;
using std::placeholders::_1;

WhisperNode::WhisperNode() : rclcpp::Node("whisper_node") {

  std::string model;
  std::string language;
  int32_t capture_id;
  auto wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);

  this->declare_parameters<int32_t>("", {
                                            {"n_threads", 4},
                                            {"n_max_text_ctx", 16384},
                                            {"offset_ms", 0},
                                            {"duration_ms", 0},
                                            {"max_len", 0},
                                            {"max_tokens", 0},
                                            {"audio_ctx", 0},
                                            {"capture_id", -1},
                                            {"voice_ms", 10000},
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
                                          {"vad_thold", 0.60f},
                                          {"freq_thold", 100.0f},
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
                                         {"print_energy", false},
                                     });

  this->get_parameter("model", model);
  this->get_parameter("capture_id", capture_id);
  this->get_parameter("voice_ms", this->voice_ms);
  this->get_parameter("vad_thold", this->vad_thold);
  this->get_parameter("freq_thold", this->freq_thold);
  this->get_parameter("print_energy", this->print_energy);

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

  this->audio = std::make_shared<audio_async>(30 * 1000);
  if (!audio->init(capture_id, WHISPER_SAMPLE_RATE)) {
    RCLCPP_ERROR(this->get_logger(), "Audio->init() failed");
    return;
  }
}

void WhisperNode::work() {

  std::vector<float> pcmf32;
  this->audio->resume();

  while (rclcpp::ok()) {
    this->audio->get(2000, pcmf32);

    if (vad_simple(pcmf32, WHISPER_SAMPLE_RATE, 1250, this->vad_thold,
                   this->freq_thold, this->print_energy)) {

      this->audio->get(this->voice_ms, pcmf32);
      std::string text_heard = ::trim(this->whisper->transcribe(pcmf32));
      RCLCPP_INFO(this->get_logger(), "Text heard: %s", text_heard.c_str());

      std_msgs::msg::String result_msg;
      result_msg.data = text_heard;
      this->publisher_->publish(result_msg);
    }
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WhisperNode>();
  node->work();
  rclcpp::shutdown();
  return 0;
}