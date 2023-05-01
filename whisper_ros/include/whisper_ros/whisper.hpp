
#ifndef WHISPER_HPP
#define WHISPER_HPP

#include <string>
#include <vector>

#include "whisper.h"

namespace whisper_ros {

class Whisper {

public:
  Whisper(whisper_full_params wparams, std::string model);
  ~Whisper();

  whisper_full_params wparams;

  std::string transcribe(const std::vector<float> &pcmf32);

protected:
  whisper_context *ctx;

private:
  bool force_speak;
  float prob0;
};

} // namespace whisper_ros

#endif