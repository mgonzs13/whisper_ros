// Copyright (C) 2025 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "whisper_utils/model_download.hpp"

namespace whisper_utils {

std::string hf_hub_download(const std::string &repo_id,
                            const std::string &filename) {

  auto result = huggingface_hub::hf_hub_download(repo_id, filename);

  if (!result.success) {
    WHISPER_LOG_ERROR("Failed to download file '%s' from repo '%s'",
                      filename.c_str(), repo_id.c_str());
    return "";
  }

  return result.path;
}

std::string download_model(const std::string &repo, const std::string &file) {
  std::regex pattern(R"(-([0-9]+)-of-([0-9]+)\.gguf)");
  std::smatch match;

  if (std::regex_search(file, match, pattern)) {
    int total_shards = std::stoi(match[2]);
    std::string base_name = file.substr(0, match.position(0));

    // Download shards
    for (int i = 1; i <= total_shards; ++i) {
      char shard_file[256];
      snprintf(shard_file, sizeof(shard_file), "%s-%05d-of-%05d.gguf",
               base_name.c_str(), i, total_shards);
      hf_hub_download(repo, shard_file);
    }

    // Return first shard
    char first_shard[256];
    snprintf(first_shard, sizeof(first_shard), "%s-00001-of-%05d.gguf",
             base_name.c_str(), total_shards);
    return hf_hub_download(repo, first_shard);
  }

  return hf_hub_download(repo, file);
}

} // namespace whisper_utils