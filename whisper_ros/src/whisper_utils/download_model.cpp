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

std::string download_model(const std::string &repo_id,
                           const std::string &filename) {

  if (repo_id.empty() || filename.empty()) {
    return "";
  }

  auto result = huggingface_hub::hf_hub_download_with_shards(repo_id, filename);

  if (result.success) {
    return result.path;
  } else {
    return "";
  }
}

} // namespace whisper_utils