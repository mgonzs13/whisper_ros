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

#ifndef WHISPER_UTILS__MODEL_DOWNLOAD_HPP
#define WHISPER_UTILS__MODEL_DOWNLOAD_HPP

#include "huggingface_hub.h"
#include "whisper_utils/logs.hpp"

namespace whisper_utils {

/**
 * @brief Download a model and its shards from HF Hub.
 *
 * This function takes a repo and a file to download and return the path of the
 * downloaded file.
 *
 * @param repo_id The repo name from HF.
 * @param filename The filename from the repo.
 * @return The path of the donwloaded file.
 */
std::string download_model(const std::string &repo_id,
                           const std::string &filename);

} // namespace whisper_utils

#endif // WHISPER_UTILS__MODEL_DOWNLOAD_HPP