# Patches applied to the whisper.cpp sources on the Arduino VENTUNO Q build.
#
# Two upstream issues prevent whisper.cpp from using the accelerators:
#
# 1. Hexagon: whisper.cpp builds its HTP skels as `libggml-htp-vNN.so`, the
#    same name llama.cpp uses. llama_ros and whisper_ros are deployed into the
#    same install tree and both put install/lib on ADSP_LIBRARY_PATH, so the
#    FastRPC loader would pick either skel. They are built against different
#    ggml versions, so the mismatch fails with AEE_EUNABLETOLOAD /
#    AEE_ENOSUCHMAP. Rename whisper's skels to `libwhisper-htp-vNN.so` and make
#    the backend request that name, so both stacks can coexist.
#
# 2. OpenCL: whisper.cpp allocates the model tensors before it initializes the
#    compute backends. ggml-opencl initializes its context lazily in
#    ggml_cl_init(), and the model loader asks for the buffer alignment/size
#    before allocating the buffer, so the buffer type's backend_ctx is still
#    null when get_alignment() is called and the node segfaults. Initialize the
#    context from the buffer type accessors.
#
# 3. Hexagon: with flash attention the KV cache is filled with
#    ggml_cpy(Kcur, ggml_view_1d(kv, n_state*n_ctx, offset)). Kcur has shape
#    [n_state, n_ctx] and is F32, the KV cache is F16, so the copy is a
#    type-converting reshape. The HTP backend implements F32->F16 only when
#    source and destination have the same extents, and the scheduler aborts
#    with "pre-allocated tensor ... in a buffer (HTP0) that cannot run the
#    operation (CPY)". Use ggml_view_2d() with the source extents instead:
#    the memory layout is identical, the copy becomes a plain conversion.
#
# Usage: cmake -DWHISPER_SOURCE_DIR=<whisper.cpp source tree> -P this_file
#
# The replacements are idempotent, so the script can run on every configure.

if(NOT DEFINED WHISPER_SOURCE_DIR)
  message(FATAL_ERROR "WHISPER_SOURCE_DIR must be set")
endif()

function(ventuno_replace file old new)
  if(NOT EXISTS "${file}")
    message(FATAL_ERROR "ventuno patch: ${file} does not exist")
  endif()

  file(READ "${file}" contents)

  string(FIND "${contents}" "${new}" already_patched)
  if(NOT already_patched EQUAL -1)
    return()
  endif()

  string(FIND "${contents}" "${old}" position)
  if(position EQUAL -1)
    message(FATAL_ERROR
      "ventuno patch: pattern not found in ${file}:\n${old}\n"
      "whisper.cpp was probably updated; review the patch.")
  endif()

  string(REPLACE "${old}" "${new}" contents "${contents}")
  file(WRITE "${file}" "${contents}")
  message(STATUS "ventuno: patched ${file}")
endfunction()

# --- Hexagon: unique HTP skel names -----------------------------------------
set(hexagon_cmake
  "${WHISPER_SOURCE_DIR}/ggml/src/ggml-hexagon/CMakeLists.txt")
set(hexagon_htp_cmake
  "${WHISPER_SOURCE_DIR}/ggml/src/ggml-hexagon/htp/CMakeLists.txt")
set(hexagon_cpp
  "${WHISPER_SOURCE_DIR}/ggml/src/ggml-hexagon/ggml-hexagon.cpp")

ventuno_replace("${hexagon_htp_cmake}"
  "set(HTP_LIB ggml-htp-\${DSP_VERSION})"
  "set(HTP_LIB whisper-htp-\${DSP_VERSION})")

ventuno_replace("${hexagon_cmake}"
  "list(APPEND HTP_SKELS \${CMAKE_CURRENT_BINARY_DIR}/libggml-htp-\${V}.so)"
  "list(APPEND HTP_SKELS \${CMAKE_CURRENT_BINARY_DIR}/libwhisper-htp-\${V}.so)")

ventuno_replace("${hexagon_cpp}"
  "file:///libggml-htp-v%u.so?htp_iface_skel_handle_invoke&_modver=1.0"
  "file:///libwhisper-htp-v%u.so?htp_iface_skel_handle_invoke&_modver=1.0")

# --- OpenCL: create the backend context before reading it -------------------
set(opencl_cpp
  "${WHISPER_SOURCE_DIR}/ggml/src/ggml-opencl/ggml-opencl.cpp")

ventuno_replace("${opencl_cpp}"
  "static size_t ggml_backend_opencl_buffer_type_get_alignment(ggml_backend_buffer_type_t buffer_type) {
    ggml_backend_opencl_device_context * dev_ctx = (ggml_backend_opencl_device_context *) buffer_type->device->context;"
  "static size_t ggml_backend_opencl_buffer_type_get_alignment(ggml_backend_buffer_type_t buffer_type) {
    // whisper.cpp allocates the model before initializing the backends
    ggml_cl_init(buffer_type->device);
    ggml_backend_opencl_device_context * dev_ctx = (ggml_backend_opencl_device_context *) buffer_type->device->context;")

ventuno_replace("${opencl_cpp}"
  "    if (max_size == (size_t)-1) {
        ggml_backend_opencl_device_context * dev_ctx = (ggml_backend_opencl_device_context *) buffer_type->device->context;"
  "    if (max_size == (size_t)-1) {
        ggml_cl_init(buffer_type->device);
        ggml_backend_opencl_device_context * dev_ctx = (ggml_backend_opencl_device_context *) buffer_type->device->context;")

ventuno_replace("${opencl_cpp}"
  "    if (ggml_is_quantized(tensor->type)) {
        ggml_backend_opencl_device_context * dev_ctx = (ggml_backend_opencl_device_context *) buft->device->context;"
  "    if (ggml_is_quantized(tensor->type)) {
        ggml_cl_init(buft->device);
        ggml_backend_opencl_device_context * dev_ctx = (ggml_backend_opencl_device_context *) buft->device->context;")

# --- Hexagon: make the flash-attention KV cache copies type-convertible -----
set(whisper_cpp "${WHISPER_SOURCE_DIR}/src/whisper.cpp")

# encoder, self-attention scratch cache (kv_pad)
ventuno_replace("${whisper_cpp}"
  "                ggml_build_forward_expand(gf, ggml_cpy(ctx0, Kcur, ggml_view_1d(ctx0, kv_pad.k, n_ctx*n_state, 0)));
                ggml_build_forward_expand(gf, ggml_cpy(ctx0, Vcur, ggml_view_1d(ctx0, kv_pad.v, n_ctx*n_state, 0)));"
  "                ggml_build_forward_expand(gf, ggml_cpy(ctx0, Kcur, ggml_view_2d(ctx0, kv_pad.k, n_state, n_ctx,
                            ggml_element_size(kv_pad.k)*n_state, 0)));
                ggml_build_forward_expand(gf, ggml_cpy(ctx0, Vcur, ggml_view_2d(ctx0, kv_pad.v, n_state, n_ctx,
                            ggml_element_size(kv_pad.v)*n_state, 0)));")

# encoder, cross-attention cache
ventuno_replace("${whisper_cpp}"
  "            k = ggml_view_1d(ctx0, wstate.kv_cross.k, n_state*n_ctx,
                    (ggml_element_size(wstate.kv_cross.k)*n_state)*(il*n_ctx_pad));

            v = ggml_view_1d(ctx0, wstate.kv_cross.v, n_state*n_ctx,
                    (ggml_element_size(wstate.kv_cross.v)*n_state)*(il*n_ctx_pad));"
  "            k = ggml_view_2d(ctx0, wstate.kv_cross.k, n_state, n_ctx,
                    ggml_element_size(wstate.kv_cross.k)*n_state,
                    (ggml_element_size(wstate.kv_cross.k)*n_state)*(il*n_ctx_pad));

            v = ggml_view_2d(ctx0, wstate.kv_cross.v, n_state, n_ctx,
                    ggml_element_size(wstate.kv_cross.v)*n_state,
                    (ggml_element_size(wstate.kv_cross.v)*n_state)*(il*n_ctx_pad));")

# decoder, self-attention cache (flash attention branch only)
ventuno_replace("${whisper_cpp}"
  "                    k = ggml_view_1d(ctx0, kv_self.k, n_tokens*n_state,
                            (ggml_element_size(kv_self.k)*n_state)*(il*n_ctx + kv_head));

                    v = ggml_view_1d(ctx0, kv_self.v, n_tokens*n_state,
                            (ggml_element_size(kv_self.v)*n_state)*(il*n_ctx + kv_head));"
  "                    k = ggml_view_2d(ctx0, kv_self.k, n_state, n_tokens,
                            ggml_element_size(kv_self.k)*n_state,
                            (ggml_element_size(kv_self.k)*n_state)*(il*n_ctx + kv_head));

                    v = ggml_view_2d(ctx0, kv_self.v, n_state, n_tokens,
                            ggml_element_size(kv_self.v)*n_state,
                            (ggml_element_size(kv_self.v)*n_state)*(il*n_ctx + kv_head));")
