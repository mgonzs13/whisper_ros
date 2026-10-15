# Cross-compiling whisper_ros for the Arduino VENTUNO Q (aarch64, Hexagon / OpenCL)

Builds the full `whisper_ros` workspace on an x86_64 Linux host for an aarch64 target, e.g. the
Arduino VENTUNO Q (Qualcomm Dragonwing IQ8 / QCS8275), with whisper.cpp's Hexagon (NPU) and
Adreno (OpenCL) backends enabled.

- `Dockerfile` — Ubuntu 24.04 cross-build image with clang for `aarch64-linux-gnu`,
  `qemu-user-static`, ROS 2 Jazzy host tools and the Hexagon SDK 6.6.0.0 (from
  `ghcr.io/snapdragon-toolchain/arm64-linux:v0.7`).
- `toolchain-aarch64.cmake` — compiles against the board snapshot at `/sysroot` and searches
  `/workspace/install` as an extra prefix, which is why `--merge-install` is mandatory.

## Prerequisites

- x86_64 Linux host with Docker.
- `qemu-user-static` on the host so aarch64 binaries can run during the build:
  `sudo apt install qemu-user-static binfmt-support` (or
  `docker run --privileged --rm tonistiigi/binfmt --install arm64`).
- Board running Ubuntu 24.04 + ROS 2 Jazzy, reachable over SSH as `arduino@ventuno`
  (`ventuno` is the board's hostname or IP).

## 1. Prepare the board and create the sysroot

Install the workspace dependencies on the board so the snapshot contains matching `-dev`
packages. Only the sources are needed there (`rosdep` reads the `package.xml` files); nothing
is built on the board:

```bash
git clone https://github.com/mgonzs13/whisper_ros.git ~/ros2_ws/src/whisper_ros
cd ~/ros2_ws
vcs import src < src/whisper_ros/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
sudo apt install build-essential python3-dev python3-vcstool
# only for the OpenCL backend:
sudo apt install ocl-icd-opencl-dev
# let the host pull the sysroot as root over non-interactive SSH:
echo "$USER ALL=(root) NOPASSWD: /usr/bin/rsync" | sudo tee /etc/sudoers.d/rsync
sudo chmod 440 /etc/sudoers.d/rsync
# optional: the sources were only needed for rosdep
rm -rf ~/ros2_ws/src/whisper_ros ~/ros2_ws/src/audio_common
```

Then snapshot the board to the host:

```bash
mkdir -p ~/sysroot/opt

sudo rsync -aHAX --numeric-ids --info=progress2 --rsync-path="sudo -n rsync" \
  --exclude={"/usr/share/doc/*","/usr/share/man/*","/usr/share/locale/*","/usr/share/icons/*","/usr/share/fonts/*","/usr/lib/firmware/*","/usr/lib/modules/*","/usr/lib/debug/*","/usr/src/*"} \
  arduino@ventuno:/usr arduino@ventuno:/lib ~/sysroot/

sudo rsync -aHAX --numeric-ids --info=progress2 --rsync-path="sudo -n rsync" \
  arduino@ventuno:/opt/ros ~/sysroot/opt/

# /etc/alternatives is not copied, so update-alternatives symlinks (libblas.so.3,
# liblapack.so.3) would dangle and break linking; copy the resolved files instead:
sudo rsync -aLR --numeric-ids --rsync-path="sudo -n rsync" \
  arduino@ventuno:/usr/lib/aarch64-linux-gnu/libblas.so.3 \
  arduino@ventuno:/usr/lib/aarch64-linux-gnu/liblapack.so.3 \
  arduino@ventuno:/usr/lib/aarch64-linux-gnu/libblas.so \
  arduino@ventuno:/usr/lib/aarch64-linux-gnu/liblapack.so \
  ~/sysroot/
```

The sysroot must mirror the board: ROS belongs at `~/sysroot/opt/ros/jazzy`. Re-run all three
rsyncs after installing or removing packages on the board. If another alternatives-managed
file dangles (`find ~/sysroot -xtype l -lname '/etc/alternatives/*'`), dereference it the same
way.

## 2. Build the cross image

From the repo root:

```bash
docker build -f cross/ventuno_q/Dockerfile -t ventuno_cross .
```

## 3. Build whisper_ros

Import the workspace dependencies into the repo root on the host first — the container only
sees the mounted tree, and `audio_common` (which provides `audio_common_msgs`) is built from
source together with the whisper packages:

```bash
vcs import . < dependencies.repos
```

From the repo root:

```bash
docker run --rm -it -v ~/sysroot:/sysroot -v $PWD:/workspace ventuno_cross
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --merge-install --cmake-args \
  -DCMAKE_TOOLCHAIN_FILE=/workspace/cross/ventuno_q/toolchain-aarch64.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DGGML_OPENCL=ON \
  -DGGML_HEXAGON=ON \
  -DHEXAGON_SDK_ROOT=/opt/hexagon/6.6.0.0 \
  -DHEXAGON_TOOLS_ROOT=/opt/hexagon/6.6.0.0/tools/HEXAGON_Tools/19.0.07 \
  -DPREBUILT_LIB_DIR=linux_aarch64
```

Notes:

- Source the container's ROS, not the sysroot copy: colcon records the underlay in
  `install/setup.bash`, and `/opt/ros/jazzy` is the path that also exists on the board. The
  target packages are still resolved from the sysroot by the toolchain.
- `PREBUILT_LIB_DIR` is required by the Hexagon SDK (`hexagon_fun.cmake`); without it the
  configure fails with `string sub-command FIND requires 3 or 4 parameters`.
- Drop `-DGGML_OPENCL=ON` if the board has no Adreno OpenCL ICD (check with `clinfo` on the
  board); drop `-DBUILD_TESTING=OFF` to build and cross-run tests through QEMU.
- The backends land in `install/lib` as `libwhisper_ggml_hexagon.so` and
  `libwhisper_ggml_opencl.so`, plus the HTP skels (`libggml-htp-v73.so`, `v75`, `v79`, `v81`).
- Add `-DGGML_BLAS=ON -DGGML_BLAS_VENDOR=OpenBLAS` to speed up F16/F32 CPU inference; BLAS is
  not used for quantized models (the default `ggml-large-v3-turbo-q5_0.bin` included).
- `dependencies.repos` packages are not fetched by the build; import them on the host as
  above. whisper.cpp v1.9.5, huggingface-hub-cpp and the aarch64 ONNX Runtime tarball are
  downloaded by CMake during the build, so the container needs network access.

To rebuild a single package while iterating:

```bash
colcon build --merge-install --packages-select whisper_cpp_vendor --cmake-args ...
```

## 4. Deploy to the board

From the host:

```bash
rsync -aP install/ arduino@ventuno:~/ros2_ws/install/
```

## 5. Run on the board

Set the environment before launching. `ADSP_LIBRARY_PATH` / `CDSP_LIBRARY_PATH` are how the
FastRPC stub finds the HTP skels:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$HOME/ros2_ws/install/lib:$LD_LIBRARY_PATH
export ADSP_LIBRARY_PATH=$HOME/ros2_ws/install/lib
export CDSP_LIBRARY_PATH=$HOME/ros2_ws/install/lib
ros2 launch whisper_bringup whisper.launch.py
```

Pass `stream:=True` to run the streaming node instead of the action server. The Whisper model
(`ggml-large-v3-turbo-q5_0.bin` by default, see `whisper_bringup/config/whisper.yaml`) is
downloaded from Hugging Face on first run.

Expected log lines:

```
ggml-hex: Hexagon backend (experimental) : allocating new registry : ndev 1
ggml-hex: Hexagon Arch version vXX
whisper_init_with_params_no_state: use gpu    = 1
whisper_backend_init_gpu: device 0: <Adreno device> (type: 2)
whisper_backend_init_gpu: device 1: HTP0 (type: 2)
whisper_backend_init_gpu: using <selected device> backend
```

## Selecting the accelerator

whisper.cpp offloads the whole model to a single accelerator: `gpu.device` (default `0`) is the
index of the GPU-type device to use, as enumerated in the `device N:` startup log lines.
Unsupported operators fall back to the CPU automatically. With both backends built, OpenCL is
registered before Hexagon, so the Adreno GPU is `device 0` and the NPU (`HTP0`) is `device 1`;
confirm with the log. Select the NPU in `whisper_bringup/config/whisper.yaml`:

```yaml
/whisper/whisper_node:
  ros__parameters:
    gpu:
      enabled: true
      flash_attn: true
      device: 1 # Hexagon NPU (HTP0); 0 = Adreno (OpenCL)
```

`gpu.flash_attn` only applies to the accelerator path. There is no layer split in whisper.cpp:
the model is placed on the selected device in one piece, so a single HTP session is enough for
all Whisper models.

## Troubleshooting

- `sudo: a terminal is required to read the password` during the sysroot pull: the `NOPASSWD`
  rule from step 1 is missing on the board.
- Missing package configuration or headers while configuring: the dependency is not in the
  sysroot. Install it on the board (`rosdep install`), refresh the sysroot, and rebuild.
- `Exec format error` while building: QEMU/binfmt is not set up on the host.
- `not found: "/sysroot/.../local_setup.bash"` when sourcing `install/setup.bash` on the
  board: the install tree was built with the sysroot ROS sourced. Rebuild per step 3 and
  redeploy; for an existing tree, source `/opt/ros/jazzy/setup.bash` and
  `~/ros2_ws/install/local_setup.bash` instead.
- `warning: liblapack.so.3, needed by ... not found` or `undefined reference to 'dgemm_'`,
  `'sgesdd_'`, ...: dangling BLAS/LAPACK symlinks in the sysroot; run the dereferencing rsync
  from step 1.
- `Make sure HEXAGON_SDK_ROOT ...` or `string sub-command FIND requires 3 or 4 parameters` at
  `hexagon_fun.cmake`: Hexagon SDK flags are wrong or missing; pass the `-DHEXAGON_*` and
  `-DPREBUILT_LIB_DIR=linux_aarch64` arguments from step 3.
- `ggml-hex: ... failed to open session : ... error 0x80000406` (`AEE_EUNABLETOLOAD`): the
  FastRPC stub cannot find the HTP skel. Export `ADSP_LIBRARY_PATH` (and
  `CDSP_LIBRARY_PATH`) as in step 5 and make sure `libggml-htp-vNN.so` was deployed.
- `ggml-hex: ... buffer mapping failed : ... error 0x00000001` / `fastrpc_mmap failed`: the
  model does not fit in a session's mappable address space (~3 GiB). Use a smaller model or
  quantization — whisper.cpp cannot layer-split across sessions.
- `GGML_ASSERT(M % 4 == 0) failed` while transcribing on OpenCL: ggml-opencl rejects some
  quantized tensor shapes (known upstream issue). Switch `model.filename` to an F16 model
  (e.g. `ggml-medium.bin`) or select the NPU/CPU.
- Only Hexagon architectures v73, v75, v79 and v81 have prebuilt skels. If the runtime reports
  a different architecture, the NPU is not supported by this backend.
