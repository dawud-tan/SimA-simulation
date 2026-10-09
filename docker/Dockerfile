# syntax=docker/dockerfile:1.7
# =============================================================================
#  Chrono 10.0.0 — containerised build for the SIM A simulator
#
#  WHY FEDORA 44 AND NOT UBUNTU:
#  These artifacts are not merely *run* in the container — they are linked into
#  an Unreal plugin running on the host. That makes the container a
#  cross-compilation environment whose ABI must match the host exactly.
#  Host baseline measured on this machine:
#      glibc 2.43   libstdc++ 16.2.1 (GLIBCXX_3.4.36)
#  A Ubuntu base would produce .so files demanding symbols the host lacks.
#
#  STAGES:
#    builder  — full toolchain, clones and compiles Chrono
#    runtime  — slim image for headless physics / offline ANCF studies
#    sdk      — scratch stage, for `docker build --target sdk --output type=local`
# =============================================================================

ARG FEDORA_VERSION=44

# ============================== STAGE: builder ===============================
FROM fedora:${FEDORA_VERSION} AS builder

ARG CHRONO_TAG=10.0.0
# 8 physical cores, no SMT, 16 GiB RAM. Chrono's template-heavy translation
# units peak around 1.5 GiB each, so 6 concurrent jobs is the ceiling before
# the OOM killer becomes a risk. Drop to 4 if you see the build die silently.
ARG BUILD_JOBS=6
# i7-9700 is Coffee Lake: AVX2 and FMA, but NO AVX-512. `skylake` is the exact
# ISA match. See the ALIGNMENT WARNING at the bottom of this file before you
# change this value.
ARG MARCH=skylake

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN --mount=type=cache,target=/var/cache/libdnf5,sharing=locked \
    dnf -y install --setopt=install_weak_deps=False --setopt=keepcache=1 \
        gcc gcc-c++ cmake ninja-build make git ccache patch \
        eigen3-devel \
        openblas-devel lapack-devel \
        python3 \
    && dnf clean packages

# Sanity-check Eigen before spending 40 minutes compiling against it.
#
# Assert what find_package(Eigen3) actually consumes -- the Core header and the
# CMake package config -- rather than pattern-matching a header whose layout
# moves between releases. Eigen 5.0 relocated the version macros out of
# Eigen/src/Core/util/Macros.h into a new Eigen/Version header; Chrono 10's
# cmake/FindEigen3.cmake reads Eigen/Version first and falls back to Macros.h,
# so both layouts configure correctly. We just report which one we found.
RUN set -e; \
    test -f /usr/include/eigen3/Eigen/Core; \
    test -f /usr/share/cmake/eigen3/Eigen3Config.cmake; \
    if [ -f /usr/include/eigen3/Eigen/Version ]; then \
        echo "Eigen layout: 5.x (Eigen/Version)"; \
        grep -E '^#define EIGEN_(WORLD|MAJOR|MINOR|PATCH)_VERSION|^#define EIGEN_VERSION_STRING' \
             /usr/include/eigen3/Eigen/Version; \
    else \
        echo "Eigen layout: 3.x (Macros.h)"; \
        grep -E '^#define EIGEN_(WORLD|MAJOR|MINOR)_VERSION' \
             /usr/include/eigen3/Eigen/src/Core/util/Macros.h; \
    fi

WORKDIR /src
RUN git clone --depth 1 --branch "${CHRONO_TAG}" \
        https://github.com/projectchrono/chrono.git chrono \
    && git -C chrono log -1 --format='Chrono %H (%d)'

# --- Local patches ---------------------------------------------------------
# Carried as PATCH FILES applied here, not as a vendored copy of the edited
# source. The difference matters on a version bump: a patch that no longer
# applies FAILS THE BUILD and says which hunk, whereas a vendored file would
# silently keep an old edit or silently lose it.
#
# 0001 exposes the TMeasy Dahl standstill parameters (sigma0/sigma1) and the
# friction blend range. It is PURELY ADDITIVE -- inline setters and getters
# beside the existing SetFrictionCoefficient -- so a build with the patch and
# no caller behaves identically to an unpatched one. Nothing is upstreamed and
# nothing needs to be: this is our image.
#
# WHY: below m_frblend_begin (1.0 m/s) the tire force is 100% the Dahl bristle
# model, whose damping term is viscous in slip velocity. At the stock
# sigma1 = 5000 N.s/m that is ~500 N per tire at 0.1 m/s of slip, ~2000 N over
# four, against the 2101 N gravity component of a 12% grade. Measured
# consequence: a car in neutral on a 12% slope rolls back 49% short of
# g*sin(theta) and full brake will not hold it -- which makes the hill-start
# rollback rule unmeasurable. Chrono's own comment on the model reads "this
# model is experimental and needs some testing".
COPY patches/ /src/patches/
RUN cd /src/chrono \
    && for p in /src/patches/*.patch; do \
         echo "applying $(basename "$p")"; \
         patch -p1 --forward --fuzz=0 < "$p" || exit 1; \
       done \
    && grep -q SetDahlParameters \
         src/chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h \
    && echo "patches applied and verified"

# --- Configure -------------------------------------------------------------
# Module choices explained:
#   VEHICLE      ON  — Chrono::Vehicle, the whole point of Milestone 4.
#   FEA          n/a — FEA lives in the CORE library (src/chrono/fea) in
#                      Chrono 10. There is no CH_ENABLE_MODULE_FEA flag; the
#                      ANCF tire classes come for free with the core build.
#   POSTPROCESS  ON  — offline output for the ANCF validation rig.
#   IRRLICHT/VSG OFF — Unreal is the renderer. Linking a second windowing
#                      toolkit into a headless container is pure liability.
#   SENSOR       OFF — needs CUDA/OptiX; there is no NVIDIA GPU on this host.
#
# CH_USE_EIGEN_OPENMP=OFF is a REAL-TIME decision, not a performance one.
# Eigen spawning an OpenMP team inside a 1 kHz control loop introduces
# scheduling jitter far worse than the arithmetic it saves. This sets
# EIGEN_DONT_PARALLELIZE. Chrono's own OpenMP (coarse-grained, per-system)
# stays ON.
RUN --mount=type=cache,target=/ccache,sharing=locked \
    export CCACHE_DIR=/ccache && \
    mkdir -p /build && \
    cmake -S /src/chrono -B /build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/chrono \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_DEMOS=ON \
        -DBUILD_TESTING=OFF \
        -DBUILD_BENCHMARKING=OFF \
        -DUSE_CCACHE=ON \
        -DCH_ENABLE_MODULE_VEHICLE=ON \
        -DCH_ENABLE_MODULE_POSTPROCESS=ON \
        -DCH_ENABLE_MODULE_IRRLICHT=OFF \
        -DCH_ENABLE_MODULE_VSG=OFF \
        -DCH_ENABLE_MODULE_PYTHON=OFF \
        -DCH_ENABLE_MODULE_SENSOR=OFF \
        -DCH_ENABLE_MODULE_MULTICORE=OFF \
        -DCH_ENABLE_OPENMP=ON \
        -DCH_USE_EIGEN_OPENMP=OFF \
        -DCH_USE_SIMD=ON \
        -DCMAKE_CXX_FLAGS="-march=${MARCH} -mtune=${MARCH}" \
        -DCMAKE_C_FLAGS="-march=${MARCH} -mtune=${MARCH}" \
    2>&1 | tee /build/configure.log

# Fail loudly if the configure step silently disabled Vehicle. Read CMakeCache
# rather than grepping human-readable log text, which is not a stable contract.
RUN grep -q '^CH_ENABLE_MODULE_VEHICLE:BOOL=ON' /build/CMakeCache.txt \
    || { echo "FATAL: Vehicle module not configured"; \
         grep -i 'eigen\|vehicle' /build/configure.log | tail -20; exit 1; }

# --- Compile ---------------------------------------------------------------
RUN --mount=type=cache,target=/ccache,sharing=locked \
    export CCACHE_DIR=/ccache && \
    ccache -z >/dev/null 2>&1 || true; \
    cmake --build /build --parallel "${BUILD_JOBS}" && \
    cmake --install /build && \
    (ccache -s 2>/dev/null || true)

# Record provenance so the host side can detect a stale SDK.
RUN mkdir -p /opt/chrono/share && { \
      echo "chrono_tag=${CHRONO_TAG}"; \
      echo "march=${MARCH}"; \
      echo "built=$(date -u +%Y-%m-%dT%H:%M:%SZ)"; \
      echo "gcc=$(gcc -dumpfullversion)"; \
      echo "glibc=$(ldd --version | head -1 | grep -oE '[0-9]+\.[0-9]+$')"; \
      echo "eigen_openmp=OFF"; \
    } > /opt/chrono/share/BUILDINFO

# ============================== STAGE: runtime ===============================
FROM fedora:${FEDORA_VERSION} AS runtime

RUN --mount=type=cache,target=/var/cache/libdnf5,sharing=locked \
    dnf -y install --setopt=install_weak_deps=False \
        libstdc++ libgomp openblas lapack \
    && dnf clean all

COPY --from=builder /opt/chrono /opt/chrono
COPY --from=builder /build/bin   /opt/chrono/bin-demos

ENV LD_LIBRARY_PATH=/opt/chrono/lib64:/opt/chrono/lib \
    CHRONO_DATA_DIR=/opt/chrono/share/chrono/data/

RUN useradd -m -u 1000 chrono
USER chrono
WORKDIR /work
CMD ["/bin/bash"]

# ================================ STAGE: sdk =================================
# Export the linkable SDK to the host filesystem:
#   docker build --target sdk --output type=local,dest=./chrono-sdk docker/
FROM scratch AS sdk
COPY --from=builder /opt/chrono /

# =============================================================================
#  ⚠ ALIGNMENT WARNING — read before Milestone 2
#
#  `-march=skylake` enables AVX2, which makes Eigen use 32-byte alignment
#  (EIGEN_MAX_ALIGN_BYTES=32) and changes the LAYOUT of every fixed-size Eigen
#  type embedded in a Chrono class — ChVector3d, ChMatrix33, ChQuaternion, and
#  everything containing them.
#
#  Unreal compiles its own translation units with its own flags. If a UE .cpp
#  ever `#include`s a Chrono header, the two sides will disagree about object
#  layout, and you get silent heap corruption — not a link error.
#
#  This is precisely why the Milestone 2 plugin must talk to Chrono through a
#  narrow `extern "C"` surface passing only POD structs, compiled inside this
#  flag environment. Never include a Chrono or Eigen header from UE code.
# =============================================================================
