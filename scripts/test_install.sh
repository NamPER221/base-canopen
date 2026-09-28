#!/usr/bin/env bash
# =============================================================================
# Kiểm chứng việc cài đặt thư viện và tiêu thụ từ BÊN NGOÀI
#
# Vì sao cần: lỗi đóng gói (thiếu canopenConfig.cmake, sai include path,
# thiếu phụ thu) KHÔNG BAO GIỜ lộ ra khi build tại chỗ — chỉ lộ ra khi một
# dự án khác cài thư viện rồi dùng find_package(canopen). Đây là bài kiểm tra
# bắt buộc cho bất kỳ thư viện C++ nào phát hành ra ngoài.
#
# Dùng:  ./scripts/test_install.sh
# Giữ nguyên mọi thứ trong thư mục tạm, không chạm vào build/ của repo.
# =============================================================================
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK="$(mktemp -d /tmp/canopen-install-test.XXXXXX)"
BUILD="$WORK/build"
PREFIX="$WORK/prefix"
CONSUMER_BUILD="$WORK/consumer-build"

cleanup() { rm -rf "$WORK"; }
trap cleanup EXIT

step() { printf '\n=== %s ===\n' "$1"; }

step "1/5  Build thư viện (test/example tắt để build nhanh)"
cmake -S "$SRC" -B "$BUILD" \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DCMAKE_INSTALL_PREFIX="$PREFIX" \
      > "$WORK/configure.log" 2>&1 \
  || { echo "CẤU HÌNH THẤT BẠI:"; tail -30 "$WORK/configure.log"; exit 1; }
echo "  ok"

step "2/5  Build"
cmake --build "$BUILD" -j"$(nproc)" > "$WORK/build.log" 2>&1 \
  || { echo "BUILD THẤT BẠI:"; tail -30 "$WORK/build.log"; exit 1; }
echo "  ok"

step "3/5  Cài đặt vào $PREFIX"
cmake --install "$BUILD" > "$WORK/install.log" 2>&1 \
  || { echo "CÀI ĐẶT THẤT BẠI:"; tail -30 "$WORK/install.log"; exit 1; }

echo "  Nội dung prefix:"
find "$PREFIX" -maxdepth 3 \
     \( -name '*.cmake' -o -name '*.a' -o -name '*.pc' -o -name '*.hpp' \) \
     | sed "s|$PREFIX|    |" | sort | head -20

# Bắt buộc phải có file Config, không có thì find_package sẽ thất bại
for f in lib/cmake/canopen/canopenConfig.cmake \
         lib/cmake/canopen/canopenTargets.cmake \
         lib/cmake/canopen/canopenConfigVersion.cmake; do
    if [[ ! -f "$PREFIX/$f" ]]; then
        echo "  THIẾU FILE: $f  (find_package(canopen) sẽ thất bại)" >&2
        exit 1
    fi
done
echo "  canopenConfig.cmake: có"

step "4/5  Dự án bên ngoài dùng find_package(canopen)"
cmake -S "$SRC/test/consumer" -B "$CONSUMER_BUILD" \
      -DCMAKE_PREFIX_PATH="$PREFIX" \
      -DCMAKE_BUILD_TYPE=Release \
      > "$WORK/consumer-cfg.log" 2>&1 \
  || { echo "find_package THẤT BẠI:"; tail -30 "$WORK/consumer-cfg.log"; exit 1; }
echo "  find_package(canopen): ok"

cmake --build "$CONSUMER_BUILD" -j"$(nproc)" > "$WORK/consumer-build.log" 2>&1 \
  || { echo "BUILD CONSUMER THẤT BẠI:"; tail -30 "$WORK/consumer-build.log"; exit 1; }
echo "  build: ok"

step "5/5  Chạy và kiểm tra kết quả"
rc=0
ctest --test-dir "$CONSUMER_BUILD" --output-on-failure || rc=1

# Thêm: pkg-config phải dùng được
if command -v pkg-config > /dev/null 2>&1; then
    if PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig" pkg-config --exists canopen; then
        echo "  pkg-config canopen: $(PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig" \
                                     pkg-config --modversion canopen)"
    else
        echo "  pkg-config: KHÔNG tìm thấy canopen.pc" >&2
        rc=1
    fi
fi

if [[ $rc -ne 0 ]]; then
    echo
    echo "TEST CÀI ĐẶT THẤT BẠI" >&2
    exit 1
fi

echo
echo "TEST CÀI ĐẶT ĐẠT — thư viện dùng được từ dự án khác."
