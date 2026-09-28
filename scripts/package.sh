#!/usr/bin/env bash
# =============================================================================
# Đóng gói thư viện thành 3 loại gói
#
#   Source.tar.gz   mã nguồn, build được ở mọi máy
#   amd64.deb / tar.gz   cho máy dev
#   arm64.deb / tar.gz   cho robot (cross-compile)
#
# Dùng:  ./scripts/package.sh [x86_64] [aarch64]
# Mặc định đóng gói cả hai. Chỉ đóng gói những kiến trúc mà có toolchain.
#
# Kết quả nằm trong dist/
# =============================================================================
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIST="$SRC/dist"
JOBS="$(nproc)"

mkdir -p "$DIST"

step() { printf '\n\033[1m=== %s ===\033[0m\n' "$1"; }

build_arch() {
    local arch="$1"
    local build="$SRC/build-$arch"

    # CPack đặt tên gói theo TÊN KIẾN TRÚC CỦA DEBIAN, không phải tên CPU:
    # x86_64 -> amd64, aarch64 -> arm64. Lệch tên này thì glob trong bước
    # mv bên dưới không khớp, file gói rơi lại trong thư mục build rồi bị xoá.
    local deb_arch
    case "$arch" in
        x86_64)  deb_arch="amd64" ;;
        aarch64) deb_arch="arm64" ;;
        *)       deb_arch="$arch" ;;
    esac

    if [[ "$arch" == "x86_64" ]]; then
        local tc=()
    else
        if ! command -v "${arch}-linux-gnu-g++" > /dev/null 2>&1; then
            echo "BỎ QUA $arch: chưa cài toolchain. Chạy:" >&2
            echo "  sudo apt install gcc-${arch}-linux-gnu g++-${arch}-linux-gnu" >&2
            return 0
        fi
        local tc=(-DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-aarch64.cmake)
    fi

    step "Build $arch"
    rm -rf "$build"
    cmake -S "$SRC" -B "$build" \
          -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_TESTING=OFF \
          -DBUILD_EXAMPLES=ON \
          "${tc[@]}" 2>&1 | tail -3
    cmake --build "$build" -j"$JOBS" 2>&1 | grep -iE "error|warning" || true
    echo "  build xong"

    step "Gói $arch"
    (cd "$build" && cpack -G DEB 2>&1 | grep -E "package:" || true
                  cpack -G TGZ 2>&1 | grep -E "package:" || true)
    if ! mv -f "$build"/canopen-*-"$deb_arch".deb "$DIST"/ 2>/dev/null; then
        echo "  THẤT BẠI: không tìm thấy file .deb cho $deb_arch" >&2
    fi
    if ! mv -f "$build"/canopen-*-"$deb_arch".tar.gz "$DIST"/ 2>/dev/null; then
        echo "  THẤT BẠI: không tìm thấy file .tar.gz cho $deb_arch" >&2
    fi
    rm -rf "$build"
}

# Cho phép chỉ đóng gói một kiến trúc: ./scripts/package.sh aarch64
if [[ $# -gt 0 ]]; then
    ARCHES=("$@")
else
    ARCHES=(x86_64 aarch64)
fi

for arch in "${ARCHES[@]}"; do
    case "$arch" in
        x86_64|aarch64) build_arch "$arch" ;;
        *) echo "Kiến trúc không hỗ trợ: $arch (chỉ nhận x86_64 hoặc aarch64)" >&2; exit 1 ;;
    esac
done

step "Gói mã nguồn"
rm -rf "$DIST"/canopen-*-Source.tar.gz
# git archive chỉ lấy đúng những gì đã commit — nên mã nguồn phát hành không
# bao giờ lỡ tay chứa file tạm hay build artifact.
if git -C "$SRC" rev-parse --git-dir > /dev/null 2>&1; then
    VERSION="$(grep -m1 'project(canopen VERSION' "$SRC/CMakeLists.txt" \
               | sed -E 's/.*VERSION ([0-9.]+).*/\1/')"
    git -C "$SRC" archive --format=tar.gz \
        -o "$DIST/canopen-$VERSION-Source.tar.gz" \
        --prefix="canopen-$VERSION/" HEAD
    echo "  canopen-$VERSION-Source.tar.gz"
else
    echo "  không phải git repo — bỏ qua gói mã nguồn" >&2
fi

step "Kết quả"
ls -lh "$DIST" | awk 'NR>1 {printf "  %-6s %s\n", $5, $9}'

cat <<'EOF'

Cài đặt:
  # máy dev (Debian/Ubuntu)
  sudo dpkg -i dist/canopen-*-amd64.deb

  # robot: giải nén bản nén (không cần dpkg, không cần quyền root)
  tar xzf dist/canopen-*-arm64.tar.gz -C /opt
  # → /opt/canopen/usr/...  hoặc tương tự tuỳ bản đóng gói

  # hoặc từ mã nguồn
  tar xzf dist/canopen-*-Source.tar.gz && cd canopen-*/
  cmake -S . -B build && cmake --build build -j$(nproc) && sudo cmake --install build
EOF
