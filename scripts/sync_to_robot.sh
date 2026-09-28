#!/usr/bin/env bash
# =============================================================================
# Đồng bộ mã nguồn từ máy phát triển sang robot
#
# Dùng:  ./scripts/sync_to_robot.sh robotics@192.168.1.50
#
# Chỉ copy các file thực sự thay đổi, giữ nguyên thứ tự đường dẫn trên robot
# (nguồn -> đúng thư mục đích). KHÔNG copy thư mục build/.
# =============================================================================
set -euo pipefail

REMOTE="${1:-}"
if [[ -z "$REMOTE" ]]; then
    echo "Usage: $0 <user>@<robot-ip>   (vd: robotics@192.168.1.50)" >&2
    exit 1
fi

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DST="${REMOTE}:~/NamNC/base-canopen"

# Mỗi dòng: đường dẫn tương đối trong repo (thư mục đích giữ nguyên)
FILES=(
    "CMakeLists.txt"

    "include/canopen/can/frame/frame.hpp"
    "include/canopen/co/cia402/cia402_drive.hpp"
    "include/canopen/co/nmt/nmt.hpp"
    "include/canopen/co/sdo/sdo.hpp"
    "include/canopen/device/device_profile.hpp"
    "include/canopen/device/motor_device.hpp"
    "include/canopen/drivers/zlac8015/zlac8015_driver.hpp"

    "src/can/msg/message_factory.cpp"
    "src/co/nmt/nmt.cpp"
    "src/co/sdo/sdo.cpp"
    "src/device/device_profile.cpp"
    "src/device/motor_device.cpp"
    "src/drivers/zlac8015/zlac8015_driver.cpp"

    "examples/CMakeLists.txt"
    "examples/keyboard_control.cpp"
    "examples/motor_device_demo.cpp"
    "examples/pdo_test.cpp"
    "examples/robot_move.cpp"

    "drivers/zlac8015/CMakeLists.txt"

    "config/zlac8015_config.yaml"

    "test/CMakeLists.txt"
    "test/src/test_sdo.cpp"
    "test/src/test_nmt.cpp"
    "test/src/test_device_profile.cpp"
    "test/src/test_motor_device.cpp"
)

missing=0
for f in "${FILES[@]}"; do
    if [[ ! -f "$SRC/$f" ]]; then
        echo "  BỎ QUA (không tồn tại): $f" >&2
        missing=1
        continue
    fi
    printf '  %-52s' "$f"
    if scp -q "$SRC/$f" "$DST/$f" 2>/dev/null; then
        echo "ok"
    else
        echo "LỖI"
        missing=1
    fi
done

if [[ $missing -ne 0 ]]; then
    echo >&2
    echo "CÓ FILE KHÔNG COPY ĐƯỢC — kiểm tra IP/kết nối SSH." >&2
    exit 1
fi

echo
echo "Đã copy ${#FILES[@]} file. Trên robot chạy:"
echo "  cd ~/NamNC/base-canopen/build && make -j4"
