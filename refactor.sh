#!/bin/bash
set -e

echo "1. Tạo cấu trúc thư mục mới..."
mkdir -p canopen_core/include/canopen
mkdir -p canopen_core/src
mkdir -p drivers/kinematics/include/canopen/kinematics
mkdir -p drivers/kinematics/src/kinematics
mkdir -p drivers/mbdv/include/canopen/drivers
mkdir -p drivers/mbdv/src/drivers/mbdv
mkdir -p drivers/zlac8015/include/canopen/drivers
mkdir -p drivers/zlac8015/src/drivers/zlac8015

echo "2. Di chuyển header files..."
mv include/canopen/kinematics/* drivers/kinematics/include/canopen/kinematics/
mv include/canopen/drivers/mbdv drivers/mbdv/include/canopen/drivers/
mv include/canopen/drivers/zlac8015 drivers/zlac8015/include/canopen/drivers/

# Các phần còn lại của include/canopen chuyển sang canopen_core
mv include/canopen/* canopen_core/include/canopen/ 2>/dev/null || true
rm -rf include/canopen/drivers
rm -rf include/canopen/kinematics
rmdir include/canopen || true
rmdir include || true

echo "3. Di chuyển source files..."
mv src/kinematics/* drivers/kinematics/src/kinematics/
mv src/drivers/mbdv/* drivers/mbdv/src/drivers/mbdv/
mv src/drivers/zlac8015/* drivers/zlac8015/src/drivers/zlac8015/

mv src/* canopen_core/src/ 2>/dev/null || true
rm -rf src/drivers/mbdv src/drivers/zlac8015 src/drivers
rm -rf src/kinematics
rmdir src || true

echo "Hoàn thành di chuyển."
