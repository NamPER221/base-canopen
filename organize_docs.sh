#!/bin/bash
set -e

echo "1. Di chuyển tài liệu và cấu hình cho driver MBDV..."
mv GUIDE_MBDV.md drivers/mbdv/README.md
mkdir -p drivers/mbdv/config
mv config/CANOPEN-EDS-MBDV-Servo-DulAxes-V1.0.eds drivers/mbdv/config/

echo "2. Di chuyển tài liệu và cấu hình cho driver ZLAC8015..."
mkdir -p drivers/zlac8015/config
mv ZLAC8015D.eds drivers/zlac8015/config/
mv config/zlac8015_config.yaml drivers/zlac8015/config/

echo "3. Di chuyển tài liệu cho kinematics..."
mkdir -p drivers/kinematics/docs
mv odometry_kinematics_summary.md drivers/kinematics/docs/

echo "4. Di chuyển các tài liệu chung (Planning, CAN help)..."
mkdir -p docs/planning
mv PLAN.md PLAN_MOTOR_CONTROL.md docs/planning/
mv can_help.pdf docs/

echo "5. Dọn dẹp thư mục config gốc..."
rmdir config || true

echo "Hoàn tất tổ chức tài liệu!"
