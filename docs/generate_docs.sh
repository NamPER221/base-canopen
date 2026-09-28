#!/usr/bin/env bash
# Sinh tài liệu tra cứu từ comment trong header.
#
# Dùng:  ./docs/generate_docs.sh
# Kết quả: docs/html/index.html
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

# Doxygen phân giải đường dẫn tương đối theo thư mục đang chạy, KHÔNG theo
# vị trí file Doxyfile. Vì vậy phải cd vào docs/ trước khi gọi.
doxygen Doxyfile

echo "Xong: docs/html/index.html"
