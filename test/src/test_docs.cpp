/**
 * @file test_docs.cpp
 * @brief Kiểm tra tài liệu không lệch khỏi code
 *
 * Vì sao cần: docs/API.md liệt kê hàm bằng tên. Khi hàm bị đổi tên hoặc
 * xoá, tài liệu vẫn nằm đó và người đọc sẽ tin nhầm rồi debug mất thời gian
 * tìm hàm không tồn tại. Test này biến "tài liệu sai" từ lỗi im lặng thành
 * lỗi CI.
 *
 * Cách kiểm tra: mọi tên hàm xuất hiện trong docs/API.md dưới dạng `tên(`
 * phải tồn tại trong ít nhất một header của thư viện.
 *
 * LƯU Ý: test đọc file từ đĩa, nên cần chạy từ thư mục gốc của repo.
 */

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

// Gom toàn bộ nội dung header lại thành một chuỗi để tìm tên hàm
std::string read_all_headers(const std::string& include_dir) {
    std::string all;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(include_dir)) {
        if (!entry.is_regular_file()) continue;
        if (entry.path().extension() != ".hpp") continue;
        std::ifstream f(entry.path());
        if (!f) continue;
        std::stringstream ss;
        ss << f.rdbuf();
        all += ss.str();
        all += '\n';
    }
    return all;
}

// Tên hàm được nhắc trong tài liệu: `tên_hàm(`
std::set<std::string> documented_names(const std::string& doc) {
    std::set<std::string> names;
    // Bỏ qua thẻ markdown, chỉ lấy phần trong backtick
    const std::regex in_code("`([a-z_][a-z0-9_]*)\\(");
    for (auto it = std::sregex_iterator(doc.begin(), doc.end(), in_code);
         it != std::sregex_iterator(); ++it) {
        names.insert((*it)[1].str());
    }
    return names;
}

} // namespace

int main(int argc, char** argv) {
    const std::string root = (argc > 1) ? argv[1] : ".";
    const std::string api_path = root + "/docs/API.md";
    const std::string include_dir = root + "/include/canopen";

    std::ifstream api(api_path);
    if (!api) {
        std::cerr << "Không đọc được " << api_path
                  << " — chạy test từ thư mục gốc repo\n";
        return 1;
    }
    std::stringstream ss;
    ss << api.rdbuf();

    const std::string headers = read_all_headers(include_dir);
    if (headers.empty()) {
        std::cerr << "Không đọc được header nào trong " << include_dir << "\n";
        return 1;
    }

    const auto names = documented_names(ss.str());
    std::cout << "Tài liệu nhắc " << names.size() << " hàm\n";

    std::vector<std::string> missing;
    for (const auto& n : names) {
        if (headers.find(n + "(") == std::string::npos) {
            missing.push_back(n);
        }
    }

    if (!missing.empty()) {
        std::cerr << "\nFAIL: " << missing.size()
                  << " hàm có trong docs/API.md nhưng không có trong header:\n";
        for (const auto& m : missing) std::cerr << "  - " << m << "\n";
        std::cerr << "\nSửa docs/API.md (xoá/ghi đúng tên) hoặc khôi phục hàm.\n";
        return 1;
    }

    std::cout << "PASS: mọi hàm trong docs/API.md đều tồn tại trong header\n";
    return 0;
}
