#include <string>
#include <cstring>

inline bool validate_file_name(char const* s) {
    if (strlen(s) != 12) return false;
    for (int i = 0; i < 3; ++i)
        if (char c = toupper(s[i]); c < 'A' || c > 'Z') return false;
    for (int i = 3; i < 8; ++i)
        if (char c = s[i]; c < '0' || c > '9') return false;
    if (char c = s[8]; c != '.') return false;
    for (int i = 9; i < 12; ++i)
        if (char c = toupper(s[i]); c < 'A' || c > 'Z') return false;
    return true;
}

inline int filename_to_index(std::string& s) {
    if (!validate_file_name(s.c_str())) {
        return -1;
    }
    return ((s[1] - 'A') * 26 + (s[2] - 'A')) * 100000 + std::stoi(s.substr(3, 5));
}

inline void index_to_filename(int idx, char* buf, bool full_path = true) {
    int epoch = idx / 100000;
    int index = idx % 100000;
    static constexpr char templ[] = "/flash/L%c%c%05d.bin";
    static constexpr char templ_s[] = "L%c%c%05d.bin";
    snprintf(buf, 30, full_path ? templ : templ_s, 'A' + epoch / 26, 'A' + epoch % 26, index);
}