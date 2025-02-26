#include "parse.h"

static Json::Value root;


bool gremsy_open(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Cannot open JSON file: " << filename << std::endl;
        return false;
    }
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    std::string errs;
    bool ok = Json::parseFromStream(builder, file, &root, &errs);
    file.close();
    if (!ok) {
        std::cerr << "Failed to parse JSON: " << errs << std::endl;
        return false;
    }
    return true;
}

unsigned int gremsy_size() {
    if (!root.isMember("image_size") || !root["image_size"].isUInt()) {
        return 0;
    }
    return root["image_size"].asUInt();
}


std::vector<unsigned char> hexStringToBytes(const std::string& hex) {
    std::vector<unsigned char> bytes;
    bytes.reserve(hex.size() / 2);
    for (size_t i = 0; i < hex.length(); i += 2) {
        std::string byteStr = hex.substr(i, 2);
        unsigned char byte = static_cast<unsigned char>(strtol(byteStr.c_str(), nullptr, 16));
        bytes.push_back(byte);
    }
    return bytes;
}


std::vector<unsigned char> gremsy_read() {
    if (!root.isMember("image") || !root["image"].isString()) {
        return std::vector<unsigned char>();
    }
    return hexStringToBytes(root["image"].asString());
}


bool gremsy_version(uint8_t &x, uint8_t &y, uint8_t &z) {
    if (!root.isMember("version") || !root["version"].isInt()) {
        return false;
    }
    int version = root["version"].asInt();
    z = version % 10;
    y = (version / 10) % 10;
    x = (version / 100) % 10;
    return true;
}


uint32_t gremsy_crc32() {
    if (!root.isMember("crc32") || !root["crc32"].isUInt()) {
        return false;
    }
    return static_cast<uint32_t>(root["crc32"].asUInt64());
}


void gremsy_close() {
    root.clear();
}
