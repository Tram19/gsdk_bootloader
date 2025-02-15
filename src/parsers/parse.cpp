#include "parse.h"

using json = nlohmann::json;

json * json_data;

std::vector<unsigned char> hexStringToBytes(std::string hex) {
    std::vector<unsigned char> bytes;
    
    // Nếu chuỗi có khoảng trắng, loại bỏ chúng:
    hex.erase(std::remove_if(hex.begin(), hex.end(), ::isspace), hex.end());
    
    // Nếu độ dài không chẵn, bạn có thể padding thêm '0' ở đầu hoặc xử lý theo yêu cầu:
    if (hex.length() % 2 != 0) {
        hex = "0" + hex;
    }
    
    for (size_t i = 0; i < hex.length(); i += 2) {
        std::string byteString = hex.substr(i, 2);
        unsigned char byte = static_cast<unsigned char>(std::stoul(byteString, nullptr, 16));
        bytes.push_back(byte);
    }
    return bytes;
}

unsigned int    gremsy_size()
{
    if (json_data == NULL)
    {
        return 0;
    }
    else
    {
        if (!json_data->contains("image_size") ) {
            return 0;
        }
        return json_data->at("image_size").get<uint32_t>();
    }
    
}

bool            gremsy_close()
{
    delete json_data;
    return true;
}

bool            gremsy_open(std::string filename)
{
    json_data = new json();
    std::ifstream i(filename);
    if (!i.is_open()) {
        return false;
    }
    try {
        i >> *json_data;
    } catch (std::exception &e) {
        return false;
    }
    return true;
}

std::vector<unsigned char> gremsy_read()
{
    if (json_data == NULL)
    {
        return std::vector<unsigned char>();
    }
    else
    {
        if (!json_data->contains("image") ) {
            return std::vector<unsigned char>();
        }
        return hexStringToBytes(json_data->at("image").get<std::string>());
    }
}

bool            gremsy_version(uint8_t *x, uint8_t *y, uint8_t *z)
{
    if (json_data == NULL)
    {
        return 0;
    }
    else
    {
        // if (!json_data->contains("version") ) {
        //     return 0;
        // }
        // *x = (*json_data)["version"]["x"];
        // *y = (*json_data)["version"]["y"];
        // *z = (*json_data)["version"]["z"];
    }
}

uint32_t            gremsy_crc32()
{
    if (json_data == NULL)
    {
        return 0;
    }
    else
    {
        if (!json_data->contains("crc32") ) {
            return 0;
        }
        return json_data->at("crc32").get<uint32_t>();

    }
}