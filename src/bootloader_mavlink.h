#pragma once

#include <string>
#include <vector>
#include "serial_port.h"
#include "parsers/binary.h"

// As found in
// https://stackoverflow.com/questions/1537964#answer-3312896
#ifdef _MSC_VER // MSVC
#define PACK(__Declaration__) __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))
#else
#define PACK(__Declaration__) __Declaration__ __attribute__((__packed__))
#endif

using namespace std;

class BootloaderMavlink
{
public:
    /// @brief Maximum data size in RequestHeader::data
    static constexpr uint8_t max_data_length = 239; 

    /// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload.
    /// This needs to be packed, because it's typecasted from
    /// mavlink_file_transfer_protocol_t.payload, which starts at a 3 byte offset, causing an
    /// unaligned access to seq_number and offset
    PACK(struct PayloadHeader {
        uint16_t seq_number; ///< sequence number for message
        uint8_t session; ///< Session id for read and write commands
        Opcode opcode; ///< Command opcode
        uint8_t size; ///< Size of data
        Opcode req_opcode; ///< Request opcode returned in RSP_ACK, RSP_NAK message
        uint8_t burst_complete; ///< Only used if req_opcode=CMD_BURST_READ_FILE - 1: set of burst
        ///< packets complete, 0: More burst packets coming.
        uint8_t padding; ///< 32 bit alignment padding
        uint32_t offset; ///< Offsets for List and Read commands
        uint8_t data[max_data_length]; ///< command data, varies by Opcode
    });

    static_assert(
        sizeof(PayloadHeader) == sizeof(mavlink_file_transfer_protocol_t::payload),
        "PayloadHeader size is incorrect.");
        
    typedef enum {
        ERR_OK = 0,
        ERR_UNKNOWN,	/* Generic error */
        ERR_NACK,
        ERR_TIMEOUT,
        ERR_NO_CMD,	/* Command not available in bootloader */
    }err_t;

    BootloaderMavlink(Serial_Port *serial_port , string path);
    BootloaderMavlink() = delete;
    ~BootloaderMavlink();

    bool init();
    bool run();

    void start_read_thread(void);
    bool is_gimbal(uint8_t compid);

private:
    enum State : uint8_t{
        PARSER,
        CONNECT,
        TRANSMIT,
        COMPLETE,
    };
    struct bin_t
    {
        uint32_t flash_size;
        uint32_t flash_index;
        vector<uint8_t>  flash_data;
    };
        struct messages_t {
        pthread_mutex_t mutex;

        // Heartbeat
        mavlink_heartbeat_t heartbeat = { 0 };

        

        messages_t()
        {
            pthread_mutex_init(&mutex, NULL);
        }

        ~messages_t()
        {
            pthread_mutex_destroy(&mutex);
        }

        void reset_timestamps()
        {
            timestamps.reset_timestamps();
        }
    };
    string		path;
    Serial_Port *_serial_port;
    void		*p_st		= NULL;
    bool time_to_exit{true};  
    mavlink_channel_t _channel{MAVLINK_COMM_0};
    pthread_t read_tid  = 0;
    bool init_flag       = false;
    State state = (State)0;
    bin_t binArray;

    void  read_thread(void);
    void  messages_handler(const mavlink_message_t &message);

    err_t parse_bin_and_send_seq();
};