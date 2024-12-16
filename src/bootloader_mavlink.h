#pragma once

#include <string>
#include <vector>
#include "serial_port.h"
#include "parsers/binary.h"
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <queue>

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
    /// @brief Command opcodes
    enum class Opcode : uint8_t {
        CMD_NONE, ///< ignored, always acked
        CMD_TERMINATE_SESSION, ///< Terminates open Read session
        CMD_RESET_SESSIONS, ///< Terminates all open Read sessions
        CMD_LIST_DIRECTORY, ///< List files in <path> from <offset>
        CMD_OPEN_FILE_RO, ///< Opens file at <path> for reading, returns <session>
        CMD_READ_FILE, ///< Reads <size> bytes from <offset> in <session>
        CMD_CREATE_FILE, ///< Creates file at <path> for writing, returns <session>
        CMD_WRITE_FILE, ///< Writes <size> bytes to <offset> in <session>
        CMD_REMOVE_FILE, ///< Remove file at <path>
        CMD_CREATE_DIRECTORY, ///< Creates directory at <path>
        CMD_REMOVE_DIRECTORY, ///< Removes Directory at <path>, must be empty
        CMD_OPEN_FILE_WO, ///< Opens file at <path> for writing, returns <session>
        CMD_TRUNCATE_FILE, ///< Truncate file at <path> to <offset> length
        CMD_RENAME, ///< Rename <path1> to <path2>
        CMD_CALC_FILE_CRC32, ///< Calculate CRC32 for file at <path>
        CMD_BURST_READ_FILE, ///< Burst download session file

        RSP_ACK = 128, ///< Ack response
        RSP_NAK ///< Nak response
    };
    /// @brief Maximum data size in RequestHeader::data
    static constexpr uint8_t max_data_length = 128; 

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
    PACK(struct FirmwareInfo {
        uint32_t total_size; ///< Total size of the file 
        uint8_t x;
        uint8_t y;
        uint8_t z;
    });
    // static_assert(
    //     sizeof(PayloadHeader) == sizeof(mavlink_file_transfer_protocol_t::payload),
    //     "PayloadHeader size is incorrect.");
        
    typedef enum {
        ERR_OK = 0,
        ERR_UNKNOWN,	/* Generic error */
        ERR_NACK,
        ERR_TIMEOUT,
        ERR_SKIP,	/* Skip this state */
    }err_t;

    BootloaderMavlink(Serial_Port *serial_port , string path);
    BootloaderMavlink() = delete;
    ~BootloaderMavlink();

    bool init();
    bool run();
    bool close();

    void start_read_thread(void);
    void start_write_thread(void);
    bool is_gimbal(uint8_t compid);


    uint8_t expect_compid{MAV_COMP_ID_USER75};
private:
    enum State : uint8_t{
        CONNECT,
        SETBAUD,
        WAIT_SETBAUD,
        TRANSMIT,
    };
    enum StateUploadFile : uint8_t{
        CREATE_FILE,
        WRITE_DATA,
        TERMINAL,
    };
    struct bin_t
    {
        uint32_t flash_size;
        uint32_t flash_index;
        vector<uint8_t>  flash_data;
    };
    
    struct messages_t {
        pthread_mutex_t     mutex;
        pthread_mutex_t     mutex_ftp;
        pthread_mutex_t     mutex_srl;
        pthread_mutex_t     mutex_cmd_ack;

        pthread_cond_t      _condition_srl;
        pthread_cond_t      _condition_ftp;
        pthread_cond_t      _condition_cmd_ack;
        // Heartbeat
        mavlink_heartbeat_t heartbeat        = { 0 };

        mavlink_file_transfer_protocol_t ftp = { 0 };

        mavlink_serial_control_t sr_ctl      = { 0 }; 

        mavlink_command_ack_t   cmd_ack = { 0 };
        messages_t()
        {
            pthread_mutex_init(&mutex, NULL);
            pthread_mutex_init(&mutex_srl, NULL);
            pthread_mutex_init(&mutex_ftp, NULL);
            pthread_mutex_init(&mutex_cmd_ack, NULL);
            pthread_cond_init(&_condition_srl, NULL);
            pthread_cond_init(&_condition_ftp, NULL);
            pthread_cond_init(&_condition_cmd_ack, NULL);
        }

        ~messages_t()
        {
            pthread_mutex_destroy(&mutex);
            pthread_mutex_destroy(&mutex_srl);
            pthread_mutex_destroy(&mutex_ftp);
            pthread_mutex_destroy(&mutex_cmd_ack);
            pthread_cond_destroy(&_condition_srl);
            pthread_cond_destroy(&_condition_ftp);
            pthread_cond_destroy(&_condition_cmd_ack);
        }

    };
    string		        path;
    Serial_Port         *_serial_port;
    void		        *p_st		        = NULL;
    bool                time_to_exit        {false};  
    bool                has_detected        {false};
    mavlink_channel_t   _channel            {MAVLINK_COMM_0};
    pthread_t           read_tid            = 0;
    pthread_t           write_tid           = 0;
    bool                init_flag           = false;
    State               state               = (State)0;
    bin_t               binArray;
    messages_t          _messages;
    pthread_mutex_t     w_mutex;
    queue<mavlink_message_t> w_queue;
    mavlink_system_t    _system{1,MAV_COMP_ID_ONBOARD_COMPUTER};
    mavlink_system_t    _gimbal{};
    uint64_t            timeslap_hb {0};
    void  read_thread(void);
    void  write_thread(void);
    void  messages_handler(const mavlink_message_t &message);

    bool  w_serial_control_sync(const mavlink_serial_control_t &srl_send, mavlink_serial_control_t &srl_receive);
    bool  w_ftp_sync(const PayloadHeader &ftp_send, PayloadHeader &ftp_receive);
    bool  w_reboot_sync(const mavlink_command_long_t &cmdlong, mavlink_command_ack_t &cmdack);

    void  w_heartbeat();
    bool  w_message(const mavlink_message_t &message);

    err_t parse_bin_and_send_seq();
    err_t connect_seq();
    err_t reboot_seq();
    uint64_t  lastest_timeslap_heartbeat();

};