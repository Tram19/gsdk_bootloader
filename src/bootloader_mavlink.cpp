#include <filesystem>
#include <iostream>
#include "bootloader_mavlink.h"


namespace fs = std::filesystem;
uint32_t start_time;
// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------


static uint64_t get_time_usec()
{
    static struct timeval _timestamp;
    gettimeofday(&_timestamp, NULL);
    return _timestamp.tv_sec * 1000000 + _timestamp.tv_usec;
}

static uint64_t get_time_msec()
{
    static struct timeval _timestamp;
    gettimeofday(&_timestamp, NULL);
    return _timestamp.tv_sec * 1000 + _timestamp.tv_usec / 1000;
}

static void *start_bootloader_read_thread(void *args);

static void *start_bootloader_write_thread(void *args);

BootloaderMavlink::BootloaderMavlink(Serial_Port *serial_port , string path, uint8_t expect_compid)
{
    this -> path = path;
    this -> _serial_port = serial_port;
    this -> expect_compid = expect_compid;
}

BootloaderMavlink::~BootloaderMavlink()
{
    time_to_exit = true;
    close();
}

bool BootloaderMavlink::init()
{
    if (_serial_port->status != SERIAL_PORT_OPEN)
    {
        _serial_port->baudrate = expect_baudrate;
        _serial_port->open_serial();
    }
    pthread_mutex_init(&w_mutex, NULL);
    int result = pthread_create(&read_tid, NULL, &start_bootloader_read_thread, this);
    if (result) throw result;

    result = pthread_create(&write_tid, NULL, &start_bootloader_write_thread, this);
    if (result) throw result;
    // try to init parser
    if(!gremsy_open(path))
    {
        GSDK_DebugError("Cannot open file %s\n", path.c_str());
        return false;
    }
    GSDK_DebugInfo("Crc32: %d\n", gremsy_crc32());
    GSDK_DebugInfo("Size: %d\n", gremsy_size());

    init_flag = true;
    start_time = get_time_msec();
    timeslap_hb = get_time_msec();
    return true;
}
bool BootloaderMavlink::close()
{
    if (init_flag)
    {
        pthread_mutex_destroy(&w_mutex);
        pthread_cancel(read_tid);
        pthread_cancel(write_tid);
        gremsy_close();
        _serial_port->close_serial();
    }
    return true;
}
bool BootloaderMavlink::run()
{
    if (!init_flag)
    {
        GSDK_DebugError("Bootloader Mavlink is not initialized yet!\n");
        return false;
    }
    uint8_t total_e = 0;


    while (1)
    {

        uint64_t time  = lastest_timeslap_heartbeat();
        if (time > 20000)
        {
            GSDK_DebugError("Cannot communication!\n");
            return false;
        }
        else if((time < 20000) && !has_detected)
        {

            sleep(1);
            continue;
        }

        switch (state)
        {
        case SEND_REBOOT:
        {
            if(reboot_seq() == ERR_OK)
            {
                total_e = 0; // reset error
                set_progress(0);
                state = CONNECT;
            }
            else
            {
                total_e++;
                usleep(1000000);//wait 1 seconds gimbal reboot.
            }
        }
            break;
        case CONNECT:
        {

            err_t result = connect_seq();
            if (result == ERR_OK)
            {

                state = SETBAUD;
            }
            else if (result == ERR_SKIP)
            {

                state = TRANSMIT;
            }
            else
            {
                total_e++;
            }
            
        }
            break;
        case SETBAUD:
        {
            _serial_port->stop();
            _serial_port->baudrate = expect_baudrate;
            _serial_port->start();
            has_detected = false;
            state = WAIT_SETBAUD;
        }
            break;
        case WAIT_SETBAUD:
        {
            state = TRANSMIT;
        }
            break;
        case TRANSMIT:
        {

            if(parse_bin_and_send_seq() == ERR_OK)
            {
                GSDK_DebugSuccess("Code flashed successfully!");
                return true;
            }            
            else
            {

                return false;
            }
        }
            break;
        default:
            break;
        }
        
        if (total_e >= 10)
        {
            GSDK_DebugError("Code flash failed!\n");
            return false;
        }
        
    }
    
}
uint64_t BootloaderMavlink::lastest_timeslap_heartbeat()
{
    uint64_t lastest_time = 0;
    pthread_mutex_lock(&_messages.mutex);
    lastest_time = get_time_msec() - (timeslap_hb);
    pthread_mutex_unlock(&_messages.mutex);
    return (lastest_time);
}
bool BootloaderMavlink::w_message(const mavlink_message_t &message)
{
    pthread_mutex_lock(&w_mutex);
    w_queue.push(message);
    pthread_mutex_unlock(&w_mutex);
    return 1;
}
void  BootloaderMavlink::w_heartbeat()
{

    mavlink_heartbeat_t heartbeat = { 0 };
    heartbeat.type 			= MAV_TYPE_ONBOARD_CONTROLLER;
    heartbeat.autopilot 	= MAV_AUTOPILOT_ARDUPILOTMEGA;
    heartbeat.base_mode 	= 0;
    heartbeat.custom_mode 	= 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message = { 0 };

    mavlink_msg_heartbeat_encode_chan(_system.sysid, _system.compid, _channel, &message, &heartbeat);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------
    if (w_message(message) <= 0)
    GSDK_DebugError("WARNING: could not send HEARTBEAT\n");

}
bool  BootloaderMavlink::w_serial_control_sync(const mavlink_serial_control_t &srl_send, mavlink_serial_control_t &srl_receive)
{
    mavlink_message_t message = { 0 };
    mavlink_msg_serial_control_encode_chan(_system.sysid, _system.compid, _channel, &message, &srl_send);


    struct timespec time_wait;
    int rt = 0;
    bool result = false;
    clock_gettime(CLOCK_REALTIME, &time_wait);
// Wait for 1 sec timeout
    time_wait.tv_sec += 3;
    w_message(message);
    pthread_mutex_lock(&_messages.mutex_srl);
    rt = pthread_cond_timedwait(&_messages._condition_srl, &_messages.mutex_srl, &time_wait);
    if (rt != ETIMEDOUT) {
        result = true;
        memcpy(&srl_receive, &_messages.sr_ctl, sizeof(mavlink_serial_control_t));
    }
    pthread_mutex_unlock(&_messages.mutex_srl);

    return result;
}

bool  BootloaderMavlink::w_ftp_sync(const PayloadHeader &ftp_send, PayloadHeader &ftp_receive)
{
    mavlink_message_t message = { 0 };
    const uint8_t * data = reinterpret_cast<const uint8_t*>(&ftp_send);

    GSDK_DebugMsg("Send data --------------------------\n");
    GSDK_DebugMsg("seq_number : %u\n",ftp_send.seq_number);
    GSDK_DebugMsg("session : %u\n",ftp_send.session);
    GSDK_DebugMsg("OP code : %u\n",ftp_send.opcode);

    mavlink_msg_file_transfer_protocol_pack_chan(
                                                _system.sysid, 
                                                _system.compid, 
                                                _channel, 
                                                &message, 
                                                0,
                                                _gimbal.sysid,
                                                _gimbal.compid,
                                                data);
    struct timespec time_wait;
    int rt = 0;
    bool result = false;
    clock_gettime(CLOCK_REALTIME, &time_wait);
    // Wait for 5 sec timeout
    time_wait.tv_sec += 5;
    w_message(message);
    pthread_mutex_lock(&_messages.mutex_ftp);
    rt = pthread_cond_timedwait(&_messages._condition_ftp, &_messages.mutex_ftp, &time_wait);
    if (rt != ETIMEDOUT) {
        if (_messages.ftp.target_component == _system.compid && _messages.ftp.target_system == _system.sysid)
        {
            result = true;
            memcpy((&ftp_receive) , &_messages.ftp.payload[0] , sizeof(PayloadHeader));
            GSDK_DebugMsg("Receive data -----------------------\n");
            GSDK_DebugMsg("seq_number : %u\n",ftp_receive.seq_number);
            GSDK_DebugMsg("session : %u\n",ftp_receive.session);
            GSDK_DebugMsg("OP code : %u\n",ftp_receive.opcode);
        }else
        {
            GSDK_DebugError("Wrong ID\n");
        }
        
    }else
    {
        // printf("Time out in FTP\n");
    }
    
    pthread_mutex_unlock(&_messages.mutex_ftp);
    return result;
}
bool  BootloaderMavlink::w_reboot_sync(const mavlink_command_long_t &cmdlong, mavlink_command_ack_t &cmdack)
{
    mavlink_message_t message = { 0 };
    mavlink_msg_command_long_encode_chan(
                                    _system.sysid, 
                                    _system.compid, 
                                    _channel,
                                    &message,
                                    &cmdlong
                                    );
    struct timespec time_wait;
    int rt = 0;
    bool result = false;
    clock_gettime(CLOCK_REALTIME, &time_wait);
    // Wait for 1 sec timeout
    time_wait.tv_sec += 1;
    w_message(message);
    pthread_mutex_lock(&_messages.mutex_cmd_ack);
    rt = pthread_cond_timedwait(&_messages._condition_cmd_ack, &_messages.mutex_cmd_ack, &time_wait);
    if (rt != ETIMEDOUT) {
        if (_messages.cmd_ack.target_component == _system.compid && _messages.cmd_ack.target_system == _system.sysid && _messages.cmd_ack.command == cmdlong.command)
        {
            result = true;
            memcpy(&cmdack,&_messages.cmd_ack,sizeof( mavlink_command_ack_t));
        }
    }else
    {
        // printf("Timeout reboot!\n\n");
    }

    pthread_mutex_unlock(&_messages.mutex_cmd_ack);
    return result;
}
BootloaderMavlink::err_t BootloaderMavlink::reboot_seq()
{
    if(!has_detected) return ERR_SKIP;
    mavlink_command_long_t cmd_long{0};
    mavlink_command_ack_t  cmd_ack{0};
    cmd_long.command = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
    cmd_long.param3  = 3;
    cmd_long.param4  = expect_compid;
    cmd_long.target_component = expect_compid;
    cmd_long.target_system = _gimbal.sysid;
    GSDK_DebugMsg("Send reboot, sys : %u, comp: %u\n", _gimbal.sysid ,expect_compid);
    if(!w_reboot_sync(cmd_long, cmd_ack))
    {
        GSDK_DebugError("Reboot Time out: %u\n",cmd_ack.result);
        return ERR_TIMEOUT;
    } 

    if (cmd_ack.result != 0)
    {
        GSDK_DebugError("Reboot NACK: %u\n",cmd_ack.result);
        return ERR_NACK;
    }
    return ERR_OK;
}
BootloaderMavlink::err_t BootloaderMavlink::connect_seq()
{
    mavlink_serial_control_t srl;
    srl.device = SERIAL_CONTROL_SERIAL0;
    srl.flags = SERIAL_CONTROL_FLAG_RESPOND;
    srl.timeout = 1000; // timeout 1s
    srl.baudrate = expect_baudrate;
    srl.count = 0;
    srl.target_system = _gimbal.sysid;
    srl.target_component = _gimbal.compid;
    mavlink_serial_control_t srl_receive{0};
    

    if(!w_serial_control_sync(srl, srl_receive)) return ERR_TIMEOUT;

    if (srl_receive.target_component != _system.compid && srl_receive.target_system != _system.sysid &&
        srl_receive.device == SERIAL_CONTROL_SERIAL0 && srl_receive.flags == SERIAL_CONTROL_FLAG_REPLY) 
    {
        return ERR_UNKNOWN;
    } 
    
    if (srl_receive.baudrate != srl.baudrate || srl_receive.baudrate == _serial_port->baudrate) return ERR_SKIP;

    return ERR_OK;
}

BootloaderMavlink::err_t BootloaderMavlink::parse_bin_and_send_seq()
{
    unsigned int size = 0;
    uint32_t total_size = gremsy_size();
    PayloadHeader ftp_rq = {0};
    PayloadHeader ftp_rs = {0};
    StateUploadFile state_uploadfile = CREATE_FILE;
    uint8_t       err = 0;
    enum WriteDataState{
        PASER,
        SEND
    };
    uint32_t offset = 0;
    
    WriteDataState write_state = PASER;
    std::vector<unsigned char> image = gremsy_read();
    while (1)
    {
        switch (state_uploadfile)
        {
        case CREATE_FILE:
        {
            
            ftp_rq.session          = 0;
            ftp_rq.opcode           = Opcode::CMD_CREATE_FILE;
            ftp_rq.size             = sizeof(FirmwareInfo);
            FirmwareInfo            fw_info{0};
            fw_info.total_size = total_size;
            fw_info.crc32 = gremsy_crc32();
            
            gremsy_version(fw_info.x, fw_info.y, fw_info.z);
            GSDK_DebugMsg("Firmware version: %u.%u.%u\n",fw_info.x, fw_info.y, fw_info.z);
            memcpy(&ftp_rq.data[0],&fw_info,sizeof(FirmwareInfo));
            ftp_rq.burst_complete   = 0;
            ftp_rq.offset           = 0;

            if(!w_ftp_sync(ftp_rq, ftp_rs))  
            {
                err ++;
                break;
            }

            if (ftp_rs.opcode != Opcode::RSP_ACK || ftp_rs.seq_number != ftp_rq.seq_number)
            {
                GSDK_DebugError("Oh, wrong at req opcode : %u -  OP CODE : %u\n",ftp_rs.seq_number,ftp_rs.opcode);
                err ++;
                break;
            }
            if (ftp_rs.size == 1)
            {
                max_data_length = ftp_rs.data[0];
            }
            
            ftp_rq.seq_number++;
            state_uploadfile = WRITE_DATA;
        }
            break;
        case WRITE_DATA:
        {
            
            switch (write_state)
            {
            case PASER: // NO break in this case, for sending after parse
            {
                uint32_t left	= total_size - offset;
                uint32_t len = max_data_length > left ? left : max_data_length;
                uint32_t len_t = len;
                memcpy(&ftp_rq.data[0],&image[offset],len);
                ftp_rq.size = len;
                write_state = SEND;
                // printf("Offset : %lu\n",offset);
                // printf("---------------\n");
            }
            case SEND:
            {
                ftp_rq.opcode           = Opcode::CMD_WRITE_FILE;
                ftp_rq.offset =         offset;
                if(!w_ftp_sync(ftp_rq, ftp_rs))  
                {
                    GSDK_DebugError("Time out in send fw step\n");
                    err ++;
                    break;
                }

                if (ftp_rs.opcode != Opcode::RSP_ACK || ftp_rs.seq_number != ftp_rq.seq_number)
                {
                    GSDK_DebugError("Oh, wrong at req opcode : %u -  OP CODE : %u\n",ftp_rs.seq_number,ftp_rs.opcode);
                    err ++;
                    break;
                }
                offset += ftp_rq.size;
                ftp_rq.seq_number++;
                write_state = PASER;

                set_progress(static_cast<int>((offset * 100) / total_size));
            }
                break;
            default:
                break;
            }
            // Make sure write all the file
            if (offset >= total_size && write_state == PASER)
            {
                // printf("Offset : %lu\n",offset);
                state_uploadfile = TERMINAL;
                break;
            }
        }
            break;
        case TERMINAL:
        {
            
            ftp_rq.opcode           = Opcode::CMD_TERMINATE_SESSION;
            ftp_rq.size             = 0;
            if(!w_ftp_sync(ftp_rq, ftp_rs))  
            {
                GSDK_DebugError("Time out in send fw step\n");
                err ++;
                break;
            }

            if (ftp_rs.opcode != Opcode::RSP_ACK || ftp_rs.seq_number != ftp_rq.seq_number)
            {
                GSDK_DebugError("Oh, wrong at req opcode : %u -  OP CODE : %u\n",ftp_rs.seq_number,ftp_rs.opcode);
                err ++;
                break;
            }
            ftp_rq.seq_number++;

            return ERR_OK;
        }
            break;
        default:
            break;
        }
        
        if (err > 5)
        {
            GSDK_DebugError("Too many error\n");
            return ERR_UNKNOWN;
        }
        usleep(10000);
    }
    
    return ERR_OK;
    
}
// ------------------------------------------------------------------------------
//  Start read thread
// ------------------------------------------------------------------------------
void BootloaderMavlink::start_read_thread(void)
{
    this->read_thread();
}

void BootloaderMavlink::start_write_thread(void)
{
    this->write_thread();
}

void BootloaderMavlink::read_thread(void)
{
    mavlink_message_t message     = { 0 };
    mavlink_status_t mav_status;
    if (_serial_port == nullptr) {
        GSDK_DebugMsg("ERROR: serial port not exist\n");
        throw 1;
    }
    while (!time_to_exit) {
        if (_serial_port->status == SERIAL_PORT_OPEN)
        {
            
            char buf[256] = { 0 };
            int result = _serial_port->read_message(buf, 256);
            for (int i = 0; i < result; ++i) {
                // the parsing
                if (mavlink_parse_char(_channel, (uint8_t)buf[i], &message, &mav_status)) {
                    messages_handler(message);
                }
            }
        }
    }
    usleep(1000);
}

void  BootloaderMavlink::write_thread(void)
{
    uint64_t tnow_ms = 0;
    uint64_t time_send_heartbeat_ms = 0;
    mavlink_message_t message;
    bool have_message = false;
    // Blocking wait for new data
    while (!time_to_exit) {
        if (get_time_msec() - time_send_heartbeat_ms >= 1000) {
            time_send_heartbeat_ms = get_time_msec();
            // write a message and signal writing
            w_heartbeat();

        }
        pthread_mutex_lock(&w_mutex);
        if(w_queue.size() > 0)
        {
            message = w_queue.front();
            w_queue.pop();
            have_message = true;
        }
        pthread_mutex_unlock(&w_mutex);
        if (have_message)
        {
            have_message = false;
            uint16_t len = _serial_port->write_message(message);
        }

        usleep(1000);
    }
}
bool BootloaderMavlink::is_gimbal(uint8_t compid)
{

    switch (compid)
    {
    case MAV_COMP_ID_GIMBAL:
    case MAV_COMP_ID_GIMBAL2:
    case MAV_COMP_ID_GIMBAL3:
    case MAV_COMP_ID_GIMBAL4:
    case MAV_COMP_ID_GIMBAL5:
    case MAV_COMP_ID_GIMBAL6:
        return true;
    break;
    
    default:
        break;
    }
    return false;
}
void BootloaderMavlink::messages_handler(const mavlink_message_t &message)
{
    // if (is_gimbal(message.compid))
    {
        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_HEARTBEAT: 
        {

            pthread_mutex_lock(&_messages.mutex);
            mavlink_msg_heartbeat_decode(&message, &_messages.heartbeat);
            if (message.compid == expect_compid)
            {
                // Store message sysid and compid.
                _gimbal.sysid  = message.sysid;
                _gimbal.compid = message.compid;
                has_detected = true;
            }
            timeslap_hb = get_time_msec();
            pthread_mutex_unlock(&_messages.mutex);
        }
        break;
        case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
        {
            pthread_mutex_lock(&_messages.mutex_ftp);
            mavlink_msg_file_transfer_protocol_decode(&message, &_messages.ftp);
            pthread_cond_signal(&_messages._condition_ftp);
            pthread_mutex_unlock(&_messages.mutex_ftp);
        }
        break;
        case MAVLINK_MSG_ID_SERIAL_CONTROL:
        {
            pthread_mutex_lock(&_messages.mutex_srl);
            mavlink_msg_serial_control_decode(&message, &_messages.sr_ctl);
            pthread_cond_signal(&_messages._condition_srl);
            pthread_mutex_unlock(&_messages.mutex_srl);
        }
        break;
        case MAVLINK_MSG_ID_COMMAND_ACK:
        {
            pthread_mutex_lock(&_messages.mutex_cmd_ack);
            mavlink_msg_command_ack_decode(&message, &_messages.cmd_ack);
            pthread_cond_signal(&_messages._condition_cmd_ack);
            pthread_mutex_unlock(&_messages.mutex_cmd_ack);
        }
        break;
        default:
        break;
        }
    }
}
// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------
void *start_bootloader_read_thread(void *args)
{
    // takes an bootloader object argument
    BootloaderMavlink *boot_loader = (BootloaderMavlink *)args;
    // run the object's read thread
    boot_loader->start_read_thread();
    // done!
    return NULL;
}
void *start_bootloader_write_thread(void *args)
{
    // takes an bootloader object argument
    BootloaderMavlink *boot_loader = (BootloaderMavlink *)args;
    // run the object's read thread
    boot_loader->start_write_thread();
    // done!
    return NULL;
}