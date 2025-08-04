// 以下代码以 USBCANFD-200U 设备型号为例
#include "stdafx.h"
#include "zlgcan/zlgcan.h"
#include <iostream>
#include <iomanip>
#include <windows.h>
#include <thread>
#define CH_COUNT 2
bool g_thd_run = 1;
// 此函数仅用于构造示例 CAN 报文
void get_can_frame(ZCAN_Transmit_Data& can_data, canid_t id)
{
    memset(&can_data, 0, sizeof(can_data));
    can_data.frame.can_id = MAKE_CAN_ID(id, 0, 0, 0); // CAN ID + STD/EXT + DATA/RMT
    can_data.frame.can_dlc = 8; // CAN 数据长度 8
    can_data.transmit_type = 0; // 正常发送
    can_data.frame.__pad |= TX_ECHO_FLAG; // 发送回显
    for (int i = 0; i < 8; ++i) { // 填充 CAN 报文 DATA
        can_data.frame.data[i] = i;
    }
}
// 此函数仅用于构造示例 CANFD 报文
void get_canfd_frame(ZCAN_TransmitFD_Data& canfd_data, canid_t id)
{
    memset(&canfd_data, 0, sizeof(canfd_data));
    canfd_data.frame.can_id = MAKE_CAN_ID(id, 0, 0, 0); // CAN ID + STD/EXT + DATA/RMT
    canfd_data.frame.len = 64; // CANFD 数据长度 64
    canfd_data.transmit_type = 0; // 正常发送
    canfd_data.frame.flags |= TX_ECHO_FLAG; // 发送回显
    for (int i = 0; i < 64; ++i) { // 填充 CANFD 报文 DATA
        canfd_data.frame.data[i] = i;
    }
}
// 此函数仅用于构造示例队列发送报文
void get_can_frame_queue(ZCANDataObj& data, int ch, canid_t id, bool is_fd, UINT delay)
{
    memset(&data, 0, sizeof(data));
    data.dataType = ZCAN_DT_ZCAN_CAN_CANFD_DATA;

    data.chnl = ch;
    ZCANCANFDData & can_data = data.data.zcanCANFDData;
    can_data.frame.can_id = MAKE_CAN_ID(id, 0, 0, 0); // CAN ID + STD/EXT + DATA/RMT
    can_data.frame.len = is_fd ? 64 : 8; // 数据长度 8/64
    can_data.flag.unionVal.transmitType = 0; // 正常发送
    can_data.flag.unionVal.txEchoRequest = 1; // 设置发送回显
    can_data.flag.unionVal.frameType = is_fd ? 1 : 0; // CAN or CANFD
    can_data.flag.unionVal.txDelay = ZCAN_TX_DELAY_UNIT_MS; // 队列延时单位毫秒
    can_data.timeStamp = delay; // 队列延时时间，最大值 65535
    for (int i = 0; i < can_data.frame.len; ++i) { // 填充 CAN 报文 DATA
        can_data.frame.data[i] = i;
    }
}
// 此函数仅用于构造示例合并发送报文
void get_can_canfd_frame(ZCANDataObj& data, int ch, canid_t id, bool is_fd)
{
    memset(&data, 0, sizeof(data));
    data.dataType = ZCAN_DT_ZCAN_CAN_CANFD_DATA;
    data.chnl = ch;
    ZCANCANFDData & can_data = data.data.zcanCANFDData;
    can_data.frame.can_id = MAKE_CAN_ID(id,0,0,0); // CAN ID 
    can_data.frame.len = is_fd ? 64:8; // CAN 数据长度 8
    can_data.flag.unionVal.transmitType = 0; // 正常发送
    can_data.flag.unionVal.txEchoRequest = 1; // 设置发送回显
    can_data.flag.unionVal.frameType = is_fd ? 1 : 0; // CAN or CANFD
    can_data.flag.unionVal.txDelay = ZCAN_TX_DELAY_NO_DELAY;// 直接发送报文到总线
    for (int i = 0; i < can_data.frame.len; ++i) { // 填充 CAN 报文 DATA
        can_data.frame.data[i] = i;
    }
}

void thread_task(DEVICE_HANDLE handle)
{
    std::cout << "Thread run, handle:0x" << std::hex << handle << std::endl;
    ZCANDataObj recv_data[100] = {0}; // 定义接收数据缓冲区，100 仅用于举例，根据实际情况定义
    while (g_thd_run) {
        int rcount = ZCAN_ReceiveData(handle, recv_data, 100, 1);
        int lcount = rcount;
        while (g_thd_run && lcount > 0) {
            for (int i = 0; i < rcount; ++i,--lcount) {
                if (recv_data[i].dataType != ZCAN_DT_ZCAN_CAN_CANFD_DATA) { //只处理 CAN 或 CANFD 数据
                    continue;
                }
                std::cout << "CHNL: " << std::dec << (int)recv_data[i].chnl; // 打印通道
                ZCANCANFDData & can_data = recv_data[i].data.zcanCANFDData;
                std::cout << " TIME:" << std::fixed << std::setprecision(6) <<
                can_data.timeStamp/1000000.0f <<"s[" << 
                std::dec << can_data.timeStamp <<"]"; // 打印时间戳
                if (can_data.flag.unionVal.txEchoed == 1) {
                    std::cout << " [TX] "; // 发送帧
                }
                else {
                    std::cout << " [RX] "; // 接收帧
                }
                std::cout << "ID: 0x" << std::hex << can_data.frame.can_id; // 打印 ID
                std::cout << " LEN " << std::dec << (int)can_data.frame.len; // 打印长度
                std::cout << " DATA " << std::hex; // 打印数据
                for (int ind = 0; ind < can_data.frame.len; ++ind) {
                    std::cout << std::hex << " " << (int)can_data.frame.data[ind];
                }
                std::cout << std::endl;
            }
        }
        Sleep(10);
    }
    std::cout << "Thread exit" << std::endl;
}

int _tmain(int argc, _TCHAR* argv[])
{
    UINT dev_type = ZCAN_USBCANFD_200U;
    std::thread thd_handle;
    CHANNEL_HANDLE ch[CH_COUNT] = {};
    // 打开设备
    DEVICE_HANDLE device = ZCAN_OpenDevice(dev_type, 0, 0);

    if (INVALID_DEVICE_HANDLE == device) {
        std::cout << "open device failed!" << std::endl;
        return 0;
    }

    for (int i = 0; i < CH_COUNT; ++i)
    {
        char path[64] = {};

        // 设置 CANFD 标准为 ISO;
        sprintf_s(path, "%d/canfd_standard", i);
        if (0 == ZCAN_SetValue(device, path, "0")) {
            std::cout << "set canfd standard failed" << std::endl;
            goto end;
        }
        // 设置仲裁域波特率为 1M
        sprintf_s(path, "%d/canfd_abit_baud_rate", i);
        if (0 == ZCAN_SetValue(device, path, "1000000")) {
            std::cout << "set abit baud rate failed" << std::endl;
            goto end;
        }
        // 设置通道 0 数据域波特率为 5M
        sprintf_s(path, "%d/canfd_dbit_baud_rate", i);
        if (0 == ZCAN_SetValue(device, path, "5000000")) {
            std::cout << "set dbit baud rate failed" << std::endl;
            goto end;
        }
        // 设置通道 0 自定义波特率，此处仅演示调用方式
        /*
        if (0 == ZCAN_SetValue(device, 
        "0/baud_rate_custom","1.0Mbps(75%),5.0Mbps(75%),(60,0080040D,00800001)")) {
        std::cout << "set custom baud rate failed" << std::endl;
        goto end;
        }
        */
        // 初始化通道
        ZCAN_CHANNEL_INIT_CONFIG config;
        memset(&config, 0, sizeof(config));
        config.can_type = 1; // 0 - CAN，1 - CANFD
        config.can.mode = 0; // 0 - 正常模式，1 - 只听模式
        ch[i] = ZCAN_InitCAN(device, i, &config);
        if (INVALID_CHANNEL_HANDLE == ch[i]) {
            std::cout << "init channel failed!" << std::endl;
            goto end;
        }
        // 使能通道终端电阻
        sprintf_s(path, "%d/initenal_resistance", i);
        if (0 == ZCAN_SetValue(device, path, "1")) {
            std::cout << "enable terminal resistance failed" << std::endl;
            goto end;
        }

        // 设置通道发送超时时间为 100ms
        sprintf_s(path, "%d/tx_timeout", i);
        if (0 == ZCAN_SetValue(device, path, "100")) {
            std::cout << "set send timeout failed" << std::endl;
            goto end;
        }
#if 0
        // 仅对 0 通道设置滤波
        if (0 == i)
        {
            // 设置第一组滤波，只接收 ID 范围在 0x100-0x200 之间的标准帧
            ZCAN_SetValue(device, "0/filter_mode", "0"); // 标准帧
            ZCAN_SetValue(device, "0/filter_start", "0x100"); // 起始 ID
            ZCAN_SetValue(device, "0/filter_end", "0x200"); // 结束 ID
            // 设置第二组滤波，只接收 ID 范围在 0x1FFFF-0x2FFFF 之间的扩展帧
            ZCAN_SetValue(device, "0/filter_mode", "1"); // 扩展帧
            ZCAN_SetValue(device, "0/filter_start", "0x1FFFF"); // 起始 ID
            ZCAN_SetValue(device, "0/filter_end", "0x2FFFF"); // 结束 ID
            // 使能滤波
            ZCAN_SetValue(device, "0/filter_ack", "0");
            // 清除滤波,此处仅举例，何时调用用户自由决定
            // ZCAN_SetValue(device, "0/filter_clear", "0");
        }
#endif
        // 设置合并接收标志，启用合并发送，接收接口（只需设置 1 次）
        if (0 == i) {
            ZCAN_SetValue(device, "0/set_device_recv_merge", "1");
        }

        // 启动 CAN 通道
        if (0 == ZCAN_StartCAN(ch[i])) {
            std::cout << "start channel 0 failed" << std::endl;
            goto end;
        }
    }
    // 启动 CAN 通道的接收线程
    thd_handle = std::thread(thread_task, device);

    // 设置通道 0 自定义序列号为 abc
    ZCAN_SetValue(device, "0/set_cn", "abc");
    const char* pRet = ZCAN_GetValue(device, "0/get_cn/1");
    std::cout << "sn: " << pRet << std::endl;
    #if 0

    /*
    下列代码构造两条定时发送 CAN 报文以及两条定时发送 CANFD 报文，
    索引 0 的 CAN 报文周期 100ms 发送一次，
    索引 1 的 CAN 报文周期 200ms 发送一次，并且延时 1s 启动，
    索引 2 的 CANFD 报文周期 500ms 发送一次，
    索引 3 的 CANFD 报文周期 600ms 发送一次，
    发送 5s 后停止发送
    */
    ZCAN_AUTO_TRANSMIT_OBJ auto_can;
    ZCANFD_AUTO_TRANSMIT_OBJ auto_canfd;
    ZCAN_AUTO_TRANSMIT_OBJ_PARAM delay_param;
    memset(&auto_can, 0, sizeof(auto_can));
    auto_can.index = 0; // 定时列表索引 0
    auto_can.enable = 1; // 使能此索引，每条可单独设置
    auto_can.interval = 100; // 定时发送间隔 100ms
    get_can_frame(auto_can.obj, 0); // 构造 CAN 报文
    ZCAN_SetValue(device, "0/auto_send", (const char*)&auto_can); // 设置定时发送
    memset(&auto_can, 0, sizeof(auto_can));
    auto_can.index = 1; // 定时列表索引 1
    auto_can.enable = 1; // 使能此索引，每条可单独设置
    auto_can.interval = 200; // 定时发送间隔 200ms
    get_can_frame(auto_can.obj, 1); // 构造 CAN 报文
    ZCAN_SetValue(device, "0/auto_send", (const char*)&auto_can); // 设置定时发送
    memset(&auto_canfd, 0, sizeof(auto_canfd));
    auto_canfd.index = 2; // 定时列表索引 2
    auto_canfd.enable = 1; // 使能此索引，每条可单独设置
    auto_canfd.interval = 500; // 定时发送间隔 500ms
    get_canfd_frame(auto_canfd.obj, 2); // 构造 CANFD 报文
    ZCAN_SetValue(device, "0/auto_send_canfd", (const char*)&auto_canfd); // 设置定时发送
    memset(&auto_canfd, 0, sizeof(auto_canfd));
    auto_canfd.index = 3; // 定时列表索引 3
    auto_canfd.enable = 1; // 使能此索引，每条可单独设置
    auto_canfd.interval = 600; // 定时发送间隔 600ms
    get_canfd_frame(auto_canfd.obj, 3); // 构造 CANFD 报文
    ZCAN_SetValue(device, "0/auto_send_canfd", (const char*)&auto_canfd); // 设置定时发送
    // 设置索引 1 的 CAN 帧延时 1s 启动发送
    delay_param.index = 1; // 索引 1
    delay_param.type = 1; // type 为 1 表示延时启动
    delay_param.value = 1000; // 延时 1000ms
    ZCAN_SetValue(device, "0/auto_send_param", (const char*)&delay_param); // 设置发送延时
    ZCAN_SetValue(device, "0/apply_auto_send", "0"); // 使能定时发送
    Sleep(5000); // 等待发送 5s
    ZCAN_SetValue(device, "0/clear_auto_send", "0"); // 清除定时发送
    system("pause");

    #endif
    #if 1
    // 设置队列发送模式
    ZCAN_SetValue(device, "0/set_send_mode", "1");
    // 获取队列发送可用缓冲
    int free_count = *((int*) ZCAN_GetValue(device, "0/get_device_available_tx_count/1"));
    // 构造 50 条报文，报文 ID 从 0 递增，帧间隔 10ms 递增
    if (free_count > 50) {
        ZCANDataObj tran_data[50] = {};
        for (int i = 0; i < 50; ++i) {
            get_can_frame_queue(tran_data[i], 0, i, i%2 ? true : false, i*10);
        }
        int ret_count = ZCAN_TransmitData(device, tran_data, 50);
    }
    // 5 秒后清空队列发送
    Sleep(5000);
    ZCAN_SetValue(device, "0/clear_delay_send_queue", "0");
    system("pause");
#endif
    // 构造 10 帧 CAN 报文(0 通道发送)以及 10 帧 CANFD 报文（1 通道发送）
    ZCANDataObj trans_data[20] = {};
    for (int i = 0; i < 20; ++i)
    {
        int ch = i < 10 ? 0 : 1;
        bool is_fd = i < 10 ? false : true;
        get_can_canfd_frame(trans_data[i], ch, i + 0x100, is_fd);
    }
    int send_count = ZCAN_TransmitData(device, trans_data, 20);
    std::cout << "send frame: " << std::dec << send_count << std::endl;
    Sleep(500);
    system("pause");
    end:
    g_thd_run = 0;
    if (thd_handle.joinable())
        thd_handle.join();
    std::cout << "Thread exited, close device" << std::endl;
    if (INVALID_DEVICE_HANDLE != device)
        ZCAN_CloseDevice(device);
    system("pause");
    return 0;
}
