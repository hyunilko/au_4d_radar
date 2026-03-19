#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

extern "C" {
#include "PCANBasic.h"
}

#include "pcan_long_frame.hpp"
#include "pcan_short_frame.hpp"

/**
 * @brief Raw CAN FD transport layer.
 *
 * 책임 범위:
 *   - PCAN FD 채널 열기/닫기 (init / shutdown)
 *   - raw CAN FD 프레임 송신 (send_data, send_frame64)
 *   - raw CAN FD 프레임 수신 루프 (start_rx / stop_rx / receiveThread)
 *   - 수신 프레임을 PcanShortFrame / PcanLongFrame 으로 라우팅 (poll_rx)
 *
 * 책임 외 (상위 레이어 담당):
 *   - CustomTP 세그먼트 분할 / 조립  → PcanLongFrame
 *   - Short 커맨드 패킹 / 파싱       → PcanShortFrame
 *   - 비즈니스 로직 처리             → Handler 클래스
 */
class PcanFdTransfer
{
public:
    struct Config
    {
        TPCANHandle handle = PCAN_USBBUS1;

        const char* bitrate_fd =
            "f_clock_mhz=80, "
            "nom_brp=1, nom_tseg1=63, nom_tseg2=16, nom_sjw=16, "
            "data_brp=1, data_tseg1=7, data_tseg2=2, data_sjw=2";

        bool brs_on = true;

        /* Long(CustomTP) frame config: CAN IDs, device count, rx buf size, quiet */
        PcanLongFrameConfig  long_frame{};

        /* Short frame config: CAN IDs, device count, quiet */
        PcanShortFrameConfig short_frame{};
    };

    explicit PcanFdTransfer(const Config& cfg);
    ~PcanFdTransfer();

    /* ----- 채널 초기화 / 해제 -------------------------------------------- */
    bool init(void);
    void shutdown(void);

    /**
     * @brief init() + start_rx() 를 순서대로 호출하는 편의 메쏘드.
     *        au_4d_radar.cpp 에서 핸들러 콜백 등록 완료 후 호출.
     */
    void start(void);

    /* ----- 수신 루프 제어 (transport 계층이 직접 소유) -------------------- */
    /**
     * @brief 수신 스레드 시작.
     *        poll_rx()를 루프하며 수신 프레임을 PcanLongFrame / PcanShortFrame 으로 라우팅.
     *        핸들러의 RX 콜백을 먼저 등록한 뒤 호출해야 한다.
     */
    void start_rx(void);

    /**
     * @brief 수신 스레드 정지 및 join.
     */
    void stop_rx(void);

    /* ----- 프로토콜 레이어 접근자 ---------------------------------------- */
    /**
     * @brief PcanLongFrame 직접 접근.
     *        상위 핸들러가 send_long_payload() / set_rx_callback() 을 직접 호출.
     */
    PcanLongFrame&  long_frame();

    /**
     * @brief PcanShortFrame 직접 접근.
     *        상위 핸들러가 send_short_command_with_data() / set_rx_callback() 을 직접 호출.
     */
    PcanShortFrame& short_frame();

private:
    friend class PcanShortFrame;
    friend class PcanLongFrame;

    /* ----- raw 하드웨어 I/O ---------------------------------------------- */
    static uint8_t len_to_dlc(uint8_t len);
    static uint8_t dlc_to_len(uint8_t dlc);
    static void    print_pcan_err(const char* tag, TPCANStatus st);

    bool send_data(uint16_t can_id, const uint8_t* data, uint8_t length);
    bool send_frame64(uint16_t can_id, const uint8_t data64[64]);

    /* ----- 수신 루프 내부 구현 ------------------------------------------- */
    void poll_rx(void);
    void receiveThread(void);

private:
    Config cfg_;
    bool   initialized_{false};

    std::mutex io_mtx_;

    std::unique_ptr<PcanShortFrame> short_frame_;
    std::unique_ptr<PcanLongFrame>  long_frame_;

    /* 수신 스레드 */
    std::thread       rx_thread_;
    std::atomic<bool> rx_running_{false};
};
