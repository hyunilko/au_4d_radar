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
 * Responsibilities:
 *   - Open / close the PCAN FD channel (init / shutdown)
 *   - Transmit raw CAN FD frames (send_data, send_frame64)
 *   - Drive the receive loop (start_rx / stop_rx / receiveThread)
 *   - Route received frames to PcanShortFrame / PcanLongFrame (poll_rx)
 *
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

        /* Short(Command) frame config: CAN IDs, device count, quiet */
        PcanShortFrameConfig short_frame{};
    };

    explicit PcanFdTransfer(const Config& cfg);
    ~PcanFdTransfer();

    /* ----- Channel initialisation / release -------------------------------- */
    bool init(void);
    void shutdown(void);

    /**
     * @brief Convenience method that calls init() then start_rx() in order.
     *        Call this after all handler callbacks have been registered in au_4d_radar.cpp.
     */
    void start(void);

    /* ----- Receive loop control (owned by the transport layer) ------------ */
    /**
     * @brief Starts the receive thread.
     *        Loops poll_rx() to route incoming frames to PcanLongFrame / PcanShortFrame.
     *        Must be called after all handler RX callbacks have been registered.
     */
    void start_rx(void);

    /**
     * @brief Stops the receive thread and joins it.
     */
    void stop_rx(void);

    /* ----- Protocol layer accessors ---------------------------------------- */
    /**
     * @brief Direct access to the owned PcanLongFrame instance.
     *        Upper-layer handlers call send_long_payload() / set_rx_callback() directly.
     */
    PcanLongFrame&  long_frame();

    /**
     * @brief Direct access to the owned PcanShortFrame instance.
     *        Upper-layer handlers call send_short_command_with_data() / set_rx_callback() directly.
     */
    PcanShortFrame& short_frame();

    /* ----- Raw CAN FD frame I/O -------------------------------------------- */
    /**
     * @brief Sends a CAN-FD frame with a variable-length payload (up to 64 bytes).
     *        Called by PcanShortFrame to transmit short commands.
     */
    bool send_data(uint16_t can_id, const uint8_t* data, uint8_t length);

    /**
     * @brief Sends a fixed 64-byte CAN-FD frame (DLC = 15).
     *        Called by PcanLongFrame to transmit CustomTP chunks.
     */
    bool send_frame64(uint16_t can_id, const uint8_t data64[64]);

private:
    /* ----- Internal hardware helpers --------------------------------------- */
    static uint8_t len_to_dlc(uint8_t len);
    static uint8_t dlc_to_len(uint8_t dlc);
    static void    print_pcan_err(const char* tag, TPCANStatus st);

    /* ----- Internal receive loop ------------------------------------------ */
    void poll_rx(void);
    void receiveThread(void);

private:
    Config cfg_;
    bool   initialized_{false};

    std::mutex io_mtx_;

    std::unique_ptr<PcanShortFrame> short_frame_;
    std::unique_ptr<PcanLongFrame>  long_frame_;

    /* Receive thread */
    std::thread       rx_thread_;
    std::atomic<bool> rx_running_{false};
};
