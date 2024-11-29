#include <board/board_dev.h>
#include "timers.h"

#ifndef __BOARD_IO_H
#define __BOARD_IO_H
enum io_event
{
    IO_EventNone = 0U,
    IO_EventRisingEdge,
    IO_EventFallingEdge,
    IO_EventEitherEdge,
    IO_EventLowLevel,
    IO_EventHighLevel,
};

enum io_direction
{
    IO_DigitalInput = 0,
    IO_DigitalOutput,
};

enum io_value
{
    IO_ValueLow = 0U,
    IO_ValueHigh,
};

const uint8_t PTA_wuuPins[] = {
    0U, /* WUU_P0 PTA0 */
    3U, /* WUU_P1 PTA3 */
    4U, /* WUU_P2 PTA4 */
    6U, /* WUU_P3 PTA6 */
    7U, /* WUU_P4 PTA7 */
    8U, /* WUU_P5 PTA8 */
    9U, /* WUU_P6 PTA9 */
    10U, /* WUU_P7 PTA10 */
    11U, /* WUU_P8 PTA11 */
    12U, /* WUU_P9 PTA12 */
    13U, /* WUU_P10 PTA13 */
    14U, /* WUU_P11 PTA14 */
    15U, /* WUU_P12 PTA15 */
    16U, /* WUU_P13 PTA16 */
    17U, /* WUU_P14 PTA17 */
    18U, /* WUU_P15 PTA18 */
    24U, /* WUU_P16 PTA24 */
};

const uint8_t PTB_wuuPins[] = {
    0U, /* WUU_P17 PTB0 */
    1U, /* WUU_P18 PTB1 */
    2U, /* WUU_P19 PTB2 */
    3U, /* WUU_P20 PTB3 */
    4U, /* WUU_P21 PTB4 */
    5U, /* WUU_P22 PTB5 */
    6U, /* WUU_P23 PTB6 */
    12U, /* WUU_P24 PTB12 */
    13U, /* WUU_P25 PTB13 */
    14U, /* WUU_P26 PTB14 */
    15U, /* WUU_P27 PTB15 */
};

struct io_pin {
    uint32_t pinID;
    uint8_t direction;
    uint8_t value;
    enum io_event event;
    bool wakeup;
    bool overridden; /* CA35 pin configuration is overridden by CM33 wakeup pin. */
};

struct io_iface {
    struct dev dev;
    uint8_t ifaceID;
    uint8_t num_pins;
    struct io_pin *io_pins;
    const uint8_t *wuu_pins;
    void (*irqPinHandler)(void *pvParameter1, uint32_t ulParameter2);
};

struct io_adapter;
struct io_ops {
    /**
     * @brief
     * use pin ID to get pin data
     * @param io_iface: ptr to io_iface
     * @param pinID: Pin ID
     * 
     * @return ptr to io_pin or NULL
     */
    struct io_pin *(*get_pin_from_idx)(struct io_iface *io_iface, uint8_t pinID);

    /**
     * @brief
     * use iface ID to get io_iface data
     * @param io_adapter: ptr to io_adapter
     * @param ifaceID: Interface ID
     * 
     * @return ptr to io_iface or NULL
     */
    struct io_iface *(*get_iface_from_idx)(struct io_adapter *io_adapter, uint8_t ifaceID);

    /**
     * @brief
     * set IO Output
     * @param io_adapter: ptr to io_adapter
     * @param ifaceID: Interface ID
     * @param pinID: Pin ID
     * @param value: IO logic Value
     * @return kStatus_Success or Error
     */
    status_t (*set_output)(struct io_adapter *io_adapter,
                    uint8_t ifaceID,
                    uint8_t pinID,
                    enum io_value value);

    /**
     * @brief
     * Get IO Value
     * @param io_adapter: ptr to io_adapter
     * @param ifaceID: Interface ID
     * @param pinID: Pin ID
     * @param value: ptr for IO logic Value
     * @return kStatus_Success or Error
     */
    status_t (*get_input)(struct io_adapter *io_adapter,
                    uint8_t ifaceID,
                    uint8_t pinID,
                    enum io_value *value);

    /**
     * @brief
     * configures Interrupts for IOs and Wakeup Unit
     * @param io_adapter: ptr to io_adapter
     * @param ifaceID: Interface ID
     * @param pinID: Pin ID
     * @param value: Io Event Type
     * @param wakeup: use as wakeup pin
     * @return kStatus_Success or Error
     */
    status_t (*confIRQEvent)(struct io_adapter *io_adapter,
            uint8_t ifaceID,
            uint8_t pinID,
            enum io_event event,
            bool wakeup);

    /**
     * @brief
     * Enables Clock and Interrupts for IO_Interfaces
     * @param io_adapter: ptr to io_adapter
     * @param io_iface: ptr to io_iface
     */
    void (*init_iface)(struct io_adapter *io_adapter,
                struct io_iface *io_iface);
    
    /**
     * @brief
     * Disables Clock and Interrupts
     * @param io_adapter: ptr to io_adapter
     * @param io_iface: ptr to io_iface
     */
    void (*deinit_iface)(struct io_adapter *io_adapter,
                struct io_iface *io_iface);
};

struct io_adapter{
    struct io_iface *io_iface;
    uint8_t num_gpio;
    struct io_ops ops;
};

/**
 * @brief
 * creates IO Description based on board-type
 * @param io_adapter: ptr to io_adapter
 * @param io_devs: ptr to all SoC io_devs
 * @param btype: Board Type
 * @returns 0 or -ERRNO
 */
int init_io_adapter(struct io_adapter *io_adapter, struct dev *io_devs, enum board_types btype);

/**
 * @brief
 * register ISR to IO_Interface
 * @param io_adapter: ptr to io_adapter
 * @param ifaceID: IO Interface ID
 * @param pxCallbackFunction: Callback function for ISR
 * @returns kStatus_Success or Error
 */
status_t IO_RegisterIRQCallback(struct io_adapter *io_adapter,
                uint8_t ifaceID, PendedFunction_t pxCallbackFunction);

/**
 * @brief
 * register ISR to IO_Interface
 * @param io_adapter: ptr to io_adapter
 * @param ifaceID: IO Interface ID
 * @returns kStatus_Success or Error
 */
status_t IO_unregisterIRQCallback(struct io_adapter *io_adapter, uint8_t ifaceID);

#endif /* __BOARD_IO_H */