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
    TimerHandle_t timer; /* GPIO glitch detect timer */
    enum io_event event;
    bool wakeup;
    bool overridden; /* Means the CA35 pin configuration is overridden by CM33 wakeup pin. */
};

struct io_iface {
    struct dev dev;
    uint8_t ifaceID;
    uint8_t num_pins;
    struct io_pin *io_pins;
    const uint8_t *wuu_pins;
    void (*irqPinHandler)(struct io_iface *io_iface, struct io_pin *io_pin);
};

struct io_adapter;
struct io_ops {
    /* returns io_pin obj or NULL */
    struct io_pin *(*get_pin_from_idx)(struct io_iface *io_iface, uint8_t pinID);

    /* returns io_iface obj or NULL */
    struct io_iface *(*get_iface_from_idx)(struct io_adapter *io_adapter, uint8_t ifaceID);

    status_t (*set_output)(struct io_adapter *io_adapter,
                    uint8_t ifaceID,
                    uint8_t pinID,
                    enum io_value value);

    status_t (*get_input)(struct io_adapter *io_adapter,
                    uint8_t ifaceID,
                    uint8_t pinID,
                    enum io_value *value);

    /* Configures Interrupt Events and WUU Pins */
    status_t (*confIRQEvent)(struct io_adapter *io_adapter,
            uint8_t ifaceID,
            uint8_t pinID,
            enum io_event event,
            bool wakeup);

    /* Enables Clock and Interrupts */
    void (*init_iface)(struct io_adapter *io_adapter,
                struct io_iface *io_iface);
    
    /* Disables Clock and Interrupts */
    void (*deinit_iface)(struct io_adapter *io_adapter,
                struct io_iface *io_iface);
};

struct io_adapter{
    struct io_iface *io_iface;
    uint8_t num_gpio;
    struct io_ops ops;
};

int init_io_adapter(struct io_adapter *io_adapter, struct dev *io_devs, enum board_types btype);

void GPIOA_PCore_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin);
void GPIOB_PCore_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin);
void GPIOC_PCore_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin);
void GPIOA_OSM_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin);
void GPIOB_OSM_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin);
void GPIOC_OSM_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin);
void GPIOA_ArmStone_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin);
void GPIOB_ArmStone_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin);
void GPIOC_ArmStone_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin);

#endif /* __BOARD_IO_H */