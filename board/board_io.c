/*
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <board/board.h>
#include <asm-generic/errno.h>
#include "fsl_common.h"
#include "fsl_rgpio.h"
#include "fsl_wuu.h"

static struct io_iface *get_iface_from_idx(struct io_adapter *io_adapter, uint8_t ifaceID)
{
    struct io_iface *io_iface;
    int i;

    io_iface = io_adapter->io_iface;

    for(i = 0; i < io_adapter->num_gpio; i++){
        if(io_iface[i].dev.instance == ifaceID)
            break;
    }

    if(i >= io_adapter->num_gpio)
        return NULL;

    return &io_iface[i];
}

static struct io_pin *get_pin_from_idx(struct io_iface *io_iface, uint8_t pinID)
{
    struct io_pin *io_pin;
    int i;

    io_pin = io_iface->io_pins;
    for(i = 0; i < io_iface->num_pins; i++){
        if(io_pin[i].pinID == pinID)
            break;
    }

    if(i >= io_iface->num_pins)
        return NULL;

    return &io_pin[i];
}

static void set_output(struct io_iface *io_iface, struct io_pin *io_pin)
{
    struct _rgpio_pin_config pin_config;

    if(io_pin->direction != IO_DigitalOutput){
        io_pin->direction = IO_DigitalOutput;
        pin_config.pinDirection = IO_DigitalOutput;
        pin_config.outputLogic = io_pin->value;
        RGPIO_PinInit((RGPIO_Type *)io_iface->dev.base_addr, io_pin->pinID, &pin_config);
    }

    RGPIO_PinWrite((RGPIO_Type *)io_iface->dev.base_addr, io_pin->pinID, io_pin->value);
}

static uint32_t get_input(struct io_iface *io_iface, struct io_pin *io_pin)
{
    struct _rgpio_pin_config pin_config;

    if(io_pin->direction != IO_DigitalInput){
        io_pin->direction = IO_DigitalInput;
        pin_config.pinDirection = IO_DigitalInput;
        RGPIO_PinInit((RGPIO_Type *)io_iface->dev.base_addr, io_pin->pinID, &pin_config);
    }

    return RGPIO_PinRead((RGPIO_Type *)io_iface->dev.base_addr, io_pin->pinID);
}

status_t BOARD_IO_set_output(struct io_adapter *io_adapter,
                    uint8_t ifaceID,
                    uint8_t pinID,
                    enum io_value value)
{
    struct io_iface *io_iface;
    struct io_pin *io_pin;

    /* get Pin Objects */
    io_iface = get_iface_from_idx(io_adapter, ifaceID);
    if(!io_iface)
        return kStatus_NoData;
    io_pin = get_pin_from_idx(io_iface, pinID);
    if(!io_pin)
        return kStatus_NoData;

    /* set Output Val */
    io_pin->value = value;
    set_output(io_iface, io_pin);

    return kStatus_Success;
}

status_t BOARD_IO_get_input(struct io_adapter *io_adapter,
                    uint8_t ifaceID,
                    uint8_t pinID,
                    enum io_value *value)
{
    struct io_iface *io_iface;
    struct io_pin *io_pin;

    /* get Pin Objects */
    io_iface = get_iface_from_idx(io_adapter, ifaceID);
    if(!io_iface)
        return kStatus_NoData;
    io_pin = get_pin_from_idx(io_iface, pinID);
    if(!io_pin)
        return kStatus_NoData;

    /* return Input val */
    io_pin->value = get_input(io_iface, io_pin);
    if(value)
        *value = io_pin->value;
        
    return kStatus_Success;
}

static void init_iface(struct io_adapter *io_adapter, struct io_iface *io_iface)
{
    struct dev *dev = &io_iface->dev;

    CLOCK_EnableClock(dev->ip_name);

    switch(dev->base_addr){
        case GPIOA_BASE:
            NVIC_SetPriority(GPIOA_INT0_IRQn, CONFIG_GPIO_INT_PRIO);
            NVIC_SetPriority(GPIOA_INT1_IRQn, CONFIG_GPIO_INT_PRIO);
            EnableIRQ(GPIOA_INT0_IRQn);
            EnableIRQ(GPIOA_INT1_IRQn);
            break;
        case GPIOB_BASE:
            NVIC_SetPriority(GPIOB_INT0_IRQn, CONFIG_GPIO_INT_PRIO);
            NVIC_SetPriority(GPIOB_INT1_IRQn, CONFIG_GPIO_INT_PRIO);
            EnableIRQ(GPIOB_INT0_IRQn);
            EnableIRQ(GPIOB_INT1_IRQn);
            break;
        case GPIOC_BASE:
            NVIC_SetPriority(GPIOC_INT0_IRQn, CONFIG_GPIO_INT_PRIO);
            NVIC_SetPriority(GPIOC_INT1_IRQn, CONFIG_GPIO_INT_PRIO);
            EnableIRQ(GPIOC_INT0_IRQn);
            EnableIRQ(GPIOC_INT1_IRQn);
            break;
        default:
            break;
    }
}

static void deinit_iface(struct io_adapter *io_adapter, struct io_iface *io_iface)
{
    struct dev *dev = &io_iface->dev;

        switch(dev->base_addr){
        case GPIOA_BASE:
            DisableIRQ(GPIOA_INT0_IRQn);
            DisableIRQ(GPIOA_INT1_IRQn);
            break;
        case GPIOB_BASE:
            DisableIRQ(GPIOB_INT0_IRQn);
            DisableIRQ(GPIOB_INT1_IRQn);
            break;
        case GPIOC_BASE:
            DisableIRQ(GPIOC_INT0_IRQn);
            DisableIRQ(GPIOC_INT1_IRQn);
            break;
        default:
            break;
    }

    CLOCK_DisableClock(dev->ip_name);
}

static int __get_wuuIdx(struct io_iface *io_iface, struct io_pin *io_pin, uint8_t *wuuIdx)
{
    struct dev *dev = &io_iface->dev;
    const uint8_t *wuuPins = io_iface->wuu_pins;
    int i;
    uint8_t idx = 0;
    int array_size = 0;

    if(!io_iface->wuu_pins)
        return -EINVAL;
    
    if(dev->base_addr == GPIOA_BASE) 
        array_size = ARRAY_SIZE(PTA_wuuPins);
    else if(dev->base_addr == GPIOB_BASE){
        array_size = ARRAY_SIZE(PTB_wuuPins);
        idx = ARRAY_SIZE(PTA_wuuPins);
    }

    for(i = 0; i < array_size; i++){
        if(wuuPins[i] == io_pin->pinID)
            break;
    }

    if(i >= array_size)
        return -EINVAL;
    
    idx += i;
    *wuuIdx = idx;

    return 0;    
}

static void __confIRQEvent(struct io_iface *io_iface, struct io_pin *io_pin)
{
    struct dev *dev = &io_iface->dev;
    enum io_event event = io_pin->event;
    bool wakeup = io_pin->wakeup;
    wuu_external_wakeup_pin_config_t wuu_config;
    uint8_t wuuIdx;
    int ret;
    
    switch(event){
        case IO_EventRisingEdge:
            RGPIO_SetPinInterruptConfig((RGPIO_Type *)dev->base_addr, io_pin->pinID, CONFIG_RGPIO_INTERRUPT_SEL, kRGPIO_InterruptRisingEdge);
            if(wakeup){
                wuu_config.edge = kWUU_ExternalPinRisingEdge;
                ret = __get_wuuIdx(io_iface, io_pin, &wuuIdx);
                if(ret)
                    break;
                WUU_SetExternalWakeUpPinsConfig(WUU0, wuuIdx, &wuu_config);
            }
            break;
        case IO_EventFallingEdge:
            RGPIO_SetPinInterruptConfig((RGPIO_Type *)dev->base_addr, io_pin->pinID, CONFIG_RGPIO_INTERRUPT_SEL, kRGPIO_InterruptFallingEdge);
            if(wakeup){
                wuu_config.edge = kRGPIO_InterruptFallingEdge;
                ret = __get_wuuIdx(io_iface, io_pin, &wuuIdx);
                if(ret)
                    break;
                WUU_SetExternalWakeUpPinsConfig(WUU0, wuuIdx, &wuu_config);
            }
            break;
        case IO_EventEitherEdge:
            RGPIO_SetPinInterruptConfig((RGPIO_Type *)dev->base_addr, io_pin->pinID, CONFIG_RGPIO_INTERRUPT_SEL, kRGPIO_InterruptEitherEdge);
            if(wakeup){
                wuu_config.edge = kWUU_ExternalPinAnyEdge;
                ret = __get_wuuIdx(io_iface, io_pin, &wuuIdx);
                if(ret)
                    break;
                WUU_SetExternalWakeUpPinsConfig(WUU0, wuuIdx, &wuu_config);
            }
            break;
        case IO_EventLowLevel:
            RGPIO_SetPinInterruptConfig((RGPIO_Type *)dev->base_addr, io_pin->pinID, CONFIG_RGPIO_INTERRUPT_SEL, kRGPIO_InterruptLogicZero);
            /* LOW level cannot trigger wakeup */
            break;
        case IO_EventHighLevel:
            RGPIO_SetPinInterruptConfig((RGPIO_Type *)dev->base_addr, io_pin->pinID, CONFIG_RGPIO_INTERRUPT_SEL, kRGPIO_InterruptLogicOne);
            /* LOW level cannot trigger wakeup */
            break;
        default:
            RGPIO_SetPinInterruptConfig((RGPIO_Type *)dev->base_addr, io_pin->pinID, CONFIG_RGPIO_INTERRUPT_SEL, kRGPIO_InterruptOrDMADisabled);
            break;
    }
    if(!wakeup && !__get_wuuIdx(io_iface, io_pin, &wuuIdx)){
        wuu_config.edge = kWUU_ExternalPinDisable;
        WUU_SetExternalWakeUpPinsConfig(WUU0, wuuIdx, &wuu_config);
    }
}

status_t BOARD_IO_confIRQEvent(struct io_adapter *io_adapter,
        uint8_t ifaceID,
        uint8_t pinID,
        enum io_event event,
        bool wakeup)
{
    struct io_iface *io_iface;
    struct io_pin *io_pin;

    io_iface = get_iface_from_idx(io_adapter, ifaceID);
    if(!io_iface)
        return kStatus_NoData;

    io_pin = get_pin_from_idx(io_iface, pinID);
    if(!io_pin)
        return kStatus_NoData;

    io_pin->event = event;
    io_pin->wakeup = wakeup;
    __confIRQEvent(io_iface, io_pin);
    return kStatus_Success;
}

/* TODO:  */
#ifdef CONFIG_BOARD_PICOCOREMX8ULP
void GPIOA_PCore_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin)
{
    BaseType_t reschedule = pdFALSE;


    switch(io_pin->pinID){
        default:
            break;
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

void GPIOB_PCore_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin)
{
    BaseType_t reschedule = pdFALSE;


    switch(io_pin->pinID){
        default:
            break;
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

void GPIOC_PCore_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin)
{
    BaseType_t reschedule = pdFALSE;


    switch(io_pin->pinID){
        default:
            break;
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}
#endif /* CONFIG_BOARD_PICOCOREMX8ULP */

/* TODO:  */
#ifdef CONFIG_BOARD_OSMSFMX8ULP
void GPIOA_OSM_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin)
{
    BaseType_t reschedule = pdFALSE;


    switch(io_pin->pinID){
        default:
            break;
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

void GPIOB_OSM_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin)
{
    BaseType_t reschedule = pdFALSE;


    switch(io_pin->pinID){
        default:
            break;
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

void GPIOC_OSM_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin)
{
    BaseType_t reschedule = pdFALSE;


    switch(io_pin->pinID){
        default:
            break;
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}
#endif /* CONFIG_BOARD_OSMSFMX8ULP */

/* TODO:  */
#ifdef CONFIG_BOARD_ARMSTONEMX8ULP
void GPIOA_ArmStone_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin)
{
    BaseType_t reschedule = pdFALSE;


    switch(io_pin->pinID){
        default:
            break;
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

void GPIOB_ArmStone_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin)
{
    BaseType_t reschedule = pdFALSE;


    switch(io_pin->pinID){
        default:
            break;
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

void GPIOC_ArmStone_IRQPinHandler(struct io_iface *io_iface, struct io_pin *io_pin)
{
    BaseType_t reschedule = pdFALSE;


    switch(io_pin->pinID){
        default:
            break;
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}
#endif /* CONFIG_BOARD_ARMSTONEMX8ULP */

void IO_IRQHandler(long GPIO_BASE, long RGPIO_INT_SEL)
{
    struct board_descr *bdescr = get_board_description();
    struct io_adapter *io_adapter = &bdescr->io_adapter;
    struct io_iface *io_iface = NULL;
    unsigned int iflags;
    int i;

    /* Get IO Interface */
    for(i = 0; i < io_adapter->num_gpio; i++){
        if(io_adapter->io_iface[i].dev.base_addr == GPIO_BASE){
            io_iface = &io_adapter->io_iface[i];
            break;
        }
    }

    if(!io_iface)
        return;

    iflags = (uint32_t)RGPIO_GetPinsInterruptFlags((RGPIO_Type *)GPIO_BASE, RGPIO_INT_SEL);

    /* Clear Interrupt Flag and call PinHandler */
    for(i = 0; iflags != 0; i++, iflags = iflags >> 1){
        if(!(iflags & 0x1))
            continue;

        RGPIO_ClearPinsInterruptFlags((RGPIO_Type *)GPIO_BASE, RGPIO_INT_SEL, 1<<i);
        if(io_iface->irqPinHandler)
            io_iface->irqPinHandler(io_iface, &io_iface->io_pins[i]);
    }
}

void GPIOA_INT0_IRQHandler(void)
{
    IO_IRQHandler(GPIOA_BASE, kRGPIO_InterruptOutput1);
}

void GPIOA_INT1_IRQHandler(void)
{
    IO_IRQHandler(GPIOA_BASE, kRGPIO_InterruptOutput2);
}

void GPIOB_INT0_IRQHandler(void)
{
    IO_IRQHandler(GPIOB_BASE, kRGPIO_InterruptOutput1);
}

void GPIOB_INT1_IRQHandler(void)
{
    IO_IRQHandler(GPIOB_BASE, kRGPIO_InterruptOutput2);
}

void GPIOC_INT0_IRQHandler(void)
{
    IO_IRQHandler(GPIOC_BASE, kRGPIO_InterruptOutput1);
}

void GPIOC_INT1_IRQHandler(void)
{
    IO_IRQHandler(GPIOC_BASE, kRGPIO_InterruptOutput2);
}

static int init_io_iface(struct io_iface *io_iface, struct dev *io_dev, enum board_types btype)
{
    struct io_pin *io_pins = NULL;
    int i;

    memcpy(&io_iface->dev, io_dev, sizeof(struct dev));

    switch(io_iface->dev.base_addr) {
        case GPIOA_BASE:
            io_pins = pvPortMalloc(sizeof(struct io_pin) * GPIOA_NUM);
            if(!io_pins)
                return -ENOMEM;        

            io_iface->num_pins = GPIOA_NUM;
            io_iface->io_pins = io_pins;
            io_iface->wuu_pins = PTA_wuuPins;
            io_iface->ifaceID = CONFIG_GPIOA_IFACEID;
            if(btype == BT_PICOCOREMX8ULP)
                io_iface->irqPinHandler = &GPIOA_PCore_IRQPinHandler;
            if(btype == BT_OSMSFMX8ULP)
                io_iface->irqPinHandler = &GPIOA_OSM_IRQPinHandler;
            if(btype == BT_ARMSTONEMX8ULP)
                io_iface->irqPinHandler = &GPIOA_ArmStone_IRQPinHandler;
            break;
        case GPIOB_BASE:
            io_pins = pvPortMalloc(sizeof(struct io_pin) * GPIOB_NUM);
            if(!io_pins)
                return -ENOMEM;        

            io_iface->num_pins = GPIOB_NUM;
            io_iface->io_pins = io_pins;
            io_iface->wuu_pins = PTB_wuuPins;
            io_iface->ifaceID = CONFIG_GPIOB_IFACEID;
            if(btype == BT_PICOCOREMX8ULP)
                io_iface->irqPinHandler = &GPIOB_PCore_IRQPinHandler;
            if(btype == BT_OSMSFMX8ULP)
                io_iface->irqPinHandler = &GPIOB_OSM_IRQPinHandler;
            if(btype == BT_ARMSTONEMX8ULP)
                io_iface->irqPinHandler = &GPIOB_ArmStone_IRQPinHandler;
            break;
        case GPIOC_BASE:
            io_pins = pvPortMalloc(sizeof(struct io_pin) * GPIOC_NUM);
            if(!io_pins)
                return -ENOMEM;        

            io_iface->num_pins = GPIOC_NUM;
            io_iface->io_pins = io_pins;
            io_iface->wuu_pins = NULL;
            io_iface->ifaceID = CONFIG_GPIOC_IFACEID;
            if(btype == BT_PICOCOREMX8ULP)
                io_iface->irqPinHandler = &GPIOC_PCore_IRQPinHandler;
            if(btype == BT_OSMSFMX8ULP)
                io_iface->irqPinHandler = &GPIOC_OSM_IRQPinHandler;
            if(btype == BT_ARMSTONEMX8ULP)
                io_iface->irqPinHandler = &GPIOC_ArmStone_IRQPinHandler;
            break;
    }

    for (i = 0; i < io_iface->num_pins; i++){
        io_pins[i].pinID = i;
        io_pins[i].event = IO_EventNone;
        io_pins[i].overridden = false;
        io_pins[i].value = 0;
        io_pins[i].wakeup = false;
    }

    return 0;
}

int init_io_adapter(struct io_adapter *io_adapter, struct dev *io_devs, enum board_types btype)
{
    struct io_iface *io_iface;
    int ret;

    io_iface = pvPortMalloc(sizeof(struct io_iface) * 3);
    if(!io_iface)
        return -ENOMEM;

    memset(io_iface, 0, sizeof(struct io_iface) * 3);
    
    io_adapter->num_gpio = 3;
    io_adapter->io_iface = io_iface;

    /* GPIO_A */
    ret = init_io_iface(&io_iface[0], &io_devs[0], btype);
    if(ret){
        vPortFree(io_iface);
        return ret;
    }
    
    /* GPIO_B */
    ret = init_io_iface(&io_iface[1], &io_devs[1], btype);
    if(ret){
        vPortFree(io_iface);
        return ret;
    }

    /* GPIO_C */
    ret = init_io_iface(&io_iface[2], &io_devs[2], btype);
    if(ret){
        vPortFree(io_iface);
        return ret;
    }

    /* init ops */
    io_adapter->ops.set_output = &BOARD_IO_set_output;
    io_adapter->ops.get_input = &BOARD_IO_get_input;
    io_adapter->ops.get_iface_from_idx = &get_iface_from_idx;
    io_adapter->ops.get_pin_from_idx = &get_pin_from_idx;
    io_adapter->ops.confIRQEvent = &BOARD_IO_confIRQEvent;
    io_adapter->ops.init_iface = &init_iface;
    io_adapter->ops.deinit_iface = &deinit_iface;
    return 0;
}