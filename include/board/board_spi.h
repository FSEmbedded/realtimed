#ifndef BOARD_SPI_H
#define BOARD_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include <board/board_dev.h>
#include "fsl_debug_console.h"
#include "fsl_lpspi.h"


/* Description of an SPI instance */
struct spi_iface {
    uint8_t id;                // user defined Iface ID
    struct dev dev;
    volatile bool transfer_done;
    lpspi_master_config_t config;   // SPI configuration
    lpspi_master_handle_t handle;   // driver Handle
};

struct spi_adapter;
/* SPI Methods */
struct spi_ops {
    /**
     * @brief initialise SPI bus
     *
     * @return true on success, false otherwise
     */
    void (*init)(struct spi_adapter *spi_adapter, struct spi_iface *spi_iface);
    /**
     * @brief deinitialise SPI bus
     */
    void (*deinit)(struct spi_adapter *spi_adapter, struct spi_iface *spi_iface);
    /**
     * @brief write data to SPI bus
     * @brief transfer data to/from SPI bus
     *
     * @param tx_buf: pointer to data buffer to send
     * @param rx_buf: pointer to data buffer to receive
     * @param len: length of data to transfer
     * @return true on success, false otherwise
     */
    status_t (*transfer)(struct spi_adapter *spi_adapter, struct spi_iface *spi_iface, const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len, uint8_t flags);
    /**
     * @brief get iface from index
     * @param spi_adapter: spi adapter description
     * @param ifaceId: iface index
     * @return pointer to spi_iface or NULL if not found
     */
    struct spi_iface *(*get_iface_from_idx)(struct spi_adapter *spi_adapter, uint8_t ifaceId);
    /**
     * @brief configure spi interface
     * @param spi_adapter: spi adapter description
     * @param spi_iface: spi interface to configure
     * @param flags: flags contains SPI-Mode
     * @param speed_hz: transfer speed in Hz
     */
    void (*configure)(struct spi_adapter *spi_adapter, struct spi_iface *spi_iface, uint8_t flags, uint32_t speed_hz);
};

struct spi_adapter {
    struct spi_iface *spi_iface;
    uint32_t num_iface;
    struct spi_ops ops;
};

/**
 * @brief initialise spi adapter structure
 * @param spi_adapter: adapter to initialise
 * @param spi_devs: array of spi device descriptions
 * @param btype: board type to determine which spi devices to initialise
 */
int init_spi_adapter(struct spi_adapter *spi_adapter, struct dev *spi_devs, enum board_types btype);

#endif // BOARD_SPI_H
