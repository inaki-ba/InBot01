/***************************************************************************//**
 * @file rail_types.h
 * @brief This file contains the type definitions for RAIL structures, enums,
 *        and other types.
 * @copyright Copyright 2015 Silicon Laboratories, Inc. http://www.silabs.com
 ******************************************************************************/

#ifndef __RAIL_TYPES_H__
#define __RAIL_TYPES_H__

// Include standard type headers to help define structures
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "rail/rail_chip_specific.h"

/**
 * @addtogroup RAIL_API
 * @{
 */

// -----------------------------------------------------------------------------
// Calibration Structures
// -----------------------------------------------------------------------------
/**
 * @addtogroup Calibration
 * @{
 */

/**
 * @typedef RAIL_CalMask_t
 * @brief Calibration mask type
 *
 * This type is a bitmask of different RAIL calibration values. The exact
 * meaning of these bits depends on what your particular chip supports.
 */
typedef uint32_t RAIL_CalMask_t;

/**
 * @struct RAIL_CalInit_t
 * @brief Initialization structure for RAIL calibrations.
 */
typedef struct RAIL_CalInit {
  RAIL_CalMask_t calEnable; /**< Mask that defines calibrations to perform in RAIL. */
  const uint8_t *irCalSettings; /**< Pointer to image rejection calibration settings. */
} RAIL_CalInit_t;

/**
 * @}
 */

// -----------------------------------------------------------------------------
// Radio Configuration Structures
// -----------------------------------------------------------------------------

/**
 * @addtogroup General
 * @{
 */

/**
 * @struct RAIL_Version_t
 * @brief Contains RAIL Library Version Information
 */
typedef struct RAIL_Version {
    uint32_t hash;    /**< Git hash */
    uint8_t  major;   /**< Major number    */
    uint8_t  minor;   /**< Minor number    */
    uint8_t  rev;     /**< Revision number */
    uint8_t  build;   /**< Build number */
    uint8_t  flags;   /**< Build flags */
} RAIL_Version_t;

/**
 * @struct RAIL_Init_t
 * @brief Initialization structure for the RAIL library.
 */
typedef struct RAIL_Init {
  uint16_t maxPacketLength; /**< The maximum number of bytes in a packet. */
  const uint32_t rfXtalFreq; /**< The xtal frequency of the radio. */
  RAIL_CalMask_t calEnable; /**< Mask that defines calibrations to perform in RAIL. */
} RAIL_Init_t;

/**
 * @enum RAIL_PtiProtocol_t
 * @brief The protocol that RAIL outputs via the Packet Trace Interface (PTI)
 */
typedef enum RAIL_PtiProtocol {
  RAIL_PTI_PROTOCOL_CUSTOM = 0, /**< PTI output for a custom protocol */
  RAIL_PTI_PROTOCOL_ZIGBEE = 1, /**< PTI output for the Zigbee protocol */
  RAIL_PTI_PROTOCOL_THREAD = 2, /**< PTI output for the Thread protocol */
  RAIL_PTI_PROTOCOL_BLE = 3, /**< PTI output for the Bluetooth Smart protocol */
  RAIL_PTI_PROTOCOL_CONNECT = 4, /**< PTI output for the Connect protocol */
  RAIL_PTI_PROTOCOL_MAX = 0xF /**< Maximum possible protocol value for PTI */
} RAIL_PtiProtocol_t;

/**
 * @enum RAIL_RadioState_t
 * @brief The current state of the radio
 */
typedef enum RAIL_RadioState {
  RAIL_RF_STATE_IDLE, /**< Radio is idle */
  RAIL_RF_STATE_RX,   /**< Radio is in receive */
  RAIL_RF_STATE_TX,   /**< Radio is in transmit */
} RAIL_RadioState_t;

/**
 * @enum RAIL_Status_t
 * @brief The available status options
 */
typedef enum RAIL_Status {
  RAIL_STATUS_NO_ERROR, /**< RAIL function reports no error */
  RAIL_STATUS_INVALID_PARAMETER, /**< Call to RAIL function errored because of an invalid parameter */
  RAIL_STATUS_INVALID_STATE, /**< Call to RAIL function errored because called during an invalid radio state */
  RAIL_STATUS_INVALID_CALL, /**< The function is called in an invalid order */
} RAIL_Status_t;

/**
 * @enum RAIL_RfSenseBand_t
 * @brief Enumeration for specifying Rf Sense frequency band.
 */
typedef enum {
  RAIL_RFSENSE_OFF,    /**< RFSense is disabled */
  RAIL_RFSENSE_2_4GHZ, /**< RFSense is in 2.4G band */
  RAIL_RFSENSE_SUBGHZ, /**< RFSense is in subgig band */
  RAIL_RFSENSE_ANY,    /**< RfSense is in both bands */
  RAIL_RFSENSE_MAX     // Must be last
} RAIL_RfSenseBand_t;

/**
 * @enum RAIL_RfIdleMode_t
 * @brief Enumeration for the different types of idle modes we support. These
 * vary how quickly and destructively we will put the radio into idle.
 */
typedef enum {
  /**
   * Idle the radio by turning off receive and canceling any future scheduled
   * receive or transmit operations. This will not abort a receive or
   * transmit that is in progress.
   */
  RAIL_IDLE,
  /**
   * Idle the radio by turning off receive and any scheduled events. This will
   * also abort any receive, transmit, or scheduled events in progress.
   */
  RAIL_IDLE_ABORT,
  /**
   * Force the radio into a shutdown mode as quickly as possible. This will
   * abort all current operations and cancel any pending scheduled operations.
   * It may also corrupt receive or transmit buffers and end up clearing them.
   */
  RAIL_IDLE_FORCE_SHUTDOWN
} RAIL_RfIdleMode_t;

/**
 * @}
 */

// -----------------------------------------------------------------------------
// PHY Configuration Structures
// -----------------------------------------------------------------------------

/**
 * @addtogroup Radio_Configuration
 * @{
 */

/**
 * @struct RAIL_StateTiming_t
 * @brief Timing configuration structure for the RAIL State Machine
 *
 * This is used to configure the timings of the radio state transitions for
 * common situations. All of the listed timings are in us. Timing values cannot
 * exceed 13ms. Transitions to IDLE always happen as fast as possible.
 */
typedef struct RAIL_StateTiming {
  uint16_t idleToRx; /**<Transition time from IDLE to RX */
  uint16_t txToRx; /**<Transition time from TX to RX */
  uint16_t idleToTx; /**<Transition time from IDLE to RX */
  uint16_t rxToTx; /**<Transition time from RX to TX */
} RAIL_StateTiming_t;

/**
 * @struct RAIL_FrameType_t
 * @brief Configure if there is a frame type in your frame and the lengths of each frame.
 * The number of bits set in the mask determines the number of elements in frameLen
 * If your packet does not have frame types but instead are of fixed length, set the mask
 * and offset to 0. RAIL will use the value at frameLen to determine the packet length.
 * If each frame type has a different location for the addres, variableAddrLoc should be True.
 */
typedef struct RAIL_FrameType {
  uint8_t offset; /**< Zero-based location of the frame type field in packet. */
  uint8_t mask; /**< Bit mask of the frame type field. Determines number of frames expected. Must be contiguous ones. */
  uint16_t *frameLen; /**< Pointer to array of frame lengths for each frame type. */
  uint8_t *isValid; /**< Pointer to array that marks if each frame is valid or should be filtered. */
  bool variableAddrLoc; /**< If true, address location varies per frame type. */
} RAIL_FrameType_t;

/**
 * @struct RAIL_ChannelConfigEntry_t
 * @brief Channel configuration entry structure. Defines a base frequency and
 *  channel space and the channel indexes that are valid within this range.
 *
 *  * frequency = baseFrequency + channelSpacing * (channel - channelNumberStart);
 */
typedef struct RAIL_ChannelConfigEntry {
  uint16_t channelNumberStart; /**< RAIL Channel number in which this channel set begins.*/
  uint16_t channelNumberEnd; /**< The last valid RAIL channel number for this channel set. */
  uint32_t channelSpacing; /**< Channel spacing in Hz of this channel set. */
  uint32_t baseFrequency; /**< Base frequency in Hz of this channel set. */
} RAIL_ChannelConfigEntry_t;

/**
 * @struct RAIL_ChannelConfig_t
 * @brief Channel configuration structure which defines the channel meaning when
 *  passed into RAIL functions, eg. RAIL_TxStart(), RAIL_RxStart()
 */
typedef struct RAIL_ChannelConfig {
  RAIL_ChannelConfigEntry_t *configs; /**< Pointer to an array of RAIL_ChannelConfigEntry_t entries.*/
  uint32_t length; /**< Number of RAIL_ChannelConfigEntry_t entries. */
} RAIL_ChannelConfig_t;

/**
 * @}
 */

// -----------------------------------------------------------------------------
// Address Filtering Configuration Structures
// -----------------------------------------------------------------------------
/**
 * @addtogroup Address_Filtering
 * @{
 */

/// Default address filtering match table for configurations that use only one
/// address field. The truth table for address matching is below.
///
/// |          | 0000 | 0001 | 0010 | 0100 | 1000 |
/// |----------|------|------|------|------|------|
/// | __0000__ |    0 |    1 |    1 |    1 |    1 |
/// | __0001__ |    1 |    1 |    1 |    1 |    1 |
/// | __0010__ |    1 |    1 |    1 |    1 |    1 |
/// | __0100__ |    1 |    1 |    1 |    1 |    1 |
/// | __1000__ |    1 |    1 |    1 |    1 |    1 |
///
#define ADDRCONFIG_MATCH_TABLE_SINGLE_FIELD (0x1fffffe)
/// Default address filtering match table for configurations that use two
/// address fields and just want to match the same index in each. The truth
/// table for address matching is shown below.
///
/// |          | 0000 | 0001 | 0010 | 0100 | 1000 |
/// |----------|------|------|------|------|------|
/// | __0000__ |    0 |    0 |    0 |    0 |    0 |
/// | __0001__ |    0 |    1 |    0 |    0 |    0 |
/// | __0010__ |    0 |    0 |    1 |    0 |    0 |
/// | __0100__ |    0 |    0 |    0 |    1 |    0 |
/// | __1000__ |    0 |    0 |    0 |    0 |    1 |
#define ADDRCONFIG_MATCH_TABLE_DOUBLE_FIELD (0x1041040)

/**
 * @struct RAIL_AddrConfig_t
 * @brief A structure to configure the address filtering functionality in RAIL.
 */
typedef struct RAIL_AddrConfig {
  /** The number of fields to configure. You cannot have more than 2. */
  uint8_t numFields;

  /**
   * A list of the start offsets for each field
   *
   * These offsets are specified relative to the previous field's end. In the
   * case of the first field it's relative to either the beginning of the packet
   * or the end of the frame type byte if frame type decoding is enabled.
   */
  uint8_t *offsets;

  /**
   * A list of the address field sizes
   *
   * These sizes are specified in bytes and can be from 0 to 8. If you choose a
   * size of 0 this field is effectively disabled.
   */
  uint8_t *sizes;

  /**
   * The truth table to determine how the two fields combine to create a match
   *
   * For detailed information about how this truth table is formed see the
   * detailed description of @ref Address_Filtering.
   *
   * For simple predefined configurations you can use the following defines.
   *  - ADDRCONFIG_MATCH_TABLE_SINGLE_FIELD
   *    - For filtering that only uses a single address field
   *  - ADDRCONFIG_MATCH_TABLE_DOUBLE_FIELD for two field filtering where you
   *    - For filtering that uses two address fields in a configurations where
   *      you want the following logic `((Field0, Index0) && (Field1, Index0)) ||
   *      ((Field0, Index1) && (Field1, Index1)) || ...`
   */
  uint32_t matchTable;
} RAIL_AddrConfig_t;

/**
 * @}
 */

// -----------------------------------------------------------------------------
// System Timing Structures
// -----------------------------------------------------------------------------
/**
 * @addtogroup System_Timing
 * @{
 */

/**
 * @enum RAIL_TimeMode_t
 * @brief Enumeration for specifying timing offsets in RAIL for any APIs that
 * use them.
 */
typedef enum RAIL_TimeMode {
  RAIL_TIME_ABSOLUTE,  /**< The time specified is an exact time in the RAIL timebase */
  RAIL_TIME_DELAY,     /**< The time specified is relative to now */
  RAIL_TIME_DISABLED   /**< The time specified is not intended to be used */
} RAIL_TimeMode_t;

/**
 * @}
 */

// -----------------------------------------------------------------------------
// Pre-Tx Configuration Structures
// -----------------------------------------------------------------------------
/**
 * @addtogroup Pre-Transmit
 * @{
 */

/**
 * @typedef RAIL_PreTxOp_t
 * @brief Generic type used for all configurable pre-transmit operation
 * functions.
 */
typedef uint8_t (*RAIL_PreTxOp_t)(void *params);

/**
 * @struct RAIL_ScheduleTxConfig_t
 * @brief This structure is used to configure the Scheduled Tx algorithm.
 * When using the built-in RAIL_PreTxSchedule() algorithm as your
 * pre-transmit hook within RAIL_TxStart(), an instance of this structure
 * must be passed as its argument.
 */
typedef struct RAIL_ScheduleTxConfig {
  uint32_t when;        /**< When to transmit this packet in the RAIL timebase. */
  RAIL_TimeMode_t mode; /**< Specifies whether when is an absolute time or an offset from now. */
} RAIL_ScheduleTxConfig_t;

/**
 * @def RAIL_MAX_LBT_TRIES
 * @brief The maximum number of LBT/CSMA retries supported
 */
#define RAIL_MAX_LBT_TRIES 15

/**
 * @struct RAIL_CsmaConfig_t
 * @brief This structure is used to configure the CSMA algorithm. When using
 * the built-in RAIL_PreTxCsma() algorithm as your pre-transmit hook within
 * RAIL_TxStart(), an instance of this structure must be passed as its
 * argument.
 */
typedef struct RAIL_CsmaConfig {
  uint8_t  csmaMinBoExp;   /**< Minimum (starting) exponent for CSMA backoff (2^exp - 1) */
  uint8_t  csmaMaxBoExp;   /**< Maximum exponent for CSMA backoff */
  /**
   * Number of CCA failures before report CCA_FAIL. With a maximum value defined
   * in @ref RAIL_MAX_LBT_TRIES). A value of 0 will perform no CCA assessments,
   * and always transmit immediately.
   */
  uint8_t  csmaTries;
  /**
   * The CCA RSSI threshold, in dBm, above which the channel is
   * considered 'busy'.
   */
  int8_t   ccaThreshold;
  /**
   * The backoff unit period, in RAIL's microsecond time base.  This is
   * mulitiplied by the random backoff multiplier controlled by @ref
   * csmaMinBoExp and @ref csmaMaxBoExp to determine the overall backoff
   * period.  This value must be at least the idleToRx time (set by
   * RAIL_SetStateTimings). For random backoffs, any value above 511
   * microseconds will be truncated; for fixed backoffs it can go up to 65535
   * microseconds.
   */
  uint16_t ccaBackoff;
  uint16_t ccaDuration;    /**< CCA check duration, in microseconds */
  /**
   * An overall timeout, in RAIL's microsecond time base, for the operation.  If
   * transmission doesn't start before this timeout expires, the transmission
   * will fail. A value of 0 means no timeout is imposed.
   */
  uint32_t csmaTimeout;
} RAIL_CsmaConfig_t;

/**
 * @def RAIL_CSMA_CONFIG_802_15_4_2003_2p4_GHz_OQPSK_CSMA
 * @brief RAIL_CsmaConfig_t initializer configuring CSMA per 802.15.4-2003
 * on 2.4 GHz OSPSK, commonly used by ZigBee.
 */
#define RAIL_CSMA_CONFIG_802_15_4_2003_2p4_GHz_OQPSK_CSMA {                   \
  /* CSMA per 802.15.4-2003 on 2.4 GHz OSPSK, commonly used by ZigBee      */ \
  /* csmaMinBoExp */   3, /* 2^3-1 for 0..7 backoffs on 1st try            */ \
  /* csmaMaxBoExp */   5, /* 2^5-1 for 0..31 backoffs on 3rd+ tries        */ \
  /* csmaTries    */   5, /* 5 tries overall (4 re-tries)                  */ \
  /* ccaThreshold */ -75, /* 10 dB above sensitivity                       */ \
  /* ccaBackoff   */ 320, /* 20 symbols at 16 us/symbol                    */ \
  /* ccaDuration  */ 128, /* 8 symbols at 16 us/symbol                     */ \
  /* csmaTimeout  */   0, /* no timeout                                    */ \
}

/**
 * @def RAIL_CSMA_CONFIG_SINGLE_CCA
 * @brief RAIL_CsmaConfig_t initializer configuring a single CCA prior to Tx.
 * Can be used to as a basis for implementing other channel access schemes
 * with custom backoff delays.  User can override ccaBackoff with a fixed
 * delay on each use.
 */
#define RAIL_CSMA_CONFIG_SINGLE_CCA {                                         \
  /* Perform a single CCA after 'fixed' delay                              */ \
  /* csmaMinBoExp */   0, /* Used for fixed backoff                        */ \
  /* csmaMaxBoExp */   0, /* Used for fixed backoff                        */ \
  /* csmaTries    */   1, /* Single try                                    */ \
  /* ccaThreshold */ -75, /* Override if not desired choice                */ \
  /* ccaBackoff   */   0, /* No backoff (override with fixed value)        */ \
  /* ccaDuration  */ 128, /* Override if not desired length                */ \
  /* csmaTimeout  */   0, /* no timeout                                    */ \
}

/**
 * @struct RAIL_LbtConfig_t
 * @brief This structure is used to configure the LBT algorithm. When using
 * the built-in RAIL_PreTxLbt() algorithm as your pre-transmit hook within
 * RAIL_TxStart(), an instance of this structure must be passed as its
 * argument.
 */
typedef struct RAIL_LbtConfig {
  uint8_t  lbtMinBoRand;   /**< Minimum backoff random multiplier */
  uint8_t  lbtMaxBoRand;   /**< Maximum backoff random multiplier */
  /**
   * Number of CCA failures before report CCA_FAIL. With a maximum value defined
   * in @ref RAIL_MAX_LBT_TRIES). A value of 0 will perform no CCA assessments,
   * and always transmit immediately.
   */
  uint8_t  lbtTries;       /**< Number of LBT failures before report CCA_FAIL */
  /**
   * The CCA RSSI threshold, in dBm, above which the channel is
   * considered 'busy'.
   */
  int8_t   lbtThreshold;
  /**
   * The backoff unit period, in RAIL's microsecond time base.  This is
   * mulitiplied by the random backoff multiplier controlled by @ref
   * csmaMinBoExp and @ref csmaMaxBoExp to determine the overall backoff
   * period.  For random backoffs, this value must be in the range from
   * idleToRx time (set by RAIL_SetStateTimings) to 511 microseconds; for fixed
   * backoffs it can go up to 65535 microseconds.
   */
  uint16_t lbtBackoff;
  uint16_t lbtDuration;    /**< LBT check duration, in microseconds */
  /**
   * An overall timeout, in RAIL's microsecond time base, for the
   * operation.  If transmission doesn't start before this timeout expires, the
   * transmission will fail. This is important for limiting LBT due to LBT's
   * unbounded requirement that if the channel is busy, the next try must wait
   * for the channel to clear.  A value of 0 means no timeout is imposed.
   */
  uint32_t lbtTimeout;
} RAIL_LbtConfig_t;

/**
 * @def RAIL_LBT_CONFIG_ETSI_EN_300_220_1_V2_4_1
 * @brief RAIL_LbtConfig_t initializer configuring LBT per ETSI 300 220-1
 * V2.4.1 for a typical Sub-GHz band.  To be practical, user should override
 * lbtTries and/or lbtTimeout so channel access failure will be reported in a
 * reasonable timeframe rather than the unbounded timeframe ETSI defined.
 */
#define RAIL_LBT_CONFIG_ETSI_EN_300_220_1_V2_4_1 {                            \
  /* LBT per ETSI 300 220-1 V2.4.1                                         */ \
  /* LBT time = random backoff of 0-5ms in 0.5ms increments plus 5ms fixed */ \
  /* lbtMinBoRand */     0, /*                                             */ \
  /* lbtMaxBoRand */    10, /*                                             */ \
  /* lbtTries     */ RAIL_MAX_LBT_TRIES, /* the maximum supported          */ \
  /* lbtThreshold */   -87, /*                                             */ \
  /* lbtBackoff   */   500, /* 0.5 ms                                      */ \
  /* lbtDuration  */  5000, /* 5 ms                                        */ \
  /* lbtTimeout   */     0, /* no timeout (recommend user override)        */ \
}

/**
 * @}
 */

// -----------------------------------------------------------------------------
// Tx/Rx Configuration Structures
// -----------------------------------------------------------------------------

/**
 * @addtogroup Transmit
 * @{
 */

// Tx Config Callback Defines
/** Callback for a transmit buffer underflow event */
#define RAIL_TX_CONFIG_BUFFER_UNDERFLOW  (0x01 << 1)
/** Callback for CCA/CSMA/LBT failure */
#define RAIL_TX_CONFIG_CHANNEL_BUSY      (0x01 << 2)
/** Callback for when a Tx is aborted by the user */
#define RAIL_TX_CONFIG_TX_ABORTED        (0x01 << 3)
/** Callback for when a Tx is blocked by something like PTA or RHO */
#define RAIL_TX_CONFIG_TX_BLOCKED        (0x01 << 4)

/**
 * @struct RAIL_TxData_t
 * @brief This structure is used to define the data to transmit. The data is copied
 * into an RAIL space buffer so after RAIL_TxLoadData returns, the pointer
 * can be deallocated or reused.
 */
typedef struct RAIL_TxData {
  uint8_t *dataPtr; /**< Pointer to data to transmit */
  uint16_t dataLength; /**< Number of bytes to transmit */
} RAIL_TxData_t;

/**
 * @struct RAIL_TxPacketInfo_t
 * @brief Information about the packet that was just transmitted.
 */
typedef struct RAIL_TxPacketInfo {
  /**
   * Time recorded when the last bit is transmitted out of the modulator.
   */
  uint32_t timeUs;
 } RAIL_TxPacketInfo_t;

/**
 * @struct RAIL_TxOptions_t
 * @brief Tx Option structure that modifies the transmit. Only applies to one
 * transmit.
 */
typedef struct RAIL_TxOptions {
  /**
   * Configure if radio should wait for ack after transmit.  waitForAck is only
   * honored if Auto Ack is enabled and if Auto Ack Tx is not paused
   */
  bool waitForAck;
} RAIL_TxOptions_t;

/**
 * @}
 */

/**
 * @addtogroup Receive
 * @{
 */

// Rx Config Callback Defines
/** Callback for preamble detection */
#define RAIL_RX_CONFIG_PREAMBLE_DETECT   (0x01 << 1)
/** Callback for detection of the first sync word */
#define RAIL_RX_CONFIG_SYNC1_DETECT      (0x01 << 2)
/** Callback for detection of the second sync word */
#define RAIL_RX_CONFIG_SYNC2_DETECT      (0x01 << 3)
/** Callback for detection of frame errors */
#define RAIL_RX_CONFIG_FRAME_ERROR       (0x01 << 4)
/** Callback for when we run out of Rx buffer space */
#define RAIL_RX_CONFIG_BUFFER_OVERFLOW   (0x01 << 5)
/** Callback for when a packet is address filtered */
#define RAIL_RX_CONFIG_ADDRESS_FILTERED  (0x01 << 6)
/** Callback for RF Sensed */
#define RAIL_RX_CONFIG_RF_SENSED         (0x01 << 7)
/** Callback for when an Rx event times out */
#define RAIL_RX_CONFIG_TIMEOUT           (0x01 << 8)
/** Callback for when the scheduled Rx window ends */
#define RAIL_RX_CONFIG_SCHEDULED_RX_END  (0x01 << 9)

/** To maintain backwards compatibility with RAIL 1.1,
 * RAIL_RX_CONFIG_INVALID_CRC is the same as RAIL_RX_CONFIG_FRAME_ERROR
 */
#define RAIL_RX_CONFIG_INVALID_CRC RAIL_RX_CONFIG_FRAME_ERROR

// Rx Config Ignore Error Defines
/** Ignore no errors. Drop all packets with errors */
#define RAIL_IGNORE_NO_ERRORS     (0x00)
/** Ignore CRC errors. Receive packets with CRC errors */
#define RAIL_IGNORE_CRC_ERRORS    (0x01 << 0)
/** Ignore all possible errors. Receive all possible packets */
#define RAIL_IGNORE_ALL_ERRORS    (0xFF)

/** The value returned by RAIL for an invalid RSSI: (-128 * 4) quarter dBm */
#define RAIL_RSSI_INVALID         ((int16_t)(-128 * 4))

/**
 * @struct RAIL_AppendedInfo_t
 * @brief Appended info structure that is returned in the RAILCb_RxPacketReceived
 * callback
 *
 * @todo Define where the rssi latch point is. Is it configurable?
 */
typedef struct RAIL_AppendedInfo {
  /**
   * Timestamp of the received packet in the RAIL timebase of microseconds.
   * This time is recorded at sync detect.
   */
  uint32_t timeUs;
  /**
   * Indicates whether the CRC passed or failed for the receive packet. This
   * will be set to 0 for fail and 1 for pass.
   */
  bool crcStatus:1;
  /**
   * Indicates whether frame coding found any errors in the receive packet.
   * This will be set to 0 for fail and 1 for pass.
   */
  bool frameCodingStatus:1;
  /**
   * Indicates if the received packet is an ack. An 'ack' is defined as a
   * packet received during the rx ack window when autoack is enabled.
   * Set to 0 for not an ack, and 1 for is an ack.
   */
  bool isAck:1;
  /**
   * RSSI of the received packet in integer dBm. This is latched when the sync
   * word is detected for this packet.
   */
  int8_t rssiLatch;
  /**
   * Link quality indicator of the received packet. This is not currently
   * implemented.
   */
  uint8_t lqi;
  /**
   * For radios and PHY configurations that support multiple sync words this
   * number will be the ID of the sync word that was used for this packet.
   */
  uint8_t syncWordId;
} RAIL_AppendedInfo_t;

/**
 * @struct RAIL_RxPacketInfo_t
 * @brief Rx Packet Information structure passed into RAILCb_RxPacketReceived
 *  after a packet has been received. Contains a pointer to the data recieved,
 *  as well as other packet information.
 */
typedef struct RAIL_RxPacketInfo {
  RAIL_AppendedInfo_t appendedInfo; /**< A structure containing various extra information about the received packet. */
  uint16_t dataLength;              /**< The number of bytes in the dataPtr array. */
  uint8_t dataPtr[];                /**< A variable length array holding the packet contents. */
} RAIL_RxPacketInfo_t;

/**
 * @struct RAIL_ScheduleRxConfig_t
 * @brief This structure is used to configure the Scheduled Rx algorithm. It
 * allows you to define the start and end times of the window in either absolute
 * or relative times. If start is set to \ref RAIL_TIME_DISABLED it will be
 * assumed that we should start receive now. If end is set to \ref
 * RAIL_TIME_DISABLED then the only way to end this scheduled receive is with an
 * explicit call to RAIL_RfIdle(). If end is relative it is relative to the
 * start time not the current time. All times are assumed to be specified in the
 * RAIL timebase.
 */
typedef struct RAIL_ScheduleRxConfig {
  /**
   * The time to start receive. See startMode for more information about they
   * types of start times that you can specify.
   */
  uint32_t start;

  /**
   * The type of time value specified in the start parameter. If this is
   * \ref RAIL_TIME_ABSOLUTE then it's an exact time, if it's \ref
   * RAIL_TIME_DELAY then it's an offset relative to the current time. If you
   * specify \ref RAIL_TIME_DISABLED for this then the start event will be
   * ignored.
   */

  RAIL_TimeMode_t startMode;
  /**
   * The time to end receive. See endMode for more information about the types
   * of end times you can specify.
   */
  uint32_t end;
  /**
   * The type of time value specified in the end parameter. If this is
   * \ref RAIL_TIME_ABSOLUTE then it's an exact time, if it's \ref RAIL_TIME_DELAY then
   * it's an offset relative to the start time as long as the startMode isn't
   * \ref RAIL_TIME_DISABLED and if it's \ref RAIL_TIME_DISABLED we will not configure the
   * end event so that this can run indefinitely.
   */
  RAIL_TimeMode_t endMode;
  /**
   * While in scheduled Rx you are still able to control the radio state via
   * state transitions. This option allows you to configure whether a transition
   * to Rx goes back to scheduled Rx or to the normal Rx state. Once in the
   * normal Rx state you will effectively end the scheduled Rx window and can
   * continue to receive indefinitely depending on your state transitions. Set
   * this to 1 to transition to normal Rx and 0 to stay in scheduled Rx.
   */
  uint8_t rxTransitionEndSchedule;
  /**
   * If set to 0 this will allow any packets being received when the window end
   * event occurs to complete. If set to anything else we will force an abort of
   * any packets being received when the window end occurs.
   */
  uint8_t hardWindowEnd;
} RAIL_ScheduleRxConfig_t;

/**
 * @}
 */

/**
 * @addtogroup Auto_Ack
 * @{
 */
/**
 * @struct RAIL_AutoAckConfig_t
 * @brief This structure is used to configure the Auto Ack algorithm. The
 * structure provides a defaultState for the radio to return to once an ack
 * operation occurs (transmitting or attempting to receive an ack). Regardless
 * if the ack operation was successful, the radio will return to the specified
 * default state.
 *
 * The other parameters configure auto ack timing. The application can specify
 * timing from when the radio is idle to TX/RX, the turnaround time from TX->RX
 * and RX->TX, and finally the total amount of time to look for an ack. All of
 * these timing parameters are in microseconds.
 */
typedef struct RAIL_AutoAckConfig {
  /**
   * Default state once auto ack sequence completes or errors. Can only be
   * RAIL_RF_STATE_RX or RAIL_RF_STATE_IDLE.
   */
  RAIL_RadioState_t defaultState;
  /**
   * Define the time from idleToTx and idleToRx in us. Limited to a max of
   * 13ms.
   */
  uint16_t idleTiming;
  /**
   * Define the ack turnaround time in us. Limited to a max of 13ms.
   */
  uint16_t turnaroundTime;
  /**
   * Define the rx ack timeout duration in us. Limited to a max of 65.535ms.
   */
  uint16_t ackTimeout;
} RAIL_AutoAckConfig_t;

/**
 * @struct RAIL_AutoAckData_t
 * @brief This structure is used to define the data to use during auto
 * acknowledgement. The data is copied into an RAIL space buffer so after
 * RAIL_AutoAckLoadBuffer returns, the pointer can be deallocated or reused.
 *
 * Size limited to \ref RAIL_AUTOACK_MAX_LENGTH.
 */
typedef struct RAIL_AutoAckData {
  uint8_t *dataPtr; /**< Pointer to ack data to transmit */
  uint8_t dataLength; /**< Number of ack bytes to transmit */
} RAIL_AutoAckData_t;

/// Acknowledgement packets cannot be longer than 64 bytes.
#define RAIL_AUTOACK_MAX_LENGTH 64
/**
 * @}
 * endofgroup AutoAck
 */
/******************************************************************************
 * Version
 *****************************************************************************/
/**
 * @addtogroup Diagnostic
 * @{
 */

/**
 * @enum RAIL_StreamMode_t
 * @brief Possible stream output modes.
 */
typedef enum RAIL_StreamMode {
  PSEUDO_RANDOM_STREAM, /**< Pseudo random stream of bytes */
  PN9_STREAM            /**< PN9 byte sequence */
} RAIL_StreamMode_t;

/**
 * @struct RAIL_BerConfig_t
 * @brief BER test parameters.
 */
typedef struct RAIL_BerConfig
{
  uint32_t bytesToTest; /**< Number of bytes to test */
} RAIL_BerConfig_t;

/**
 * @struct RAIL_BerStatus_t
 * @brief The status of the latest bit error rate (BER) test.
 */
typedef struct RAIL_BerStatus
{
  uint32_t bitsTotal; /**< Number of bits to receive */
  uint32_t bitsTested; /**< Number of bits currently tested */
  uint32_t bitErrors; /**< Number of bits errors detected */
  int8_t   rssi; /**< Latched RSSI value at pattern detect */
} RAIL_BerStatus_t;

/**
 * @}
 */

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/******************************************************************************
 * Debug
 *****************************************************************************/
/**
 * @addtogroup Debug
 * @{
 */

// Debug Config Callback Defines
/** Callback for radio state change */
#define RAIL_DEBUG_CONFIG_STATE_CHANGE (0x01 << 1)

/**
 * @def RAIL_DEBUG_MODE_FREQ_OVERRIDE
 * @brief A bitmask to enable the frequency override debug mode where you can
 *   manually tune to a specified frequency. Note that this should only be used
 *   for testing and is not as tuned as frequencies from the calculator.
 */
#define RAIL_DEBUG_MODE_FREQ_OVERRIDE  0x00000001UL
/**
 * @def RAIL_DEBUG_MODE_VALID_MASK
 * @brief Any debug mode bits outside of this mask are invalid and ignored.
 */
#define RAIL_DEBUG_MODE_VALID_MASK     (!(RAIL_DEBUG_MODE_FREQ_OVERRIDE))

/**
 * @}
 */
#endif

/**
 * @}
 * end of RAIL_API
 */

#endif  // __RAIL_TYPES_H__
