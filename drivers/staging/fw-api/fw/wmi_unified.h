/*
 * Copyright (c) 2010-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

/**
 * @addtogroup WMIAPI
 *@{
 */

/** @file
 * This file specifies the WMI interface for the  Software Architecture.
 *
 * It includes definitions of all the commands and events. Commands are messages
 * from the host to the target. Events and Replies are messages from the target
 * to the host.
 *
 * Ownership of correctness in regards to WMI commands
 * belongs to the host driver and the target is not required to validate
 * parameters for value, proper range, or any other checking.
 *
 * Guidelines for extending this interface are below.
 *
 * 1. Add new WMI commands ONLY within the specified range - 0x9000 - 0x9fff
 * 2. Use ONLY A_UINT32 type for defining member variables within WMI command/event
 *    structures. Do not use A_UINT8, A_UINT16, A_BOOL or enum types within these structures.
 * 3. DO NOT define bit fields within structures. Implement bit fields using masks
 *    if necessary. Do not use the programming language's bit field definition.
 * 4. Define macros for encode/decode of A_UINT8, A_UINT16 fields within the A_UINT32
 *    variables. Use these macros for set/get of these fields. Try to use this to
 *    optimize the structure without bloating it with A_UINT32 variables for every lower
 *    sized field.
 * 5. Do not use PACK/UNPACK attributes for the structures as each member variable is
 *    already 4-byte aligned by virtue of being a A_UINT32 type.
 * 6. Comment each parameter part of the WMI command/event structure by using the
 *    2 stars at the beginning of C comment instead of one star to enable HTML document
 *    generation using Doxygen.
 *
 */

#ifndef _WMI_UNIFIED_H_
#define _WMI_UNIFIED_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <wlan_defs.h>
#include <wmi_services.h>
/*
 * Include the defs of vendor-specific messages (or possibly dummy defs
 * if there are no actual vendor-specific message defs).
 */
#include <wmi_unified_vendor.h>

#define ATH_MAC_LEN             6               /**< length of MAC in bytes */
#define WMI_EVENT_STATUS_SUCCESS 0 /* Success return status to host */
#define WMI_EVENT_STATUS_FAILURE 1 /* Failure return status to host */

#define MAX_TX_RATE_VALUES      10 /*Max Tx Rates*/
#define MAX_RSSI_VALUES         10 /*Max Rssi values*/
#define WMI_MAX_CHAINS 8
#define WMI_MAX_CHAINS_FOR_AOA_RCC 2
#define WMI_MAX_ADDRESS_SPACE   10

#define MAX_AOA_PHASEDELTA      31  /* 62 gain values */

#define MAX_20MHZ_SEGMENTS 16  /* 320 MHz / 20 MHz = 16 (20 MHz subbands) */

/* The WLAN_MAX_AC macro cannot be changed without breaking
   WMI compatibility. */
/* The maximum value of access category */
#define WLAN_MAX_AC  4

#define WMI_MAX_PN_LEN 8

#define MAX_NUM_CQI_USERS_IN_STANDALONE_SND 3

/*
 * These don't necessarily belong here; but as the MS/SM macros require
 * ar6000_internal.h to be included, it may not be defined as yet.
 */
#define WMI_F_MS(_v, _f)                                            \
            (((_v) & (_f)) >> (_f##_S))

/*
 * This breaks the "good macro practice" of only referencing each
 * macro field once (to avoid things like field++ from causing issues.)
 */
#define WMI_F_RMW(_var, _v, _f)                                     \
            do {                                                    \
                (_var) &= ~(_f);                                    \
                (_var) |= (((_v) << (_f##_S)) & (_f));              \
            } while (0)

#define WMI_GET_BITS(_val,_index,_num_bits)                         \
    (((_val) >> (_index)) & (((A_UINT32) 1 << (_num_bits)) - 1))

#define WMI_SET_BITS(_var,_index,_num_bits,_val) do {                       \
    (_var) &= ~((((A_UINT32) 1 << (_num_bits)) - 1) << (_index));           \
    (_var) |= (((_val) & (((A_UINT32) 1 << (_num_bits)) - 1)) << (_index)); \
    } while (0)

#define WMI_APPEND_TWO_SET_BITS(var, lsb_index, lsb_num_bits, msb_index, msb_num_bits, val) \
    do { \
        WMI_SET_BITS(var, lsb_index, lsb_num_bits, val); \
        WMI_SET_BITS(var, msb_index, msb_num_bits, (val >> lsb_num_bits)); \
    } while(0)

#define WMI_APPEND_TWO_GET_BITS(var, lsb_index, lsb_num_bits, msb_index, msb_num_bits, val) \
    do { \
        (var) = WMI_GET_BITS(val, lsb_index, lsb_num_bits); \
        (var) |= (WMI_GET_BITS(val, msb_index, msb_num_bits) << lsb_num_bits); \
    } while(0)

/*
 * Below GET/SET BITS_ARRAY_LEN32_BYTES macros can be used when
 * reading/writing bits which are spread across array_len32 entries.
 * These can be used to GET/SET maximum of 32 bits only,
 * also array_len32 length should be limited to maximum of 32.
 */
#define WMI_GET_BITS_ARRAY_LEN32_BYTES(var, _arrayp, _index, _num_bits) \
    do { \
        A_UINT8 i; \
        for (i = 0; i < _num_bits; i++) { \
            (var) |= (WMI_GET_BITS(_arrayp[(_index+i) / 32], ((_index+i) % 32), 1) << i); \
        } \
    } while(0)

#define WMI_SET_BITS_ARRAY_LEN32_BYTES(_arrayp, _index, _num_bits, val) \
    do { \
        A_UINT8 i; \
        for (i = 0; i < _num_bits; i++) { \
            WMI_SET_BITS(_arrayp[(_index+i) / 32], ((_index+i) % 32), 1, (val >> i)); \
        } \
    } while(0)

/**
 * A packed array is an array where each entry in the array is less than
 * or equal to 16 bits, and the entries are stuffed into an A_UINT32 array.
 * For example, if each entry in the array is 11 bits, then you can stuff
 * an array of 4 11-bit values into an array of 2 A_UINT32 values.
 * The first 2 11-bit values will be stored in the first A_UINT32,
 * and the last 2 11-bit values will be stored in the second A_UINT32.
 */
#define WMI_PACKED_ARR_SIZE(num_entries,bits_per_entry) \
    (((num_entries) / (32 / (bits_per_entry))) +            \
    (((num_entries) % (32 / (bits_per_entry))) ? 1 : 0))

#define WMI_RETURN_STRING(str) case ((str)): return (A_UINT8 *)(# str);

static INLINE A_UINT32 wmi_packed_arr_get_bits(A_UINT32 *arr,
    A_UINT32 entry_index, A_UINT32 bits_per_entry)
{
    A_UINT32 entries_per_uint = (32 / bits_per_entry);
    A_UINT32 uint_index = (entry_index / entries_per_uint);
    A_UINT32 num_entries_in_prev_uints = (uint_index * entries_per_uint);
    A_UINT32 index_in_uint = (entry_index - num_entries_in_prev_uints);
    A_UINT32 start_bit_in_uint = (index_in_uint * bits_per_entry);
    return (arr[uint_index] >> start_bit_in_uint) &
            (((A_UINT32) 1 << bits_per_entry) - 1);
}

static INLINE void wmi_packed_arr_set_bits(A_UINT32 *arr, A_UINT32 entry_index,
    A_UINT32 bits_per_entry, A_UINT32 val)
{
    A_UINT32 entries_per_uint = (32 / bits_per_entry);
    A_UINT32 uint_index = (entry_index / entries_per_uint);
    A_UINT32 num_entries_in_prev_uints = (uint_index * entries_per_uint);
    A_UINT32 index_in_uint = (entry_index - num_entries_in_prev_uints);
    A_UINT32 start_bit_in_uint = (index_in_uint * bits_per_entry);

    arr[uint_index] &=
        ~((((A_UINT32) 1 << bits_per_entry) - 1) << start_bit_in_uint);
    arr[uint_index] |=
        ((val & (((A_UINT32) 1 << bits_per_entry) - 1)) << start_bit_in_uint);
}

/** macro to convert MAC address from WMI word format to char array */
#define WMI_MAC_ADDR_TO_CHAR_ARRAY(pwmi_mac_addr,c_macaddr) do {        \
     (c_macaddr)[0] = (((pwmi_mac_addr)->mac_addr31to0)  >>  0) & 0xff; \
     (c_macaddr)[1] = (((pwmi_mac_addr)->mac_addr31to0)  >>  8) & 0xff; \
     (c_macaddr)[2] = (((pwmi_mac_addr)->mac_addr31to0)  >> 16) & 0xff; \
     (c_macaddr)[3] = (((pwmi_mac_addr)->mac_addr31to0)  >> 24) & 0xff; \
     (c_macaddr)[4] = (((pwmi_mac_addr)->mac_addr47to32) >>  0) & 0xff; \
     (c_macaddr)[5] = (((pwmi_mac_addr)->mac_addr47to32) >>  8) & 0xff; \
   } while (0)

/** macro to convert MAC address from char array to WMI word format */
#define WMI_CHAR_ARRAY_TO_MAC_ADDR(c_macaddr,pwmi_mac_addr)  do { \
    (pwmi_mac_addr)->mac_addr31to0 = \
       (((A_UINT32)(c_macaddr)[0] <<  0) | \
        ((A_UINT32)(c_macaddr)[1] <<  8) | \
        ((A_UINT32)(c_macaddr)[2] << 16) | \
        ((A_UINT32)(c_macaddr)[3] << 24)); \
    (pwmi_mac_addr)->mac_addr47to32 = ((c_macaddr)[4] | ((c_macaddr)[5] << 8));\
   } while (0)

/*
 * The below function declarations are for implementations on some
 * platforms of the above macros, but in function form, to save code
 * memory by avoiding macro-inlining of a non-trivial amount of code.
 * These function versions of the above macros may not be available
 * on all host and target platforms.
 */
void wmi_mac_addr_to_char_array(wmi_mac_addr *pwmi_mac_addr, A_UINT8 *c_macaddr);
void wmi_char_array_to_mac_addr(A_UINT8 *c_macaddr, wmi_mac_addr *pwmi_mac_addr);

/*
 * wmi command groups.
 */
typedef enum {
    /* 0 to 2 are reserved */
    WMI_GRP_START = 0x3,
    WMI_GRP_SCAN = WMI_GRP_START, /* 0x3 */
    WMI_GRP_PDEV,           /* 0x4 */
    WMI_GRP_VDEV,           /* 0x5 */
    WMI_GRP_PEER,           /* 0x6 */
    WMI_GRP_MGMT,           /* 0x7 */
    WMI_GRP_BA_NEG,         /* 0x8 */
    WMI_GRP_STA_PS,         /* 0x9 */
    WMI_GRP_DFS,            /* 0xa */
    WMI_GRP_ROAM,           /* 0xb */
    WMI_GRP_OFL_SCAN,       /* 0xc */
    WMI_GRP_P2P,            /* 0xd */
    WMI_GRP_AP_PS,          /* 0xe */
    WMI_GRP_RATE_CTRL,      /* 0xf */
    WMI_GRP_PROFILE,        /* 0x10 */
    WMI_GRP_SUSPEND,        /* 0x11 */
    WMI_GRP_BCN_FILTER,     /* 0x12 */
    WMI_GRP_WOW,            /* 0x13 */
    WMI_GRP_RTT,            /* 0x14 */
    WMI_GRP_SPECTRAL,       /* 0x15 */
    WMI_GRP_STATS,          /* 0x16 */
    WMI_GRP_ARP_NS_OFL,     /* 0x17 */
    WMI_GRP_NLO_OFL,        /* 0x18 */
    WMI_GRP_GTK_OFL,        /* 0x19 */
    WMI_GRP_CSA_OFL,        /* 0x1a */
    WMI_GRP_CHATTER,        /* 0x1b */
    WMI_GRP_TID_ADDBA,      /* 0x1c */
    WMI_GRP_MISC,           /* 0x1d */
    WMI_GRP_GPIO,           /* 0x1e */
    WMI_GRP_FWTEST,         /* 0x1f */
    WMI_GRP_TDLS,           /* 0x20 */
    WMI_GRP_RESMGR,         /* 0x21 */
    WMI_GRP_STA_SMPS,       /* 0x22 */
    WMI_GRP_WLAN_HB,        /* 0x23 */
    WMI_GRP_RMC,            /* 0x24 */
    WMI_GRP_MHF_OFL,        /* 0x25 */
    WMI_GRP_LOCATION_SCAN,  /* 0x26 */
    WMI_GRP_OEM,            /* 0x27 */
    WMI_GRP_NAN,            /* 0x28 */
    WMI_GRP_COEX,           /* 0x29 */
    WMI_GRP_OBSS_OFL,       /* 0x2a */
    WMI_GRP_LPI,            /* 0x2b */
    WMI_GRP_EXTSCAN,        /* 0x2c */
    WMI_GRP_DHCP_OFL,       /* 0x2d */
    WMI_GRP_IPA,            /* 0x2e */
    WMI_GRP_MDNS_OFL,       /* 0x2f */
    WMI_GRP_SAP_OFL,        /* 0x30 */
    WMI_GRP_OCB,            /* 0x31 */
    WMI_GRP_SOC,            /* 0x32 */
    WMI_GRP_PKT_FILTER,     /* 0x33 */
    WMI_GRP_MAWC,           /* 0x34 */
    WMI_GRP_PMF_OFFLOAD,    /* 0x35 */
    WMI_GRP_BPF_OFFLOAD,    /* 0x36 Berkeley Packet Filter */
    WMI_GRP_NAN_DATA,       /* 0x37 */
    WMI_GRP_PROTOTYPE,      /* 0x38 */
    WMI_GRP_MONITOR,        /* 0x39 */
    WMI_GRP_REGULATORY,     /* 0x3a */
    WMI_GRP_HW_DATA_FILTER, /* 0x3b */
    WMI_GRP_WLM,            /* 0x3c WLAN Latency Manager */
    WMI_GRP_11K_OFFLOAD,    /* 0x3d */
    WMI_GRP_TWT,            /* 0x3e TWT (Target Wake Time) for STA and AP */
    WMI_GRP_MOTION_DET,     /* 0x3f */
    WMI_GRP_SPATIAL_REUSE,  /* 0x40 */
    WMI_GRP_ESP,            /* 0x41 Estimate Service Parameters (802.11mc) */
    WMI_GRP_HPCS_PULSE,     /* 0x42 */
    WMI_GRP_AUDIO,          /* 0x43 */
    WMI_GRP_CFR_CAPTURE,    /* 0x44 */
    WMI_GRP_ATM,            /* 0x45 ATM (Air Time Management group) */
    WMI_GRP_VENDOR,         /* 0x46 vendor specific group */
    WMI_GRP_LATENCY,        /* 0x47 TID/AC level latency config */
    WMI_GRP_MLO,            /* 0x48 MLO(Multiple Link Operation) management */
    WMI_GRP_SAWF,           /* 0x49 SAWF (Service Aware WiFi) */
    WMI_GRP_QUIET_OFL,      /* 0x4a Quiet offloads */
    WMI_GRP_ODD,            /* 0x4b ODD */
    WMI_GRP_TDMA,           /* 0x4c TDMA */
    WMI_GRP_MANUAL_UL_TRIG  /* 0x4d Manual UL OFDMA Trigger */
} WMI_GRP_ID;

#define WMI_CMD_GRP_START_ID(grp_id) (((grp_id) << 12) | 0x1)
#define WMI_EVT_GRP_START_ID(grp_id) (((grp_id) << 12) | 0x1)

/**
 * Command IDs and commange events
 */
typedef enum {
    /** initialize the wlan sub system */
    WMI_INIT_CMDID = 0x1,

    /* Scan specific commands */

    /** start scan request to FW  */
    WMI_START_SCAN_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_SCAN),
    /** stop scan request to FW  */
    WMI_STOP_SCAN_CMDID,
    /** full list of channels as defined by the regulatory that will be used by scanner   */
    WMI_SCAN_CHAN_LIST_CMDID,
    /** overwrite default priority table in scan scheduler   */
    WMI_SCAN_SCH_PRIO_TBL_CMDID,
    /** This command to adjust the priority and min.max_rest_time
     * of an on ongoing scan request.
     */
    WMI_SCAN_UPDATE_REQUEST_CMDID,

    /** set OUI to be used in probe request if enabled */
    WMI_SCAN_PROB_REQ_OUI_CMDID,
    /** config adaptive dwell scan */
    WMI_SCAN_ADAPTIVE_DWELL_CONFIG_CMDID,
    /** Only applicable to DBS capable product */
    WMI_SET_SCAN_DBS_DUTY_CYCLE_CMDID,

    /* PDEV(physical device) specific commands */
    /** set regulatorty ctl id used by FW to determine the exact ctl power limits */
    WMI_PDEV_SET_REGDOMAIN_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_PDEV),
    /** set channel. mainly used for supporting monitor mode */
    WMI_PDEV_SET_CHANNEL_CMDID,
    /** set pdev specific parameters */
    WMI_PDEV_SET_PARAM_CMDID,
    /** enable packet log */
    WMI_PDEV_PKTLOG_ENABLE_CMDID,
    /** disable packet log*/
    WMI_PDEV_PKTLOG_DISABLE_CMDID,
    /** set wmm parameters */
    WMI_PDEV_SET_WMM_PARAMS_CMDID,
    /** set HT cap ie that needs to be carried probe requests HT/VHT channels */
    WMI_PDEV_SET_HT_CAP_IE_CMDID,
    /** set VHT cap ie that needs to be carried on probe requests on VHT channels */
    WMI_PDEV_SET_VHT_CAP_IE_CMDID,

    /** Command to send the DSCP-to-TID map to the target */
    WMI_PDEV_SET_DSCP_TID_MAP_CMDID,
    /** set quiet ie parameters. primarily used in AP mode */
    WMI_PDEV_SET_QUIET_MODE_CMDID,
    /** Enable/Disable Green AP Power Save  */
    WMI_PDEV_GREEN_AP_PS_ENABLE_CMDID,
    /** get TPC config for the current operating channel */
    WMI_PDEV_GET_TPC_CONFIG_CMDID,

    /** set the base MAC address for the physical device before a VDEV is created.
     *  For firmware that doesn't support this feature and this command, the pdev
     *  MAC address will not be changed. */
    WMI_PDEV_SET_BASE_MACADDR_CMDID,

    /* eeprom content dump , the same to bdboard data */
    WMI_PDEV_DUMP_CMDID,
    /* set LED configuration  */
    WMI_PDEV_SET_LED_CONFIG_CMDID,
    /* Get Current temperature of chip in Celcius degree*/
    WMI_PDEV_GET_TEMPERATURE_CMDID,
    /* Set LED flashing behavior  */
    WMI_PDEV_SET_LED_FLASHING_CMDID,
    /** Enable/Disable Smart Antenna */
    WMI_PDEV_SMART_ANT_ENABLE_CMDID,
    /** Set Smart Antenna RX antenna*/
    WMI_PDEV_SMART_ANT_SET_RX_ANTENNA_CMDID,
    /** Override the antenna switch table */
    WMI_PDEV_SET_ANTENNA_SWITCH_TABLE_CMDID,
    /** Override the CTL table */
    WMI_PDEV_SET_CTL_TABLE_CMDID,
    /** Override the array gain table */
    WMI_PDEV_SET_MIMOGAIN_TABLE_CMDID,
    /** FIPS test mode command */
    WMI_PDEV_FIPS_CMDID,
    /** get CCK ANI level */
    WMI_PDEV_GET_ANI_CCK_CONFIG_CMDID,
    /** get OFDM ANI level */
    WMI_PDEV_GET_ANI_OFDM_CONFIG_CMDID,
    /** NF Cal Power dBr/dBm */
    WMI_PDEV_GET_NFCAL_POWER_CMDID,
    /** TxPPDU TPC */
    WMI_PDEV_GET_TPC_CMDID,
    /** Set to enable MIB stats collection */
    WMI_MIB_STATS_ENABLE_CMDID,
    /** Set preferred channel list for DBS Mgr */
    WMI_PDEV_SET_PCL_CMDID,
    /** Set HW mode. Eg: single MAC, DBS & SBS, see soc_hw_mode_t for values */
    WMI_PDEV_SET_HW_MODE_CMDID,
    /** Set DFS, SCAN modes and other FW configurations */
    WMI_PDEV_SET_MAC_CONFIG_CMDID,
    /** Set per band and per pdev antenna chains */
    WMI_PDEV_SET_ANTENNA_MODE_CMDID,
    /** Periodic channel stats request command */
    WMI_SET_PERIODIC_CHANNEL_STATS_CONFIG_CMDID,
    /** WMI command for power debug framework */
    WMI_PDEV_WAL_POWER_DEBUG_CMDID,
    /** set per-AC rx reorder timeouts */
    WMI_PDEV_SET_REORDER_TIMEOUT_VAL_CMDID,
    /** WMI command for WOW gpio and type */
    WMI_PDEV_SET_WAKEUP_CONFIG_CMDID,
    /* Get current ANT's per chain's RSSI info */
    WMI_PDEV_GET_ANTDIV_STATUS_CMDID,
    /** WMI command for getting Chip Power Stats */
    WMI_PDEV_GET_CHIP_POWER_STATS_CMDID,
    /** set stats reporting thresholds - see WMI_REPORT_STATS_EVENTID */
    WMI_PDEV_SET_STATS_THRESHOLD_CMDID,
    /** vdev restart request for multiple vdevs */
    WMI_PDEV_MULTIPLE_VDEV_RESTART_REQUEST_CMDID,
    /** Pdev update packet routing command */
    WMI_PDEV_UPDATE_PKT_ROUTING_CMDID,
    /** Get Calibration data version details */
    WMI_PDEV_CHECK_CAL_VERSION_CMDID,
    /** Set Diversity Gain */
    WMI_PDEV_SET_DIVERSITY_GAIN_CMDID,
    /** Get chain RSSI and antena index command */
    WMI_PDEV_DIV_GET_RSSI_ANTID_CMDID,
    /** get bss chan info */
    WMI_PDEV_BSS_CHAN_INFO_REQUEST_CMDID,
    /** update pmk cache info */
    WMI_PDEV_UPDATE_PMK_CACHE_CMDID,
    /**  update fils HLP */
    WMI_PDEV_UPDATE_FILS_HLP_PKT_CMDID,
    /** update ctltable request **/
    WMI_PDEV_UPDATE_CTLTABLE_REQUEST_CMDID,
    /** Command to set beacon OUI **/
    WMI_PDEV_CONFIG_VENDOR_OUI_ACTION_CMDID,
    /** enable/disable per-AC tx queue optimizations */
    WMI_PDEV_SET_AC_TX_QUEUE_OPTIMIZED_CMDID,
    /** enable/disable rx promiscuous mode */
    WMI_PDEV_SET_RX_FILTER_PROMISCUOUS_CMDID,
    /* set a generic direct DMA ring config */
    WMI_PDEV_DMA_RING_CFG_REQ_CMDID,
    /* enable/disable Action frame response as HE TB PPDU */
    WMI_PDEV_HE_TB_ACTION_FRM_CMDID,
    /** filter packet log based on MAC address */
    WMI_PDEV_PKTLOG_FILTER_CMDID,
    /** wmi command for setting rogue ap configuration */
    WMI_PDEV_SET_RAP_CONFIG_CMDID,
    /** Specify DSM filters along with disallow bssid filters */
    WMI_PDEV_DSM_FILTER_CMDID,
    /** enable/disable periodic frame injection */
    WMI_PDEV_FRAME_INJECT_CMDID,
    /*
     * Pdev level command to:
     * a) solicit @WMI_TBTTOFFSET_EXT_UPDATE_EVENTID having TBTT in qtime
     *    domain for all active vdevs or
     * b) update one pdevs tbtt offset to another pdev for use in
     *    RNR TBTT offset calculation.
     */
    WMI_PDEV_TBTT_OFFSET_SYNC_CMDID,
    /** Bss color bitmap for SRG based spatial reuse feature */
    WMI_PDEV_SET_SRG_BSS_COLOR_BITMAP_CMDID,
    /** Partial BSSID bitmap for SRG based spatial reuse feature */
    WMI_PDEV_SET_SRG_PARTIAL_BSSID_BITMAP_CMDID,
    /** OBSS color enable bitmap for SRG based spatial reuse feature */
    WMI_PDEV_SET_SRG_OBSS_COLOR_ENABLE_BITMAP_CMDID,
    /** OBSS BSSID enable bitmap for SRG based spatial reuse feature */
    WMI_PDEV_SET_SRG_OBSS_BSSID_ENABLE_BITMAP_CMDID,
    /** OBSS color enable bitmap for NON_SRG based spatial reuse feature */
    WMI_PDEV_SET_NON_SRG_OBSS_COLOR_ENABLE_BITMAP_CMDID,
    /** OBSS BSSID enable bitmap for NON_SRG based spatial reuse feature */
    WMI_PDEV_SET_NON_SRG_OBSS_BSSID_ENABLE_BITMAP_CMDID,
    /** TPC stats display command */
    WMI_PDEV_GET_TPC_STATS_CMDID,
    /** ENABLE/DISABLE Duration based tx mode selection */
    WMI_PDEV_ENABLE_DURATION_BASED_TX_MODE_SELECTION_CMDID,
    /* Get DPD status from HALPHY */
    WMI_PDEV_GET_DPD_STATUS_CMDID,
    /* Set bios sar table */
    WMI_PDEV_SET_BIOS_SAR_TABLE_CMDID,
    /* Set bios geo table */
    WMI_PDEV_SET_BIOS_GEO_TABLE_CMDID,
    /* Get Calibration status from HALPHY */
    WMI_PDEV_GET_HALPHY_CAL_STATUS_CMDID,
    /* Set HALPHY CAL bitmap */
    WMI_PDEV_SET_HALPHY_CAL_BMAP_CMDID,
    /* WMI cmd to set a single vdev param for multiple vdevs */
    WMI_PDEV_MULTIPLE_VDEV_SET_PARAM_CMDID,
    /* Configure MEC AGING TIMER */
    WMI_PDEV_MEC_AGING_TIMER_CONFIG_CMDID,
    /* Set bios interface table */
    WMI_PDEV_SET_BIOS_INTERFACE_CMDID,
    WMI_PDEV_FIPS_EXTEND_CMDID,
    WMI_PDEV_FIPS_MODE_SET_CMDID,
    WMI_PDEV_FEATURESET_CMDID,
    /** tag as Filter Pass category and the filters set for FP mode */
    WMI_PDEV_MESH_RX_FILTER_ENABLE_CMDID,
    /* WMI cmd to set Target rate to power table */
    WMI_PDEV_SET_TGTR2P_TABLE_CMDID,
    /* WMI cmd to set RF path for PHY */
    WMI_PDEV_SET_RF_PATH_CMDID,
    /** WSI stats info WMI command */
    WMI_PDEV_WSI_STATS_INFO_CMDID,
    /*
     * WMI cmd to Enable LED blink based on Tx+Rx Data Rate
     * and download LED ON/OFF Rate table
     */
    WMI_PDEV_ENABLE_LED_BLINK_DOWNLOAD_TABLE_CMDID,


    /* VDEV (virtual device) specific commands */
    /** vdev create */
    WMI_VDEV_CREATE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_VDEV),
    /** vdev delete */
    WMI_VDEV_DELETE_CMDID,
    /** vdev start request */
    WMI_VDEV_START_REQUEST_CMDID,
    /** vdev restart request (RX only, NO TX, used for CAC period)*/
    WMI_VDEV_RESTART_REQUEST_CMDID,
    /** vdev up request */
    WMI_VDEV_UP_CMDID,
    /** vdev stop request */
    WMI_VDEV_STOP_CMDID,
    /** vdev down request */
    WMI_VDEV_DOWN_CMDID,
    /* set a vdev param */
    WMI_VDEV_SET_PARAM_CMDID,
    /* set a key (used for setting per peer unicast and per vdev multicast) */
    WMI_VDEV_INSTALL_KEY_CMDID,

    /* wnm sleep mode command */
    WMI_VDEV_WNM_SLEEPMODE_CMDID,
    WMI_VDEV_WMM_ADDTS_CMDID,
    WMI_VDEV_WMM_DELTS_CMDID,
    WMI_VDEV_SET_WMM_PARAMS_CMDID,
    WMI_VDEV_SET_GTX_PARAMS_CMDID,
    WMI_VDEV_IPSEC_NATKEEPALIVE_FILTER_CMDID,

    WMI_VDEV_PLMREQ_START_CMDID,
    WMI_VDEV_PLMREQ_STOP_CMDID,
    /* TSF timestamp action for specified vdev */
    WMI_VDEV_TSF_TSTAMP_ACTION_CMDID,
    /** set the additional IEs in probe requests for scan or
     *  assoc req etc for frames FW locally generates */
    WMI_VDEV_SET_IE_CMDID,

    WMI_VDEV_RATEMASK_CMDID,
    /** ATF VDEV REQUEST commands. */
    WMI_VDEV_ATF_REQUEST_CMDID,
    /** Command to send the DSCP-to-TID map to the target for VAP */
    WMI_VDEV_SET_DSCP_TID_MAP_CMDID,
    /* Configure filter for Neighbor Rx Pkt (smart mesh selective listening) */
    WMI_VDEV_FILTER_NEIGHBOR_RX_PACKETS_CMDID,
    /** set quiet ie parameters. primarily used in AP mode */
    WMI_VDEV_SET_QUIET_MODE_CMDID,
    /** To set custom aggregation size for per vdev */
    WMI_VDEV_SET_CUSTOM_AGGR_SIZE_CMDID,

    /* DISA feature: Encrypt-decrypt data request */
    WMI_VDEV_ENCRYPT_DECRYPT_DATA_REQ_CMDID,

    /** Command to enable mac randomizaton **/
    WMI_VDEV_ADD_MAC_ADDR_TO_RX_FILTER_CMDID,

    /** WMI commands related to dbg arp stats */
    WMI_VDEV_SET_ARP_STAT_CMDID,
    WMI_VDEV_GET_ARP_STAT_CMDID,

    /** get tx power for the current vdev */
    WMI_VDEV_GET_TX_POWER_CMDID,
    /* limit STA offchannel activity */
    WMI_VDEV_LIMIT_OFFCHAN_CMDID,
    /** To set custom software retries per-AC for vdev */
    WMI_VDEV_SET_CUSTOM_SW_RETRY_TH_CMDID,
    /** To set chainmask configuration for vdev */
    WMI_VDEV_CHAINMASK_CONFIG_CMDID,

    WMI_VDEV_GET_BCN_RECEPTION_STATS_CMDID,
    /* request LTE-Coex info */
    WMI_VDEV_GET_MWS_COEX_INFO_CMDID,
    /** delete all peer (excluding bss peer) */
    WMI_VDEV_DELETE_ALL_PEER_CMDID,
    /* To set bss max idle time related parameters */
    WMI_VDEV_BSS_MAX_IDLE_TIME_CMDID,
    /** Indicates FW to trigger Audio sync  */
    WMI_VDEV_AUDIO_SYNC_TRIGGER_CMDID,
    /** Gives Qtimer value  to FW  */
    WMI_VDEV_AUDIO_SYNC_QTIMER_CMDID,
    /** Preferred channel list for each vdev */
    WMI_VDEV_SET_PCL_CMDID,
    /** VDEV_GET_BIG_DATA_CMD IS DEPRECATED - DO NOT USE */
    WMI_VDEV_GET_BIG_DATA_CMDID,
    /** Get per vdev BIG DATA stats phase 2 */
    WMI_VDEV_GET_BIG_DATA_P2_CMDID,
    /** set TPC PSD/non-PSD power */
    WMI_VDEV_SET_TPC_POWER_CMDID,
    /** IGMP OFFLOAD */
    WMI_VDEV_IGMP_OFFLOAD_CMDID,
    /** Enable/Disable Intra Bss for each vdev */
    WMI_VDEV_ENABLE_DISABLE_INTRA_BSS_CMDID,
    /* set vdev mu sniffer param */
    WMI_VDEV_SET_MU_SNIF_CMDID,
    /** ICMP OFFLOAD */
    WMI_VDEV_ICMP_OFFLOAD_CMDID,
    /** Update vdev mac address */
    WMI_VDEV_UPDATE_MAC_ADDR_CMDID,
    /* WMI cmd to perform operation on multiple peer based on subcmd type */
    WMI_VDEV_MULTIPLE_PEER_GROUP_CMDID,
    /** Set LTF key seed which will be further used to derive LTF keys */
    WMI_VDEV_SET_LTF_KEY_SEED_CMDID,

    WMI_VDEV_PN_MGMT_RX_FILTER_CMDID,

    /** Enable SR prohibit feature for TIDs of vdev */
    WMI_VDEV_PARAM_ENABLE_SR_PROHIBIT_CMDID,

    /** pause vdev's Tx, Rx, or both for a specific duration */
    WMI_VDEV_PAUSE_CMDID,

    /** WMI Command to set status of CSA event from HOST */
    WMI_CSA_EVENT_STATUS_INDICATION_CMDID,

    /** Request to firmware to probe scheduler modes */
    WMI_VDEV_SCHED_MODE_PROBE_REQ_CMDID,

    /** Connect request on the vdev */
    WMI_VDEV_OOB_CONNECTION_REQ_CMDID,


    /* peer specific commands */

    /** create a peer */
    WMI_PEER_CREATE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_PEER),
    /** delete a peer */
    WMI_PEER_DELETE_CMDID,
    /** flush specific  tid queues of a peer */
    WMI_PEER_FLUSH_TIDS_CMDID,
    /** set a parameter of a peer */
    WMI_PEER_SET_PARAM_CMDID,
    /** set peer to associated state. will cary all parameters determined during assocication time */
    WMI_PEER_ASSOC_CMDID,
    /**add a wds  (4 address) entry. used only for testing WDS feature on AP products */
    WMI_PEER_ADD_WDS_ENTRY_CMDID,
    /**remove wds  (4 address) entry. used only for testing WDS feature on AP products */
    WMI_PEER_REMOVE_WDS_ENTRY_CMDID,
    /** set up mcast group info for multicast to unicast conversion */
    WMI_PEER_MCAST_GROUP_CMDID,
    /** request peer info from FW. FW shall respond with PEER_INFO_EVENTID */
    WMI_PEER_INFO_REQ_CMDID,
    /** request the estimated link speed for the peer. FW shall respond with
     *  WMI_PEER_ESTIMATED_LINKSPEED_EVENTID.
     */
    WMI_PEER_GET_ESTIMATED_LINKSPEED_CMDID,
    /** Set the conditions to report peer justified rate to driver
     * The justified rate means the user-rate is justified by PER.
     */
    WMI_PEER_SET_RATE_REPORT_CONDITION_CMDID,

    /** update a wds (4 address) entry */
    WMI_PEER_UPDATE_WDS_ENTRY_CMDID,
    /** add a proxy sta entry */
    WMI_PEER_ADD_PROXY_STA_ENTRY_CMDID,
    /** Set Smart Antenna TX antenna */
    WMI_PEER_SMART_ANT_SET_TX_ANTENNA_CMDID,
    /** Set Smart Antenna TX train info */
    WMI_PEER_SMART_ANT_SET_TRAIN_INFO_CMDID,
    /** Set SA node config options */
    WMI_PEER_SMART_ANT_SET_NODE_CONFIG_OPS_CMDID,
    /** ATF PEER REQUEST commands */
    WMI_PEER_ATF_REQUEST_CMDID,
    /** bandwidth fairness (BWF) peer configuration request command */
    WMI_PEER_BWF_REQUEST_CMDID,
    /** rx reorder queue setup for peer/tid */
    WMI_PEER_REORDER_QUEUE_SETUP_CMDID,
    /** rx reorder queue remove for peer/tid */
    WMI_PEER_REORDER_QUEUE_REMOVE_CMDID,
    /** specify a limit for rx A-MPDU block size */
    WMI_PEER_SET_RX_BLOCKSIZE_CMDID,
    /** request peer antdiv info from FW. FW shall respond with PEER_ANTDIV_INFO_EVENTID */
    WMI_PEER_ANTDIV_INFO_REQ_CMDID,
    /*
     * The WMI_PEER_OPER_MODE_CHANGE_EVENTID def was originally mistakenly
     * placed here, amongst the CMDID defs.
     * The WMI_PEER_OPER_MODE_CHANGE_EVENTID def has been moved to the
     * EVENTID section, but to preserve backwards compatibility, the value
     * here that had been used for WMI_PEER_OPER_MODE_CHANGE_EVENTID
     * is kept reserved/deprecated.
     *
     * This WMI_PEER_RESERVED0_CMDID value can be replaced with an actual
     * WMI peer event message ID, though it will be simpler to instead add
     * new WMI_PEER CMDID defs at the end of the WMI_GRP_PEER WMI_CMD_GRP.
     */
    WMI_PEER_RESERVED0_CMDID,
    /** Peer/Tid/Msduq threshold update */
    WMI_PEER_TID_MSDUQ_QDEPTH_THRESH_UPDATE_CMDID,
    /** TID specific configurations per peer of type
     * wmi_peer_tid_configurations_cmd_fixed_param
     */
    WMI_PEER_TID_CONFIGURATIONS_CMDID,

    /** Peer configuration for Channel Frequency Response (CFR) capture
     * of type wmi_peer_cfr_capture_cmd.
     *
     * On targets that do not use the direct DMA framework,
     * completion of the CFR capture is communicated through
     * HTT_T2H_MSG_TYPE_CFR_DUMP_COMPL_IND.
     * Such targets will set WMI_SERVICE_CFR_CAPTURE_IND_MSG_TYPE_1
     * in WMI Service Ready.
     *
     * On targets that use direct DMA, completion of CFR capture is
     * communicated through WMI_PDEV_DMA_RING_BUF_RELEASE_EVENTID
     * using module ID WMI_DMA_RING_CONFIG_MODULE_RTT.
     * Such targets will set WMI_SERVICE_CFR_CAPTURE_IND_EVT_TYPE_1
     * in WMI Service Ready and enumerate WMI_DMA_RING_CONFIG_MODULE_RTT
     * in the dma_ring_caps entry of WMI_SERVICE_READY_EXT_EVENTID.
     * Additional MAC metadata is provided in WMI_PEER_CFR_CAPTURE_EVENTID.
     */
    WMI_PEER_CFR_CAPTURE_CMDID,

    /** WMI command related to AP channel width switching */
    WMI_PEER_CHAN_WIDTH_SWITCH_CMDID,

    /** WMI command to fetch current tx PN for the peer */
    WMI_PEER_TX_PN_REQUEST_CMDID,

    /** unmap response with peer ids */
    WMI_PEER_UNMAP_RESPONSE_CMDID,

    /** WMI command for per-peer configuration of VLAN header operations
     * during TX and RX
     */
    WMI_PEER_CONFIG_VLAN_CMDID,

    /** WMI command for per-peer configuration of PPE DS */
    WMI_PEER_CONFIG_PPE_DS_CMDID,

    /** Enable/Disable Intra Bss for the peer */
    WMI_PEER_ENABLE_DISABLE_INTRA_BSS_CMDID,

    WMI_PEER_RX_PN_REQUEST_CMDID,

    /* Mac addr based filtering*/
    WMI_PEER_TX_FILTER_CMDID,

    /** flush specific tid queues of a peer */
    WMI_PEER_FLUSH_POLICY_CMDID,

    /* Set disabled scheduler modes for one or more peers */
    WMI_PEER_SCHED_MODE_DISABLE_CMDID,

    /* Group SET cmd for PEERS */
    WMI_PEER_BULK_SET_CMDID,

    /* WMI command to setup reorder queue for multiple TIDs */
    WMI_PEER_MULTIPLE_REORDER_QUEUE_SETUP_CMDID,

    /* beacon/management specific commands */

    /** transmit beacon by reference . used for transmitting beacon on low latency interface like pcie */
    WMI_BCN_TX_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_MGMT),
    /** transmit beacon by value */
    WMI_PDEV_SEND_BCN_CMDID,
    /** set the beacon template. used in beacon offload mode to setup the
     *  the common beacon template with the FW to be used by FW to generate beacons */
    WMI_BCN_TMPL_CMDID,
    /** set beacon filter with FW */
    WMI_BCN_FILTER_RX_CMDID,
    /* enable/disable filtering of probe requests in the firmware */
    WMI_PRB_REQ_FILTER_RX_CMDID,
    /** transmit management frame by value. will be deprecated */
    WMI_MGMT_TX_CMDID,
    /** set the probe response template. used in beacon offload mode to setup the
     *  the common probe response template with the FW to be used by FW to generate
     *  probe responses */
    WMI_PRB_TMPL_CMDID,
    /** Transmit Mgmt frame by reference */
    WMI_MGMT_TX_SEND_CMDID,
    /** Transmit data frame by reference */
    WMI_OFFCHAN_DATA_TX_SEND_CMDID,
    /** transmit FILS Discovery frame by value */
    WMI_PDEV_SEND_FD_CMDID,
    /** Cmd to enable/disable offloaded beacons */
    WMI_BCN_OFFLOAD_CTRL_CMDID,
    /** Cmd to enable FW handling BSS color change notification from AP. */
    WMI_BSS_COLOR_CHANGE_ENABLE_CMDID,
    /** To configure Beacon offload quiet-ie params */
    WMI_VDEV_BCN_OFFLOAD_QUIET_CONFIG_CMDID,
    /** set FILS Discovery frame template for FW to generate FD frames */
    WMI_FD_TMPL_CMDID,
    /** Transmit QoS null Frame over wmi interface */
    WMI_QOS_NULL_FRAME_TX_SEND_CMDID,
    /** WMI CMD to receive the management filter criteria from the host for RX REO */
    WMI_MGMT_RX_REO_FILTER_CONFIGURATION_CMDID,

    /** commands to directly control ba negotiation directly from host. only used in test mode */

    /** turn off FW Auto addba mode and let host control addba */
    WMI_ADDBA_CLEAR_RESP_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_BA_NEG),
    /** send add ba request */
    WMI_ADDBA_SEND_CMDID,
    WMI_ADDBA_STATUS_CMDID,
    /** send del ba */
    WMI_DELBA_SEND_CMDID,
    /** set add ba response will be used by FW to generate addba response*/
    WMI_ADDBA_SET_RESP_CMDID,
    /** send single VHT MPDU with AMSDU */
    WMI_SEND_SINGLEAMSDU_CMDID,

    /** Station power save specific config */
    /** enable/disable station powersave */
    WMI_STA_POWERSAVE_MODE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_STA_PS),
    /** set station power save specific parameter */
    WMI_STA_POWERSAVE_PARAM_CMDID,
    /** set station mimo powersave mode */
    WMI_STA_MIMO_PS_MODE_CMDID,
    /** config station TX cycle percentage in a beacon interval */
    WMI_STA_TDCC_CONFIG_CMDID,


    /** DFS-specific commands */
    /** enable DFS (radar detection)*/
    WMI_PDEV_DFS_ENABLE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_DFS),
    /** disable DFS (radar detection)*/
    WMI_PDEV_DFS_DISABLE_CMDID,
    /** enable DFS phyerr/parse filter offload */
    WMI_DFS_PHYERR_FILTER_ENA_CMDID,
    /** enable DFS phyerr/parse filter offload */
    WMI_DFS_PHYERR_FILTER_DIS_CMDID,
    /** enable DFS phyerr processing offload */
    WMI_PDEV_DFS_PHYERR_OFFLOAD_ENABLE_CMDID,
    /** disable DFS phyerr processing offload */
    WMI_PDEV_DFS_PHYERR_OFFLOAD_DISABLE_CMDID,
    /** set ADFS channel config */
    WMI_VDEV_ADFS_CH_CFG_CMDID,
    /** abort ADFS off-channel-availability-check currently in progress */
    WMI_VDEV_ADFS_OCAC_ABORT_CMDID,

    /* Roaming specific commands */
    /** set roam scan mode */
    WMI_ROAM_SCAN_MODE = WMI_CMD_GRP_START_ID(WMI_GRP_ROAM),
    /** set roam scan RSSI threshold below which roam scan is enabled  */
    WMI_ROAM_SCAN_RSSI_THRESHOLD,
    /** set roam scan period for periodic roam scan mode  */
    WMI_ROAM_SCAN_PERIOD,
    /** set roam scan trigger RSSI change threshold   */
    WMI_ROAM_SCAN_RSSI_CHANGE_THRESHOLD,
    /** set roam AP profile   */
    WMI_ROAM_AP_PROFILE,
    /** set channel list for roam scans */
    WMI_ROAM_CHAN_LIST,
    /** Stop scan command */
    WMI_ROAM_SCAN_CMD,
    /** roaming sme offload sync complete */
    WMI_ROAM_SYNCH_COMPLETE,
    /** set ric request element for 11r roaming */
    WMI_ROAM_SET_RIC_REQUEST_CMDID,
    /** Invoke roaming forcefully */
    WMI_ROAM_INVOKE_CMDID,
    /** roaming filter cmd to allow further filtering of roaming candidate */
    WMI_ROAM_FILTER_CMDID,
    /** set gateway ip, mac and retries for subnet change detection */
    WMI_ROAM_SUBNET_CHANGE_CONFIG_CMDID,
    /** configure thresholds for MAWC */
    WMI_ROAM_CONFIGURE_MAWC_CMDID,
    /** configure MultiBand Operation(refer WFA MBO spec) parameter */
    WMI_ROAM_SET_MBO_PARAM_CMDID, /* DEPRECATED */
    /** configure packet error rate threshold for triggering roaming */
    WMI_ROAM_PER_CONFIG_CMDID,
    /** configure BSS Transition Management (BTM) offload for roaming */
    WMI_ROAM_BTM_CONFIG_CMDID,
    /** Enable or Disable Fast Initial Link Setup (FILS) feature */
    WMI_ENABLE_FILS_CMDID,
    /** Request for roam scan stats */
    WMI_REQUEST_ROAM_SCAN_STATS_CMDID,
    /** Configure BSS load parameters for roam trigger */
    WMI_ROAM_BSS_LOAD_CONFIG_CMDID,
    /** Configure deauth roam trigger parameters */
    WMI_ROAM_DEAUTH_CONFIG_CMDID,
    /** Configure idle roam trigger parameters */
    WMI_ROAM_IDLE_CONFIG_CMDID,
    /**
     * WMI_ROAM_DSM_FILTER_CMDID is deprecated and should be unused,
     * but leave it reserved just to be safe.
     */
    DEPRECATED__WMI_ROAM_DSM_FILTER_CMDID,
    /** Enable or disable roaming triggers */
    WMI_ROAM_ENABLE_DISABLE_TRIGGER_REASON_CMDID,
    /** Pre-Authentication completion status command */
    WMI_ROAM_PREAUTH_STATUS_CMDID,
    /** Command to get roam scan channels list */
    WMI_ROAM_GET_SCAN_CHANNEL_LIST_CMDID,
    /** configure MLO parameters for roaming */
    WMI_ROAM_MLO_CONFIG_CMDID,
    /** set roam params **/
    WMI_ROAM_SET_PARAM_CMDID,
    /** Enable or Disable roam vendor control */
    WMI_ROAM_ENABLE_VENDOR_CONTROL_CMDID,
    /** Get firmware ini value */
    WMI_ROAM_GET_VENDOR_CONTROL_PARAM_CMDID,

    /** offload scan specific commands */
    /** set offload scan AP profile   */
    WMI_OFL_SCAN_ADD_AP_PROFILE = WMI_CMD_GRP_START_ID(WMI_GRP_OFL_SCAN),
    /** remove offload scan AP profile   */
    WMI_OFL_SCAN_REMOVE_AP_PROFILE,
    /** set offload scan period   */
    WMI_OFL_SCAN_PERIOD,

    /* P2P specific commands */
    /**set P2P device info. FW will used by FW to create P2P IE to be carried in probe response
     * generated during p2p listen and for p2p discoverability  */
    WMI_P2P_DEV_SET_DEVICE_INFO = WMI_CMD_GRP_START_ID(WMI_GRP_P2P),
    /** enable/disable p2p discoverability on STA/AP VDEVs  */
    WMI_P2P_DEV_SET_DISCOVERABILITY,
    /** set p2p ie to be carried in beacons generated by FW for GO  */
    WMI_P2P_GO_SET_BEACON_IE,
    /** set p2p ie to be carried in probe response frames generated by FW for GO  */
    WMI_P2P_GO_SET_PROBE_RESP_IE,
    /** set the vendor specific p2p ie data. FW will use this to parse the P2P NoA
     *  attribute in the beacons/probe responses received.
     *  Note: This command is currently used only for Apple P2P implementation.
     */
    WMI_P2P_SET_VENDOR_IE_DATA_CMDID,
    /** set the configure of p2p find offload */
    WMI_P2P_DISC_OFFLOAD_CONFIG_CMDID,
    /** set the vendor specific p2p ie data for p2p find offload using */
    WMI_P2P_DISC_OFFLOAD_APPIE_CMDID,
    /** set the BSSID/device name pattern of p2p find offload */
    WMI_P2P_DISC_OFFLOAD_PATTERN_CMDID,
    /** set OppPS related parameters **/
    WMI_P2P_SET_OPPPS_PARAM_CMDID,
    /** set listen offload start related parameters */
    WMI_P2P_LISTEN_OFFLOAD_START_CMDID,
    /** set listen offload stop related parameters */
    WMI_P2P_LISTEN_OFFLOAD_STOP_CMDID,

    /** AP power save specific config */
    /** set AP power save specific param */
    WMI_AP_PS_PEER_PARAM_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_AP_PS),
    /** set AP UAPSD coex specific param */
    WMI_AP_PS_PEER_UAPSD_COEX_CMDID,
    /** set Enhanced Green AP param */
    WMI_AP_PS_EGAP_PARAM_CMDID,

    /** Rate-control specific commands */
    WMI_PEER_RATE_RETRY_SCHED_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_RATE_CTRL),

    /** WLAN Profiling commands. */
    WMI_WLAN_PROFILE_TRIGGER_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_PROFILE),
    WMI_WLAN_PROFILE_SET_HIST_INTVL_CMDID,
    WMI_WLAN_PROFILE_GET_PROFILE_DATA_CMDID,
    WMI_WLAN_PROFILE_ENABLE_PROFILE_ID_CMDID,
    WMI_WLAN_PROFILE_LIST_PROFILE_ID_CMDID,

    /** Suspend resume command Ids */
    WMI_PDEV_SUSPEND_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_SUSPEND),
    WMI_PDEV_RESUME_CMDID,

    /* Beacon filter commands */
    /** add a beacon filter */
    WMI_ADD_BCN_FILTER_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_BCN_FILTER),
    /** remove a  beacon filter */
    WMI_RMV_BCN_FILTER_CMDID,

    /* WOW Specific WMI commands*/
    /** add pattern for awake */
    WMI_WOW_ADD_WAKE_PATTERN_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_WOW),
    /** deleta a wake pattern */
    WMI_WOW_DEL_WAKE_PATTERN_CMDID,
    /** enable/deisable wake event  */
    WMI_WOW_ENABLE_DISABLE_WAKE_EVENT_CMDID,
    /** enable WOW  */
    WMI_WOW_ENABLE_CMDID,
    /** host woke up from sleep event to FW. Generated in response to WOW Hardware event */
    WMI_WOW_HOSTWAKEUP_FROM_SLEEP_CMDID,
    /* IOAC add keep alive cmd. */
    WMI_WOW_IOAC_ADD_KEEPALIVE_CMDID,
    /* IOAC del keep alive cmd. */
    WMI_WOW_IOAC_DEL_KEEPALIVE_CMDID,
    /* IOAC add pattern for awake */
    WMI_WOW_IOAC_ADD_WAKE_PATTERN_CMDID,
    /* IOAC deleta a wake pattern */
    WMI_WOW_IOAC_DEL_WAKE_PATTERN_CMDID,
    /* D0-WOW enable or disable cmd */
    WMI_D0_WOW_ENABLE_DISABLE_CMDID,
    /* enable extend WoW */
    WMI_EXTWOW_ENABLE_CMDID,
    /* Extend WoW command to configure app type1 parameter */
    WMI_EXTWOW_SET_APP_TYPE1_PARAMS_CMDID,
    /* Extend WoW command to configure app type2 parameter */
    WMI_EXTWOW_SET_APP_TYPE2_PARAMS_CMDID,
    /* enable ICMPv6 Network advertisement filtering */
    WMI_WOW_ENABLE_ICMPV6_NA_FLT_CMDID,
    /*
     * Set a pattern to match UDP packet in WOW mode.
     * If match, construct a tx frame in a local buffer
     * to send through the peer AP to the entity in the
     * IP network that sent the UDP packet to this STA.
     */
    WMI_WOW_UDP_SVC_OFLD_CMDID,
    /* configure WOW host wakeup PIN pattern */
    WMI_WOW_HOSTWAKEUP_GPIO_PIN_PATTERN_CONFIG_CMDID,

    /* Set which action category should wake the host from suspend */
    WMI_WOW_SET_ACTION_WAKE_UP_CMDID,

    /*
     * Set a pattern to match broadcast CoAP packet in WoW mode.
     * If match and verify pass, cache the packet and then reply
     * a unicast response in local with pre-configured packet.
     */
    WMI_WOW_COAP_ADD_PATTERN_CMDID,

    /* Delete a pattern match broadcast CoAP packet */
    WMI_WOW_COAP_DEL_PATTERN_CMDID,

    /*
     * Add a CoAP keepalive pattern to send a CoAP broadcast packet
     * when configured timeout occurred.
     */
    WMI_WOW_COAP_ADD_KEEPALIVE_PATTERN_CMDID,

    /* Delete a CoAP keepalive pattern */
    WMI_WOW_COAP_DEL_KEEPALIVE_PATTERN_CMDID,

    /* Host read the cached CoAP packets after resume */
    WMI_WOW_COAP_GET_BUF_INFO_CMDID,


    /* RTT measurement related cmd */
    /** request to make an RTT measurement */
    WMI_RTT_MEASREQ_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_RTT),
    /** request to report a tsf measurement */
    WMI_RTT_TSF_CMDID,
    /** RTT 11az PASN authentication status */
    WMI_RTT_PASN_AUTH_STATUS_CMD,
    /** RTT 11az PASN deauthentication cmd */
    WMI_RTT_PASN_DEAUTH_CMD,

    /** spectral scan command */
    /** configure spectral scan */
    WMI_VDEV_SPECTRAL_SCAN_CONFIGURE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_SPECTRAL),
    /** enable/disable spectral scan and trigger */
    WMI_VDEV_SPECTRAL_SCAN_ENABLE_CMDID,

    /* F/W stats */
    /** one time request for stats */
    WMI_REQUEST_STATS_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_STATS),
    /** Push MCC Adaptive Scheduler Stats to Firmware */
    WMI_MCC_SCHED_TRAFFIC_STATS_CMDID,
    /** one time request for txrx stats */
    WMI_REQUEST_STATS_EXT_CMDID,
    /* Link Layer stats */
    /** Request for link layer stats */
    WMI_REQUEST_LINK_STATS_CMDID,
    /** Request for setting params to link layer stats */
    WMI_START_LINK_STATS_CMDID,
    /** Request to clear stats*/
    WMI_CLEAR_LINK_STATS_CMDID,

    /** Request for getting the Firmware Memory Dump */
    WMI_GET_FW_MEM_DUMP_CMDID,

    /** Request to flush of the buffered debug messages */
    WMI_DEBUG_MESG_FLUSH_CMDID,

    /** Cmd to configure the verbose level */
    WMI_DIAG_EVENT_LOG_CONFIG_CMDID,

    /** One time request for wlan stats */
    WMI_REQUEST_WLAN_STATS_CMDID,

    /** Request for getting RCPI of peer */
    WMI_REQUEST_RCPI_CMDID,

    /** One time request for peer stats info */
    WMI_REQUEST_PEER_STATS_INFO_CMDID,

    /** One time request for radio channel stats */
    WMI_REQUEST_RADIO_CHAN_STATS_CMDID,

    /** request for WLM (wlan latency manager) stats */
    WMI_REQUEST_WLM_STATS_CMDID,

    /** request for control path stats */
    WMI_REQUEST_CTRL_PATH_STATS_CMDID,

    /** unified request for LL stats and get station cmds */
    WMI_REQUEST_UNIFIED_LL_GET_STA_CMDID,

    /** request for thermal stats */
    WMI_REQUEST_THERMAL_STATS_CMDID,

    /** request for HALPHY stats through ctrl path */
    WMI_REQUEST_HALPHY_CTRL_PATH_STATS_CMDID,


    /** ARP OFFLOAD REQUEST*/
    WMI_SET_ARP_NS_OFFLOAD_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_ARP_NS_OFL),

    /** Proactive ARP Response Add Pattern Command*/
    WMI_ADD_PROACTIVE_ARP_RSP_PATTERN_CMDID,

    /** Proactive ARP Response Del Pattern Command*/
    WMI_DEL_PROACTIVE_ARP_RSP_PATTERN_CMDID,

    /** NS offload confid*/
    WMI_NETWORK_LIST_OFFLOAD_CONFIG_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_NLO_OFL),

    /** APFIND Config */
    WMI_APFIND_CMDID,

    /** Passpoint list config  */
    WMI_PASSPOINT_LIST_CONFIG_CMDID,

    /** configure suppressing parameters for MAWC */
    WMI_NLO_CONFIGURE_MAWC_CMDID,

    /* GTK offload Specific WMI commands*/
    WMI_GTK_OFFLOAD_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_GTK_OFL),

    /* CSA offload Specific WMI commands*/
    /** csa offload enable */
    WMI_CSA_OFFLOAD_ENABLE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_CSA_OFL),
    /** chan switch command */
    WMI_CSA_OFFLOAD_CHANSWITCH_CMDID,

    /* Chatter commands*/
    /* Change chatter mode of operation */
    WMI_CHATTER_SET_MODE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_CHATTER),
    /** chatter add coalescing filter command */
    WMI_CHATTER_ADD_COALESCING_FILTER_CMDID,
    /** chatter delete coalescing filter command */
    WMI_CHATTER_DELETE_COALESCING_FILTER_CMDID,
    /** chatter coalecing query command */
    WMI_CHATTER_COALESCING_QUERY_CMDID,

    /**addba specific commands */
    /** start the aggregation on this TID */
    WMI_PEER_TID_ADDBA_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_TID_ADDBA),
    /** stop the aggregation on this TID */
    WMI_PEER_TID_DELBA_CMDID,

    /** set station mimo powersave method */
    WMI_STA_DTIM_PS_METHOD_CMDID,
    /** Configure the Station UAPSD AC Auto Trigger Parameters */
    WMI_STA_UAPSD_AUTO_TRIG_CMDID,
    /** Configure the Keep Alive Parameters */
    WMI_STA_KEEPALIVE_CMDID,

    /* Request ssn from target for a sta/tid pair */
    WMI_BA_REQ_SSN_CMDID,


    /* misc command group */
    /** echo command mainly used for testing */
    WMI_ECHO_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_MISC),

    /* !!IMPORTANT!!
     * If you need to add a new WMI command to the WMI_GRP_MISC sub-group,
     * please make sure you add it BEHIND WMI_PDEV_UTF_CMDID,
     * as we MUST have a fixed value here to maintain compatibility between
     * UTF and the ART2 driver
     */
    /** UTF WMI commands */
    WMI_PDEV_UTF_CMDID,

    /** set debug log config */
    WMI_DBGLOG_CFG_CMDID,
    /* QVIT specific command id */
    WMI_PDEV_QVIT_CMDID,
    /* Factory Testing Mode request command
     * used for integrated chipsets */
    WMI_PDEV_FTM_INTG_CMDID,
    /* set and get keepalive parameters command */
    WMI_VDEV_SET_KEEPALIVE_CMDID,
    WMI_VDEV_GET_KEEPALIVE_CMDID,
    /* For fw recovery test command */
    WMI_FORCE_FW_HANG_CMDID,
    /* Set Mcast/Bdcast filter */
    WMI_SET_MCASTBCAST_FILTER_CMDID,
    /** set thermal management params **/
    WMI_THERMAL_MGMT_CMDID,
    /** set host auto shutdown params **/
    WMI_HOST_AUTO_SHUTDOWN_CFG_CMDID,
    /** set tpc chainmask config command */
    WMI_TPC_CHAINMASK_CONFIG_CMDID,
    /** set Antenna diversity command */
    WMI_SET_ANTENNA_DIVERSITY_CMDID,
    /** Set OCB Sched Request, deprecated */
    WMI_OCB_SET_SCHED_CMDID,
    /** Set RSSI monitoring config command */
    WMI_RSSI_BREACH_MONITOR_CONFIG_CMDID,
    /** Enable/disable Large Receive Offload processing; provide cfg params */
    WMI_LRO_CONFIG_CMDID,
    /** transfer data from host to firmware to write flash */
    WMI_TRANSFER_DATA_TO_FLASH_CMDID,
    /** Command to enable/disable filtering of multicast IP with unicast mac */
    WMI_CONFIG_ENHANCED_MCAST_FILTER_CMDID,
    /** Command to control WISA mode */
    WMI_VDEV_WISA_CMDID,
    /** set debug log time stamp sync up with host */
    WMI_DBGLOG_TIME_STAMP_SYNC_CMDID,
    /** Command for host to set/delete multiple mcast filters */
    WMI_SET_MULTIPLE_MCAST_FILTER_CMDID,
    /** upload a requested section of data from firmware flash to host */
    WMI_READ_DATA_FROM_FLASH_CMDID,
    /* Thermal Throttling SET CONF commands */
    WMI_THERM_THROT_SET_CONF_CMDID,
    /* set runtime dpd recalibration params */
    WMI_RUNTIME_DPD_RECAL_CMDID,
    /* get TX power for input HALPHY parameters */
    WMI_GET_TPC_POWER_CMDID,
    /* Specify when to start monitoring for idle state */
    WMI_IDLE_TRIGGER_MONITOR_CMDID,
    /** set ELNA BYPASS status */
    WMI_SET_ELNA_BYPASS_CMDID,
    /** get ELNA BYPASS status */
    WMI_GET_ELNA_BYPASS_CMDID,
    /** get ANI level of the channels */
    WMI_GET_CHANNEL_ANI_CMDID,
    /** set OCL (One Chain Listen) mode */
    WMI_SET_OCL_CMDID,
    /** Consolidated params for pdev/vdev:
     * Set multiple parameters at once for one pdev or vdev.
     */
    WMI_SET_MULTIPLE_PDEV_VDEV_PARAM_CMDID,

    /* WMI cmd used to allocate HW scratch registers */
    WMI_PMM_SCRATCH_REG_ALLOCATION_CMDID,

    /* WMI cmd used to start/stop XGAP (XPAN Green AP) */
    WMI_XGAP_ENABLE_CMDID,

    /* H2T HPA message */
    WMI_HPA_CMDID,

    /* WMI comamnd for standalone sounding */
    WMI_VDEV_STANDALONE_SOUND_CMDID,

    /* WMI cmd used by host to send the switch response status to FW */
    WMI_AUDIO_TRANSPORT_SWITCH_RESP_STATUS_CMDID,

    /*  Offload 11k related requests */
    WMI_11K_OFFLOAD_REPORT_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_11K_OFFLOAD),
    /* invoke neighbor report from FW */
    WMI_11K_INVOKE_NEIGHBOR_REPORT_CMDID,

    /* GPIO Configuration */
    WMI_GPIO_CONFIG_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_GPIO),
    WMI_GPIO_OUTPUT_CMDID,

    /* Txbf configuration command */
    WMI_TXBF_CMDID,

    /* Antenna Controller, connected to wlan debug uart/GPIO. */
    WMI_ANT_CONTROLLER_CMDID,

    WMI_GPIO_STATE_REQ_CMDID,

    /* FWTEST Commands */
    WMI_FWTEST_VDEV_MCC_SET_TBTT_MODE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_FWTEST),
    /** set NoA descs **/
    WMI_FWTEST_P2P_SET_NOA_PARAM_CMDID,
    /* UNIT Tests  */
    WMI_UNIT_TEST_CMDID,
    /* set debug and tuning parameters */
    WMI_FWTEST_CMDID,
    /* Q-Boost configuration test commands */
    WMI_QBOOST_CFG_CMDID,
    /* Simulation Test command  */
    WMI_SIMULATION_TEST_CMDID,
    /* WFA test config command */
    WMI_WFA_CONFIG_CMDID,

    /** TDLS Configuration */
    /** enable/disable TDLS */
    WMI_TDLS_SET_STATE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_TDLS),
    /** set tdls peer state */
    WMI_TDLS_PEER_UPDATE_CMDID,
    /** TDLS Offchannel control */
    WMI_TDLS_SET_OFFCHAN_MODE_CMDID,

    /** Resmgr Configuration */
    /** Adaptive OCS is enabled by default in the FW. This command is used to
     * disable FW based adaptive OCS.
     */
    WMI_RESMGR_ADAPTIVE_OCS_ENABLE_DISABLE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_RESMGR),
    /** set the requested channel time quota for the home channels */
    WMI_RESMGR_SET_CHAN_TIME_QUOTA_CMDID,
    /** set the requested latency for the home channels */
    WMI_RESMGR_SET_CHAN_LATENCY_CMDID,

    /** STA SMPS Configuration */
    /** force SMPS mode */
    WMI_STA_SMPS_FORCE_MODE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_STA_SMPS),
    /** set SMPS parameters */
    WMI_STA_SMPS_PARAM_CMDID,

    /* Wlan HB commands*/
    /* enable/disable wlan HB */
    WMI_HB_SET_ENABLE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_WLAN_HB),
    /* set tcp parameters for wlan HB */
    WMI_HB_SET_TCP_PARAMS_CMDID,
    /* set tcp pkt filter for wlan HB */
    WMI_HB_SET_TCP_PKT_FILTER_CMDID,
    /* set udp parameters for wlan HB */
    WMI_HB_SET_UDP_PARAMS_CMDID,
    /* set udp pkt filter for wlan HB */
    WMI_HB_SET_UDP_PKT_FILTER_CMDID,

    /* OIC ping keep alive */
    WMI_HB_OIC_PING_OFFLOAD_PARAM_CMDID,
    WMI_HB_OIC_PING_OFFLOAD_SET_ENABLE_CMDID,

    /* WMI commands related to DHCP Lease Renew Offload **/
    WMI_HB_DHCP_LEASE_RENEW_OFFLOAD_CMDID,

    /** Wlan RMC commands*/
    /** enable/disable RMC */
    WMI_RMC_SET_MODE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_RMC),
    /** configure action frame period */
    WMI_RMC_SET_ACTION_PERIOD_CMDID,
    /** For debug/future enhancement purposes only,
     *  configures/finetunes RMC algorithms */
    WMI_RMC_CONFIG_CMDID,
    /** select manual leader */
    WMI_RMC_SET_MANUAL_LEADER_CMDID,

    /** WLAN MHF offload commands */
    /** enable/disable MHF offload */
    WMI_MHF_OFFLOAD_SET_MODE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_MHF_OFL),
    /** Plumb routing table for MHF offload */
    WMI_MHF_OFFLOAD_PLUMB_ROUTING_TBL_CMDID,

    /*location scan commands*/
    /*start batch scan*/
    WMI_BATCH_SCAN_ENABLE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_LOCATION_SCAN),
    /*stop batch scan*/
    WMI_BATCH_SCAN_DISABLE_CMDID,
    /*get batch scan result*/
    WMI_BATCH_SCAN_TRIGGER_RESULT_CMDID,


    /* OEM related cmd */
    WMI_OEM_REQ_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_OEM),
    WMI_OEM_REQUEST_CMDID, /* UNUSED */
    /* OEM related cmd used for Low Power ranging */
    WMI_LPI_OEM_REQ_CMDID,
    WMI_OEM_DMA_RING_CFG_REQ_CMDID,
    /** Command to handle OEM's opaque data */
    WMI_OEM_DATA_CMDID,


    /** Nan Request */
    WMI_NAN_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_NAN),
    /** Command to handle OEM's NAN specific opaque data */
    WMI_NAN_OEM_DATA_CMDID,

    /** Modem power state command */
    WMI_MODEM_POWER_STATE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_COEX),
    WMI_CHAN_AVOID_UPDATE_CMDID,
    WMI_COEX_CONFIG_CMDID,
    WMI_CHAN_AVOID_RPT_ALLOW_CMDID,
    WMI_COEX_GET_ANTENNA_ISOLATION_CMDID,
    WMI_SAR_LIMITS_CMDID,
    WMI_SAR_GET_LIMITS_CMDID,
    /** Dedicated BT Antenna Mode (DBAM) command */
    WMI_COEX_DBAM_CMDID,
    WMI_TAS_POWER_HISTORY_CMDID,
    WMI_ESL_EGID_CMDID,
    WMI_COEX_MULTIPLE_CONFIG_CMDID,

    /**
     *  OBSS scan offload enable/disable commands
     *  OBSS scan enable CMD will send to FW after VDEV UP, if these conditions are true:
     *  1.  WMI_SERVICE_OBSS_SCAN is reported by FW in service ready,
     *  2.  STA connect to a 2.4 GHz ht20/ht40 AP,
     *  3.  AP enable 20/40 coexistence (OBSS_IE-74 can be found in beacon or association response)
     *  If OBSS parameters from beacon changed, also use enable CMD to update parameters.
     *  OBSS scan disable CMD will send to FW if have enabled when tearing down connection.
     */
    WMI_OBSS_SCAN_ENABLE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_OBSS_OFL),
    WMI_OBSS_SCAN_DISABLE_CMDID,
    WMI_OBSS_COLOR_COLLISION_DET_CONFIG_CMDID,

    /** LPI commands */
    /** LPI mgmt snooping config command */
    WMI_LPI_MGMT_SNOOPING_CONFIG_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_LPI),
    /** LPI scan start command */
    WMI_LPI_START_SCAN_CMDID,
    /** LPI scan stop command */
    WMI_LPI_STOP_SCAN_CMDID,

    /** ExtScan commands */
    WMI_EXTSCAN_START_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_EXTSCAN),
    WMI_EXTSCAN_STOP_CMDID,
    WMI_EXTSCAN_CONFIGURE_WLAN_CHANGE_MONITOR_CMDID,
    WMI_EXTSCAN_CONFIGURE_HOTLIST_MONITOR_CMDID,
    WMI_EXTSCAN_GET_CACHED_RESULTS_CMDID,
    WMI_EXTSCAN_GET_WLAN_CHANGE_RESULTS_CMDID,
    WMI_EXTSCAN_SET_CAPABILITIES_CMDID,
    WMI_EXTSCAN_GET_CAPABILITIES_CMDID,
    WMI_EXTSCAN_CONFIGURE_HOTLIST_SSID_MONITOR_CMDID,
    WMI_EXTSCAN_CONFIGURE_MAWC_CMDID,

    /** DHCP server offload commands */
    WMI_SET_DHCP_SERVER_OFFLOAD_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_DHCP_OFL),

    /** IPA Offload features related commands */
    WMI_IPA_OFFLOAD_ENABLE_DISABLE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_IPA),

    /** mDNS responder offload commands */
    WMI_MDNS_OFFLOAD_ENABLE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_MDNS_OFL),
    WMI_MDNS_SET_FQDN_CMDID,
    WMI_MDNS_SET_RESPONSE_CMDID,
    WMI_MDNS_GET_STATS_CMDID,
    WMI_MDNS_SET_STAIP_CMDID,

    /* enable/disable AP Authentication offload */
    WMI_SAP_OFL_ENABLE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_SAP_OFL),
    WMI_SAP_SET_BLACKLIST_PARAM_CMDID,
    WMI_SAP_OBSS_DETECTION_CFG_CMDID,

    /** Out-of-context-of-BSS (OCB) commands */
    WMI_OCB_SET_CONFIG_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_OCB),
    WMI_OCB_SET_UTC_TIME_CMDID,
    WMI_OCB_START_TIMING_ADVERT_CMDID,
    WMI_OCB_STOP_TIMING_ADVERT_CMDID,
    WMI_OCB_GET_TSF_TIMER_CMDID,
    WMI_DCC_GET_STATS_CMDID,
    WMI_DCC_CLEAR_STATS_CMDID,
    WMI_DCC_UPDATE_NDL_CMDID,

    /* System-On-Chip commands */
    WMI_SOC_SET_PCL_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_SOC),
    WMI_SOC_SET_HW_MODE_CMDID,
    WMI_SOC_SET_DUAL_MAC_CONFIG_CMDID,
    WMI_SOC_SET_ANTENNA_MODE_CMDID,
    /** enable/disable TQM reset (SOC level) feature */
    WMI_SOC_TQM_RESET_ENABLE_DISABLE_CMDID,

    /* packet filter commands */
    WMI_PACKET_FILTER_CONFIG_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_PKT_FILTER),
    WMI_PACKET_FILTER_ENABLE_CMDID,

    /** Motion Aided WiFi Connectivity (MAWC) commands */
    WMI_MAWC_SENSOR_REPORT_IND_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_MAWC),

    /** WMI commands related to PMF 11w Offload */
    WMI_PMF_OFFLOAD_SET_SA_QUERY_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_PMF_OFFLOAD),

    /** WMI commands related to pkt filter (BPF) offload */
    WMI_BPF_GET_CAPABILITY_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_BPF_OFFLOAD),
    WMI_BPF_GET_VDEV_STATS_CMDID,
    WMI_BPF_SET_VDEV_INSTRUCTIONS_CMDID,
    WMI_BPF_DEL_VDEV_INSTRUCTIONS_CMDID,
    WMI_BPF_SET_VDEV_ACTIVE_MODE_CMDID,
    WMI_BPF_SET_VDEV_ENABLE_CMDID,
    WMI_BPF_SET_VDEV_WORK_MEMORY_CMDID,
    WMI_BPF_GET_VDEV_WORK_MEMORY_CMDID,

    /** WMI commands related to monitor mode. */
    WMI_MNT_FILTER_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_MONITOR),

    /** WMI commands related to regulatory offload */
    WMI_SET_CURRENT_COUNTRY_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_REGULATORY),
    WMI_11D_SCAN_START_CMDID,
    WMI_11D_SCAN_STOP_CMDID,
    WMI_SET_INIT_COUNTRY_CMDID,
    WMI_AFC_CMDID,

    /**
     * Nan Data commands
     * NDI - NAN Data Interface
     * NDP - NAN Data Path
     */
    /* Commands in prototyping phase */
    WMI_NDI_GET_CAP_REQ_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_PROTOTYPE),
    WMI_NDP_INITIATOR_REQ_CMDID,
    WMI_NDP_RESPONDER_REQ_CMDID,
    WMI_NDP_END_REQ_CMDID,
    WMI_NDP_CMDID,

    /** WMI commands related to HW data filtering **/
    WMI_HW_DATA_FILTER_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_HW_DATA_FILTER),

    /** WMI commands related to WLAN latency module **/
    WMI_WLM_CONFIG_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_WLM),

    /** WMI commands related to STA & AP TWT module **/
    WMI_TWT_ENABLE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_TWT),
    WMI_TWT_DISABLE_CMDID,
    WMI_TWT_ADD_DIALOG_CMDID,
    WMI_TWT_DEL_DIALOG_CMDID,
    WMI_TWT_PAUSE_DIALOG_CMDID,
    WMI_TWT_RESUME_DIALOG_CMDID,
    WMI_TWT_BTWT_INVITE_STA_CMDID,
    WMI_TWT_BTWT_REMOVE_STA_CMDID,
    WMI_TWT_NUDGE_DIALOG_CMDID,
    WMI_VDEV_SET_TWT_EDCA_PARAMS_CMDID, /* XPAN TWT */

    /** WMI commands related to motion detection **/
    WMI_MOTION_DET_CONFIG_PARAM_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_MOTION_DET),
    WMI_MOTION_DET_BASE_LINE_CONFIG_PARAM_CMDID,
    WMI_MOTION_DET_START_STOP_CMDID,
    WMI_MOTION_DET_BASE_LINE_START_STOP_CMDID,

    /** WMI commands related to OBSS PD Spatial Reuse **/
    WMI_PDEV_OBSS_PD_SPATIAL_REUSE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_SPATIAL_REUSE),
    WMI_PDEV_OBSS_PD_SPATIAL_REUSE_SET_DEF_OBSS_THRESH_CMDID,

    /** WMI commands related to High Precision Clock Synchronization feature **/
    WMI_HPCS_PULSE_START_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_HPCS_PULSE),

    /** WMI commands related to Audio Frame aggregation feature **/
    WMI_AUDIO_AGGR_ENABLE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_AUDIO),
    WMI_AUDIO_AGGR_ADD_GROUP_CMDID,
    WMI_AUDIO_AGGR_DEL_GROUP_CMDID,
    WMI_AUDIO_AGGR_SET_GROUP_RATE_CMDID,
    WMI_AUDIO_AGGR_SET_GROUP_RETRY_CMDID,
    WMI_AUDIO_AGGR_SET_GROUP_AUTO_RATE_CMDID,
    WMI_AUDIO_AGGR_SET_GROUP_PROBE_CMDID,
    WMI_AUDIO_AGGR_UPDATE_STA_GROUP_INFO_CMDID,
    WMI_AUDIO_AGGR_GET_STATISTICS_CMDID,
    WMI_AUDIO_AGGR_RESET_STATISTICS_CMDID,
    WMI_AUDIO_AGGR_SET_RTSCTS_CONFIG_CMDID,
    WMI_AUDIO_AGGR_SET_SCHED_METHOD_CMDID,
    WMI_AUDIO_AGGR_GET_SCHED_METHOD_CMDID,

    /** WMI commands related to Channel Frequency Response Capture **/
    WMI_CFR_CAPTURE_FILTER_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_CFR_CAPTURE),

    /** WMI commands related to Air Time Management feature **/
    /** ATF SSID GROUPING REQUEST command */
    WMI_ATF_SSID_GROUPING_REQUEST_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_ATM),
    /** WMM ATF Configuration for groups */
    WMI_ATF_GROUP_WMM_AC_CONFIG_REQUEST_CMDID,
    /** ATF Peer Extended Request command */
    WMI_PEER_ATF_EXT_REQUEST_CMDID,

    /** Vendor Defined WMI commands **/
    WMI_VENDOR_PDEV_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_VENDOR),
    WMI_VENDOR_VDEV_CMDID,
    WMI_VENDOR_PEER_CMDID,
    /** Further vendor cmd IDs can be added below **/

    /** WMI commands specific to Tid level Latency config **/
    /** VDEV Latency Config command */
    WMI_VDEV_TID_LATENCY_CONFIG_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_LATENCY),
    /** TID Latency Request command */
    WMI_PEER_TID_LATENCY_CONFIG_CMDID,

    /** WMI commands specific to MLO **/
    /** MLO link active / inactive Request command */
    WMI_MLO_LINK_SET_ACTIVE_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_MLO),
    /** WMI cmd used to indicate hw_links part of MLO */
    WMI_MLO_SETUP_CMDID,
    /** WMI cmd used for init synchronization of hw_links part of MLO */
    WMI_MLO_READY_CMDID,
    /** WMI cmd used for tearing down a hw_link part of MLO */
    WMI_MLO_TEARDOWN_CMDID,
    /** WMI cmd used to setup Tid to Link Mapping for a MLO Peer */
    WMI_MLO_PEER_TID_TO_LINK_MAP_CMDID,
    /** WMI cmd for dynamically deleting a link from a MLD VAP */
    WMI_MLO_LINK_REMOVAL_CMDID,
    /** WMI cmd used to setup Tid to Link Mapping for a MLO VAP */
    WMI_MLO_AP_VDEV_TID_TO_LINK_MAP_CMDID,
    /** WMI cmd used to get mlo link information */
    WMI_MLO_VDEV_GET_LINK_INFO_CMDID,
    /** WMI cmd used to set link BSS parameters */
    WMI_MLO_LINK_SET_BSS_PARAMS_CMDID,
    /** WMI cmd to confirm the status of link switch request handling */
    WMI_MLO_LINK_SWITCH_CONF_CMDID,
    /** WMI cmd to migrate the primary link peer */
    WMI_MLO_PRIMARY_LINK_PEER_MIGRATION_CMDID,
    /** WMI cmd to recommand preferred link */
    WMI_MLO_LINK_RECOMMENDATION_CMDID,

    /** WMI commands specific to Service Aware WiFi (SAWF) */
    /** configure or reconfigure the parameters for a service class */
    WMI_SAWF_SVC_CLASS_CFG_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_SAWF),
    /** disable a service class */
    WMI_SAWF_SVC_CLASS_DISABLE_CMDID,

    /* WMI commands specific to ODD */
    WMI_ODD_LIVEDUMP_REQUEST_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_ODD),

    /* WMI commands specific to TDMA */
    WMI_TDMA_SCHEDULE_REQUEST_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_TDMA),

    /* WMI commands specific to manually-triggered UL */
    /** WMI Command to set Manual SU UL OFDMA trigger parameters */
    WMI_VDEV_SET_ULOFDMA_MANUAL_SU_TRIG_CMDID = WMI_CMD_GRP_START_ID(WMI_GRP_MANUAL_UL_TRIG),

    /** WMI Command to set Manual MU UL OFDMA trigger parameters */
    WMI_VDEV_SET_ULOFDMA_MANUAL_MU_TRIG_CMDID,
} WMI_CMD_ID;

typedef enum {
    /** WMI service is ready; after this event WMI messages can be sent/received  */
    WMI_SERVICE_READY_EVENTID = 0x1,
    /** WMI is ready; after this event the wlan subsystem is initialized and can process commands. */
    WMI_READY_EVENTID,

    /** Specify what WMI services the target supports (for services beyond
     * what fits in the WMI_SERVICE_READY_EVENT message's wmi_service_bitmap)
     */
    WMI_SERVICE_AVAILABLE_EVENTID,

    /** Specify what numbers and kinds of interfaces (a.k.a. vdevs)
     * the target supports
     */
    WMI_IFACE_COMBINATION_IND_EVENTID,

    /** Scan specific events */
    WMI_SCAN_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_SCAN),

    /** Spectral scan FW params to host */
    WMI_PDEV_SSCAN_FW_PARAM_EVENTID,

    /** Spectral scan related event start/stop trigger to host  */
    WMI_SSCAN_EVT_MESSAGE_EVENTID,

    /** Spectral scan capabilities advertisement */
    WMI_SPECTRAL_CAPABILITIES_EVENTID,


    /* PDEV specific events */
    /** TPC config for the current operating channel */
    WMI_PDEV_TPC_CONFIG_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_PDEV),
    /** Channel stats event    */
    WMI_CHAN_INFO_EVENTID,

    /** PHY Error specific WMI event */
    WMI_PHYERR_EVENTID,

    /** eeprom dump event  */
    WMI_PDEV_DUMP_EVENTID,

    /** traffic pause event */
    WMI_TX_PAUSE_EVENTID,

    /** DFS radar event  */
    WMI_DFS_RADAR_EVENTID,

    /** track L1SS entry and residency event */
    WMI_PDEV_L1SS_TRACK_EVENTID,

    /** Report current temperature of the chip in Celcius degree */
    WMI_PDEV_TEMPERATURE_EVENTID,

    /** Extension of WMI_SERVICE_READY msg with extra target capability info */
    WMI_SERVICE_READY_EXT_EVENTID,

    /** FIPS test mode event */
    WMI_PDEV_FIPS_EVENTID,

    /** Channel hopping avoidance */
    WMI_PDEV_CHANNEL_HOPPING_EVENTID,

    /** CCK ANI level event */
    WMI_PDEV_ANI_CCK_LEVEL_EVENTID,

    /** OFDM ANI level event */
    WMI_PDEV_ANI_OFDM_LEVEL_EVENTID,

    /** Tx PPDU params */
    WMI_PDEV_TPC_EVENTID,

    /** NF Cal Power in DBR/DBM for all channels */
    WMI_PDEV_NFCAL_POWER_ALL_CHANNELS_EVENTID,

    /** SOC/PDEV events */
    WMI_PDEV_SET_HW_MODE_RESP_EVENTID,
    WMI_PDEV_HW_MODE_TRANSITION_EVENTID,
    WMI_PDEV_SET_MAC_CONFIG_RESP_EVENTID,
    /** Report ANT DIV feature's status */
    WMI_PDEV_ANTDIV_STATUS_EVENTID,
    /** Chip level Power stats */
    WMI_PDEV_CHIP_POWER_STATS_EVENTID,
    /** Power Save Failure Detected */
    WMI_PDEV_CHIP_POWER_SAVE_FAILURE_DETECTED_EVENTID,

    /* Event to report the switch count in csa of one or more VDEVs */
    WMI_PDEV_CSA_SWITCH_COUNT_STATUS_EVENTID,

    /** Report the caldata version to host */
    WMI_PDEV_CHECK_CAL_VERSION_EVENTID,

    /** Report chain RSSI and antenna index to host */
    WMI_PDEV_DIV_RSSI_ANTID_EVENTID,

    /** provide noise floor and cycle counts for a channel */
    WMI_PDEV_BSS_CHAN_INFO_EVENTID,

    /** Response received the ctl table to host */
    WMI_PDEV_UPDATE_CTLTABLE_EVENTID,

    WMI_PDEV_DMA_RING_CFG_RSP_EVENTID,

    WMI_PDEV_DMA_RING_BUF_RELEASE_EVENTID,

    /** WMI Event to deliver CTL Failsafe application */
    WMI_PDEV_CTL_FAILSAFE_CHECK_EVENTID,

    /* Event to report the switch count in BSS color of one or more VDEVs */
    WMI_PDEV_CSC_SWITCH_COUNT_STATUS_EVENTID,

    /* Event to send cold boot calibration data */
    WMI_PDEV_COLD_BOOT_CAL_DATA_EVENTID,

    /* Event to report a rogue ap info that is detected in fw */
    WMI_PDEV_RAP_INFO_EVENTID,

    WMI_CHAN_RF_CHARACTERIZATION_INFO_EVENTID,

    /** 2nd extension of SERVICE_READY msg with extra target capability info */
    WMI_SERVICE_READY_EXT2_EVENTID,

    /**
     * vdev restart response for multiple vdevs in response to
     * MULTIPLE_VDEV_RESTART_REQUEST
     */
    WMI_PDEV_MULTIPLE_VDEV_RESTART_RESP_EVENTID,

    /** WMI event in response to TPC STATS command */
    WMI_PDEV_GET_TPC_STATS_EVENTID,

    /* Event to get DPD status from HALPHY */
    WMI_PDEV_GET_DPD_STATUS_EVENTID,

    /* Event to get Calibration status from HALPHY */
    WMI_PDEV_GET_HALPHY_CAL_STATUS_EVENTID,

    /* Event to set halphy cal bitmap */
    WMI_PDEV_SET_HALPHY_CAL_BMAP_EVENTID,

    /* Event to get AOA phasedelta values from HALPHY */
    WMI_PDEV_AOA_PHASEDELTA_EVENTID,

    WMI_PDEV_FIPS_EXTEND_EVENTID,

    /* Event to send packet log decode information */
    WMI_PDEV_PKTLOG_DECODE_INFO_EVENTID,

    /**
     * RSSI dB to dBm conversion params info event
     * sent to host after channel/bw/chainmask change per pdev.
     */
    WMI_PDEV_RSSI_DBM_CONVERSION_PARAMS_INFO_EVENTID,

    /* Event to indicate Schedule tid queue suspended info */
    WMI_PDEV_SCHED_TIDQ_SUSP_INFO_EVENTID,

    /* Event to send target rate to power table update status */
    WMI_PDEV_SET_TGTR2P_TABLE_EVENTID,

    /* Event to indicate completion on RF path */
    WMI_PDEV_SET_RF_PATH_RESP_EVENTID,

    /* Event to get AOA phasedelta values for all gain tables from HALPHY */
    WMI_PDEV_ENHANCED_AOA_PHASEDELTA_EVENTID,

    /* VDEV specific events */
    /** VDEV started event in response to VDEV_START request */
    WMI_VDEV_START_RESP_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_VDEV),
    /** vdev stopped event , generated in response to VDEV_STOP request */
    WMI_VDEV_STOPPED_EVENTID,
    /* Indicate the set key (used for setting per
     * peer unicast and per vdev multicast)
     * operation has completed */
    WMI_VDEV_INSTALL_KEY_COMPLETE_EVENTID,
    /* NOTE: WMI_VDEV_MCC_BCN_INTERVAL_CHANGE_REQ_EVENTID would be deprecated. Please
     don't use this for any new implementations */
    /* Firmware requests dynamic change to a specific beacon interval for a specific vdev ID in MCC scenario.
     This request is valid only for vdevs operating in soft AP or P2P GO mode */
    WMI_VDEV_MCC_BCN_INTERVAL_CHANGE_REQ_EVENTID,

    /* Return the TSF timestamp of specified vdev */
    WMI_VDEV_TSF_REPORT_EVENTID,

    /* FW response to Host for vdev delete cmdid */
    WMI_VDEV_DELETE_RESP_EVENTID,

    /* DISA feature: FW response to Host with encrypted/decrypted 802.11 DISA frame */
    WMI_VDEV_ENCRYPT_DECRYPT_DATA_RESP_EVENTID,

    /** event to report mac randomization success **/
    WMI_VDEV_ADD_MAC_ADDR_TO_RX_FILTER_STATUS_EVENTID,

    /* event for ARP stats collection */
    WMI_VDEV_GET_ARP_STAT_EVENTID,

    /** get tx power event in response to VDEV_GET_TX_POWER request */
    WMI_VDEV_GET_TX_POWER_EVENTID,

    WMI_VDEV_BCN_RECEPTION_STATS_EVENTID,

    /* provide LTE-Coex state */
    WMI_VDEV_GET_MWS_COEX_STATE_EVENTID,

    /* provide LTE-Coex Dynamic Power Back-off info */
    WMI_VDEV_GET_MWS_COEX_DPWB_STATE_EVENTID,

    /* provide LTE-Coex TDM info */
    WMI_VDEV_GET_MWS_COEX_TDM_STATE_EVENTID,

    /* provide LTE-Coex IDRx info */
    WMI_VDEV_GET_MWS_COEX_IDRX_STATE_EVENTID,

    /* provide LTE-Coex antenna sharing info */
    WMI_VDEV_GET_MWS_COEX_ANTENNA_SHARING_STATE_EVENTID,

    /* Event to handle FW offloaded mgmt packets */
    WMI_VDEV_MGMT_OFFLOAD_EVENTID,

    /* FW response to Host for delete all peer cmdid */
    WMI_VDEV_DELETE_ALL_PEER_RESP_EVENTID,

    /** Indicates host to start/stop strobing for QTIMER periodically */
    WMI_VDEV_AUDIO_SYNC_START_STOP_EVENTID,
    /** Sends the final offset in the QTIMERs of both master and slave */
    WMI_VDEV_AUDIO_SYNC_Q_MASTER_SLAVE_OFFSET_EVENTID,
    /** VDEV_SEND_BIG_DATA_EVENT IS DEPRECATED - DO NOT USE */
    WMI_VDEV_SEND_BIG_DATA_EVENTID,
    /** send BIG DATA stats to host phase 2 */
    WMI_VDEV_SEND_BIG_DATA_P2_EVENTID,
    /** Latency related information received from beacon IE */
    WMI_VDEV_BCN_LATENCY_EVENTID,
    /** Disconnect request from FW */
    WMI_VDEV_DISCONNECT_EVENTID,
    /** Send Smart Monitor related params to host */
    WMI_VDEV_SMART_MONITOR_EVENTID,
    /** Send status of vdev mac address update request to host */
    WMI_VDEV_UPDATE_MAC_ADDR_CONF_EVENTID,
    /** event to report latency level honored by FW */
    WMI_VDEV_LATENCY_LEVEL_EVENTID,
    /** Result from firmware about completed scheduler probing */
    WMI_VDEV_SCHED_MODE_PROBE_RESP_EVENTID,
    /** Connect response */
    WMI_VDEV_OOB_CONNECTION_RESP_EVENTID,

    /* peer specific events */
    /** FW reauet to kick out the station for reasons like inactivity,lack of response ..etc */
    WMI_PEER_STA_KICKOUT_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_PEER),

    /** Peer Info Event with data_rate, RSSI, tx_fail_cnt etc */
    WMI_PEER_INFO_EVENTID,

    /** Event indicating that TX fail count reaching threshold */
    WMI_PEER_TX_FAIL_CNT_THR_EVENTID,

    /* Return the estimate link speed for the Peer specified in the
     * WMI_PEER_GET_ESTIMATED_LINKSPEED_CMDID command.
     */
    WMI_PEER_ESTIMATED_LINKSPEED_EVENTID,
    /* Return the peer state
     * WMI_PEER_SET_PARAM_CMDID, WMI_PEER_AUTHORIZE
     */
    WMI_PEER_STATE_EVENTID,

    /* Peer Assoc Conf event to confirm fw had received PEER_ASSOC_CMD.
     * After that, host will send Mx message.
     * Otherwise, host will pause any Mx(STA:M2/M4) message
     */
    WMI_PEER_ASSOC_CONF_EVENTID,

    /* FW response to Host for peer delete cmdid */
    WMI_PEER_DELETE_RESP_EVENTID,

    /** Valid rate code list for peer  */
    WMI_PEER_RATECODE_LIST_EVENTID,
    WMI_WDS_PEER_EVENTID,
    WMI_PEER_STA_PS_STATECHG_EVENTID,
    /** Peer Ant Div Info Event with RSSI per chain, etc */
    WMI_PEER_ANTDIV_INFO_EVENTID,

    /*
     * WMI_PEER_RESERVED_EVENTID
     * These values are used for placeholders, to allow the subsequent
     * WMI_PEER_OPER_MODE_CHANGE_EVENTID constant to have the same value
     * as it had in its original location, when it was mistakenly placed
     * amongst the WMI_PEER CMDID defs.
     *
     * These WMI_PEER_RESERVED values can be replaced with actual WMI peer
     * event message IDs, though it will be simpler to instead add new
     * WMI_PEER EVENTID defs at the end of the WMI_GRP_PEER WMI_EVT_GRP.
     */
    WMI_PEER_RESERVED0_EVENTID,
    WMI_PEER_RESERVED1_EVENTID,
    WMI_PEER_RESERVED2_EVENTID,
    WMI_PEER_RESERVED3_EVENTID,
    WMI_PEER_RESERVED4_EVENTID,
    WMI_PEER_RESERVED5_EVENTID,
    WMI_PEER_RESERVED6_EVENTID,
    WMI_PEER_RESERVED7_EVENTID,
    WMI_PEER_RESERVED8_EVENTID,
    WMI_PEER_RESERVED9_EVENTID,
    WMI_PEER_RESERVED10_EVENTID,
    /** Peer operating mode change indication sent to host to update stats */
    WMI_PEER_OPER_MODE_CHANGE_EVENTID,

    /** report the current tx PN for the peer */
    WMI_PEER_TX_PN_RESPONSE_EVENTID,

    WMI_PEER_CFR_CAPTURE_EVENTID,

    /* Peer Create Conf event to confirm fw had received WMI_PEER_CREATE_CMDID
     * and status of WMI_PEER_CREATE_CMDID.
     */
    WMI_PEER_CREATE_CONF_EVENTID,

    WMI_PEER_RX_PN_RESPONSE_EVENTID,


    /* beacon/mgmt specific events */
    /** RX management frame. the entire frame is carried along with the event.  */
    WMI_MGMT_RX_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_MGMT),
    /** software beacon alert event to Host requesting host to Queue a beacon for transmission
     use only in host beacon mode */
    WMI_HOST_SWBA_EVENTID,
    /** beacon tbtt offset event indicating the tsf offset of the tbtt from the theoretical value.
     tbtt offset is normally 0 and will be non zero if there are multiple VDEVs operating in
     staggered beacon transmission mode */
    WMI_TBTTOFFSET_UPDATE_EVENTID,

    /** event after the first beacon is transmitted following
             a change in the template.*/
    WMI_OFFLOAD_BCN_TX_STATUS_EVENTID,
    /** event after the first probe response is transmitted following
             a change in the template.*/
    WMI_OFFLOAD_PROB_RESP_TX_STATUS_EVENTID,
    /** Event for Mgmt TX completion event */
    WMI_MGMT_TX_COMPLETION_EVENTID,
    /** Event for Mgmt TX bundle completion event */
    WMI_MGMT_TX_BUNDLE_COMPLETION_EVENTID,
    /** vdev_map used in WMI_TBTTOFFSET_UPDATE_EVENTID supports max 32 vdevs.
     *   Use this event if number of vdevs > 32.
     */
    WMI_TBTTOFFSET_EXT_UPDATE_EVENTID,
    /** Event for offchan data TX completion event */
    WMI_OFFCHAN_DATA_TX_COMPLETION_EVENTID,

    /** software FILS Discovery Frame alert event to Host, requesting host to Queue an FD frame for transmission */
    WMI_HOST_SWFDA_EVENTID,

    /** software beacon alert event to Host requesting host to Queue a beacon for transmission.
     *   Used only in host beacon mode. */
    WMI_HOST_SWBA_V2_EVENTID,

    /** Event for QoS null frame TX completion  */
    WMI_QOS_NULL_FRAME_TX_COMPLETION_EVENTID,

    /** WMI event for Firmware Consumed/Dropped Rx management frames indication */
    WMI_MGMT_RX_FW_CONSUMED_EVENTID,


    /* ADDBA Related WMI Events*/
    /** Indication the completion of the prior
     WMI_PEER_TID_DELBA_CMDID(initiator) */
    WMI_TX_DELBA_COMPLETE_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_BA_NEG),
    /** Indication the completion of the prior
     *WMI_PEER_TID_ADDBA_CMDID(initiator) */
    WMI_TX_ADDBA_COMPLETE_EVENTID,

    /* Seq num returned from hw for a sta/tid pair */
    WMI_BA_RSP_SSN_EVENTID,

    /* Aggregation state requested by BTC */
    WMI_AGGR_STATE_TRIG_EVENTID,

    /** Roam event to trigger roaming on host */
    WMI_ROAM_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_ROAM),

    /** matching AP found from list of profiles */
    WMI_PROFILE_MATCH,
    /** roam synch event */
    WMI_ROAM_SYNCH_EVENTID,
    /** roam synch frame event */
    WMI_ROAM_SYNCH_FRAME_EVENTID,
    /** various roam scan stats */
    WMI_ROAM_SCAN_STATS_EVENTID,
    /** Blacklisted AP information event */
    WMI_ROAM_BLACKLIST_EVENTID,
    /** Roam Pre-Authentication start event */
    WMI_ROAM_PREAUTH_START_EVENTID,
    /** Roaming PMKID request event */
    WMI_ROAM_PMKID_REQUEST_EVENTID,
    /** roam stats */
    WMI_ROAM_STATS_EVENTID,
    /** Roam scan channels list */
    WMI_ROAM_SCAN_CHANNEL_LIST_EVENTID,
    /** Firmware roam capability information */
    WMI_ROAM_CAPABILITY_REPORT_EVENTID,
    /** Send AP frame content like beacon/probe resp etc.. */
    WMI_ROAM_FRAME_EVENTID,
    /** Send firmware ini value corresponding to param_id */
    WMI_ROAM_GET_VENDOR_CONTROL_PARAM_EVENTID,
    /** roam synch key event */
    WMI_ROAM_SYNCH_KEY_EVENTID,

    /** P2P disc found */
    WMI_P2P_DISC_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_P2P),
    /** send noa info to host when noa is changed for beacon tx offload enable */
    WMI_P2P_NOA_EVENTID,
    /** send p2p listen offload stopped event with different reason */
    WMI_P2P_LISTEN_OFFLOAD_STOPPED_EVENTID,

    /** Send EGAP Info to host */
    WMI_AP_PS_EGAP_INFO_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_AP_PS),

    /* send pdev resume event to host after pdev resume. */
    WMI_PDEV_RESUME_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_SUSPEND),

    /** WOW wake up host event.generated in response to WMI_WOW_HOSTWAKEUP_FROM_SLEEP_CMDID.
     will cary wake reason */
    WMI_WOW_WAKEUP_HOST_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_WOW),
    WMI_D0_WOW_DISABLE_ACK_EVENTID,
    WMI_WOW_INITIAL_WAKEUP_EVENTID,
    WMI_WOW_COAP_BUF_INFO_EVENTID,

    /* RTT related event ID */
    /** RTT measurement report */
    WMI_RTT_MEASUREMENT_REPORT_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_RTT),
    /** TSF measurement report */
    WMI_TSF_MEASUREMENT_REPORT_EVENTID,
    /** RTT error report */
    WMI_RTT_ERROR_REPORT_EVENTID,
    /** RTT 11az PASN peer create request */
    WMI_RTT_PASN_PEER_CREATE_REQ_EVENTID,
    /** RTT 11az PASN peer delete event */
    WMI_RTT_PASN_PEER_DELETE_EVENTID,

    /*STATS specific events*/
    /** txrx stats event requested by host */
    WMI_STATS_EXT_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_STATS),
    /** FW iface link stats Event  */
    WMI_IFACE_LINK_STATS_EVENTID,
    /** FW iface peer link stats Event  */
    WMI_PEER_LINK_STATS_EVENTID,
    /** FW Update radio stats Event  */
    WMI_RADIO_LINK_STATS_EVENTID,

    /**  Firmware memory dump Complete event*/
    WMI_UPDATE_FW_MEM_DUMP_EVENTID,

    /** Event indicating the DIAG logs/events supported by FW */
    WMI_DIAG_EVENT_LOG_SUPPORTED_EVENTID,

    /** Instantaneous RSSI event */
    WMI_INST_RSSI_STATS_EVENTID,

    /** FW update tx power levels event */
    WMI_RADIO_TX_POWER_LEVEL_STATS_EVENTID,

    /** This event is used to report wlan stats to host.
     * It is triggered under 3 conditions:
     * (a) Periodic timer timed out, based on the period specified
     *     by WMI_PDEV_PARAM_STATS_OBSERVATION_PERIOD
     * (b) Whenever any of the (enabled) stats thresholds specified
     *     in the WMI_PDEV_SET_STATS_THRESHOLD_CMD message is exceeded
     *     within the current stats period.
     * (c) In response to the one-time wlan stats request of
     *     WMI_REQUEST_WLAN_STATS_CMDID from host.
     *
     *  If this event is triggered by condition a or b,
     *  the stats counters are cleared at the start of each period.
     *  But if it is triggered by condition c, stats counters won't be cleared.
     */
    WMI_REPORT_STATS_EVENTID,

    /** Event indicating RCPI of the peer requested by host in the WMI_REQUEST_RCPI_CMDID */
    WMI_UPDATE_RCPI_EVENTID,

    /** This event is used to respond to WMI_REQUEST_PEER_STATS_INFO_CMDID
     *  and report peer stats info to host */
    WMI_PEER_STATS_INFO_EVENTID,

    /** This event is used to respond to WMI_REQUEST_RADIO_CHAN_STATS_CMDID
     *  and report radio channel stats to host */
    WMI_RADIO_CHAN_STATS_EVENTID,

    /** This event is used to respond to WMI_REQUEST_WLM_STATS_CMDID
     *  and report WLM (WLAN latency manager) stats info to host */
    WMI_WLM_STATS_EVENTID,

    /** This event is used to respond to WMI_REQUEST_CTRL_PATH_STATS_CMDID
     *  and report stats info to host */
    WMI_CTRL_PATH_STATS_EVENTID,

    /** This event is used to respond to
     * WMI_REQUEST_HALPHY_CTRL_PATH_STATS_CMDID and report stats info to host
     */
    WMI_HALPHY_CTRL_PATH_STATS_EVENTID,

    /** FW IPA link stats Event */
    WMI_IPA_LINK_STATS_EVENTID,


    /* NLO specific events */
    /** NLO match event after the first match */
    WMI_NLO_MATCH_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_NLO_OFL),

    /** NLO scan complete event */
    WMI_NLO_SCAN_COMPLETE_EVENTID,

    /** APFIND specific events */
    WMI_APFIND_EVENTID,

    /** passpoint network match event */
    WMI_PASSPOINT_MATCH_EVENTID,

    /** GTK offload stautus event requested by host */
    WMI_GTK_OFFLOAD_STATUS_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_GTK_OFL),

    /** GTK offload failed to rekey event */
    WMI_GTK_REKEY_FAIL_EVENTID,

    /* CSA handling event */
    WMI_CSA_HANDLING_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_CSA_OFL),
    /* CSA IE received event */
    WMI_CSA_IE_RECEIVED_EVENTID,

    /*chatter query reply event*/
    WMI_CHATTER_PC_QUERY_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_CHATTER),

    /** DFS related events */
    WMI_PDEV_DFS_RADAR_DETECTION_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_DFS),
    /** Indicate channel-availability-check completion event to host */
    WMI_VDEV_DFS_CAC_COMPLETE_EVENTID,
    /** Indicate off-channel-availability-check completion event to host */
    WMI_VDEV_ADFS_OCAC_COMPLETE_EVENTID,

    /** echo event in response to echo command */
    WMI_ECHO_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_MISC),

    /* !!IMPORTANT!!
     * If you need to add a new WMI event ID to the WMI_GRP_MISC sub-group,
     * please make sure you add it BEHIND WMI_PDEV_UTF_EVENTID,
     * as we MUST have a fixed value here to maintain compatibility between
     * UTF and the ART2 driver
     */
    /** UTF specific WMI event */
    WMI_PDEV_UTF_EVENTID,

    /** event carries buffered debug messages  */
    WMI_DEBUG_MESG_EVENTID,
    /** FW stats(periodic or on shot)  */
    WMI_UPDATE_STATS_EVENTID,
    /** debug print message used for tracing FW code while debugging  */
    WMI_DEBUG_PRINT_EVENTID,
    /** DCS wlan or non-wlan interference event
     */
    WMI_DCS_INTERFERENCE_EVENTID,
    /** VI spoecific event  */
    WMI_PDEV_QVIT_EVENTID,
    /** FW code profile data in response to profile request  */
    WMI_WLAN_PROFILE_DATA_EVENTID,
    /* Factory Testing Mode request event
     * used for integrated chipsets */
    WMI_PDEV_FTM_INTG_EVENTID,
    /* avoid list of frequencies .
     */
    WMI_WLAN_FREQ_AVOID_EVENTID,
    /* Indicate the keepalive parameters */
    WMI_VDEV_GET_KEEPALIVE_EVENTID,
    /*Thermal Management event*/
    WMI_THERMAL_MGMT_EVENTID,

    /* Container for DIAG event and log data */
    WMI_DIAG_DATA_CONTAINER_EVENTID,

    /* host auto shutdown event */
    WMI_HOST_AUTO_SHUTDOWN_EVENTID,

    /*update mib counters together with WMI_UPDATE_STATS_EVENTID*/
    WMI_UPDATE_WHAL_MIB_STATS_EVENTID,

    /*update ht/vht info based on vdev (rx and tx NSS and preamble)*/
    WMI_UPDATE_VDEV_RATE_STATS_EVENTID,

    WMI_DIAG_EVENTID,

    /** Set OCB Sched Response, deprecated */
    WMI_OCB_SET_SCHED_EVENTID,

    /** event to indicate the flush of the buffered debug messages is complete*/
    WMI_DEBUG_MESG_FLUSH_COMPLETE_EVENTID,

    /** event to report mix/max RSSI breach events */
    WMI_RSSI_BREACH_EVENTID,

    /** event to report completion of data storage into flash memory */
    WMI_TRANSFER_DATA_TO_FLASH_COMPLETE_EVENTID,

    /** event to report SCPC calibrated data to host */
    WMI_PDEV_UTF_SCPC_EVENTID,

    /** event to provide requested data from the target's flash memory */
    WMI_READ_DATA_FROM_FLASH_EVENTID,

    /** event to report rx aggregation failure frame information */
    WMI_REPORT_RX_AGGR_FAILURE_EVENTID,

    /** event to upload a PKGID to host to identify chip for various products */
    WMI_PKGID_EVENTID,

    /* Thermal Throttling stats event id for every pdev and zones, etc */
    WMI_THERM_THROT_STATS_EVENTID,

    /* WMI UNIT TEST event */
    WMI_UNIT_TEST_EVENTID,

    /** event to report result of host configure SAR2 */
    WMI_SAR2_RESULT_EVENTID,

    /** event to get TX power per input HALPHY parameters */
    WMI_GET_TPC_POWER_EVENTID,

    /** event to provide MU-EDCA Parameters (to update host's beacon config) */
    WMI_MUEDCA_PARAMS_CONFIG_EVENTID,

    /** event to get ELNA BYPASS status */
    WMI_GET_ELNA_BYPASS_EVENTID,

    /** event to report ANI level of the channels */
    WMI_GET_CHANNEL_ANI_EVENTID,

    /* WMI event to available scratch registers */
    WMI_PMM_AVAILABLE_SCRATCH_REG_EVENTID,

    /* WMI event to scratch registers allocation */
    WMI_PMM_SCRATCH_REG_ALLOCATION_COMPLETE_EVENTID,

    /* WMI event to indicate Helath Monitor Infra init done */
    WMI_HEALTH_MON_INIT_DONE_EVENTID,

    /* WMI XGAP enable command response event ID */
    WMI_XGAP_ENABLE_COMPLETE_EVENTID,

    /* T2H HPA message */
    WMI_HPA_EVENTID,

    /* WMI standalone command complete Event */
    WMI_VDEV_STANDALONE_SOUND_COMPLETE_EVENTID,

    /* WMI evt to indicate switch type either to WLAN(XPAN) or non_WLAN(BLE) */
    WMI_AUDIO_TRANSPORT_SWITCH_TYPE_EVENTID,


    /* GPIO Event */
    WMI_GPIO_INPUT_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_GPIO),
    /** upload H_CV info WMI event
     * to indicate uploaded H_CV info to host
     */
    WMI_UPLOADH_EVENTID,

    /** capture H info WMI event
     * to indicate captured H info to host
     */
    WMI_CAPTUREH_EVENTID,
    /* hw RFkill */
    WMI_RFKILL_STATE_CHANGE_EVENTID,

    /* Smart Antenna Controller status */
    WMI_SMARTANT_STATE_CHANGE_EVENTID,

    WMI_GPIO_STATE_RES_EVENTID,

    /* TDLS Event */
    WMI_TDLS_PEER_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_TDLS),

    /* Resmgr Event */
    /* deliver the new channel time quota for home channels */
    WMI_RESMGR_CHAN_TIME_QUOTA_CHANGED_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_RESMGR),

    /** STA SMPS Event */
    /** force SMPS mode */
    WMI_STA_SMPS_FORCE_MODE_COMPLETE_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_STA_SMPS),

    /*location scan event*/
    /*report the firmware's capability of batch scan*/
    WMI_BATCH_SCAN_ENABLED_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_LOCATION_SCAN),
    /*batch scan result*/
    WMI_BATCH_SCAN_RESULT_EVENTID,
    /* OEM Event */
    WMI_OEM_CAPABILITY_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_OEM), /*DEPRECATED*/
    WMI_OEM_MEASUREMENT_REPORT_EVENTID, /* DEPRECATED */
    WMI_OEM_ERROR_REPORT_EVENTID, /* DEPRECATED */
    WMI_OEM_RESPONSE_EVENTID,
    WMI_OEM_DMA_RING_CFG_RSP_EVENTID,
    WMI_OEM_DMA_BUF_RELEASE_EVENTID,
    WMI_OEM_DATA_EVENTID,

    /* NAN Event */
    WMI_NAN_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_NAN),
    WMI_NAN_DISC_IFACE_CREATED_EVENTID,
    WMI_NAN_DISC_IFACE_DELETED_EVENTID,
    WMI_NAN_STARTED_CLUSTER_EVENTID,
    WMI_NAN_JOINED_CLUSTER_EVENTID,
    WMI_NAN_DMESG_EVENTID,
    /** Event to deliver OEM's NAN specific opaque data */
    WMI_NAN_OEM_DATA_EVENTID,

    /* Coex Event */
    WMI_COEX_REPORT_ANTENNA_ISOLATION_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_COEX),
    WMI_SAR_GET_LIMITS_EVENTID,
    /** Dedicated BT Antenna Mode (DBAM) complete event */
    WMI_COEX_DBAM_COMPLETE_EVENTID,
    WMI_TAS_POWER_HISTORY_EVENTID,

    /* LPI Event */
    WMI_LPI_RESULT_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_LPI),
    WMI_LPI_STATUS_EVENTID,
    WMI_LPI_HANDOFF_EVENTID,

    /* ExtScan events */
    WMI_EXTSCAN_START_STOP_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_EXTSCAN),
    WMI_EXTSCAN_OPERATION_EVENTID,
    WMI_EXTSCAN_TABLE_USAGE_EVENTID,
    WMI_EXTSCAN_CACHED_RESULTS_EVENTID,
    WMI_EXTSCAN_WLAN_CHANGE_RESULTS_EVENTID,
    WMI_EXTSCAN_HOTLIST_MATCH_EVENTID,
    WMI_EXTSCAN_CAPABILITIES_EVENTID,
    WMI_EXTSCAN_HOTLIST_SSID_MATCH_EVENTID,

    /* mDNS offload events */
    WMI_MDNS_STATS_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_MDNS_OFL),

    /* SAP Authentication offload events */
    WMI_SAP_OFL_ADD_STA_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_SAP_OFL),
    WMI_SAP_OFL_DEL_STA_EVENTID,
    WMI_SAP_OBSS_DETECTION_REPORT_EVENTID,

    /* OBSS Offloads events */
    WMI_OBSS_COLOR_COLLISION_DETECTION_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_OBSS_OFL),

    /** Out-of-context-of-bss (OCB) events */
    WMI_OCB_SET_CONFIG_RESP_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_OCB),
    WMI_OCB_GET_TSF_TIMER_RESP_EVENTID,
    WMI_DCC_GET_STATS_RESP_EVENTID,
    WMI_DCC_UPDATE_NDL_RESP_EVENTID,
    WMI_DCC_STATS_EVENTID,

    /* System-On-Chip events */
    WMI_SOC_SET_HW_MODE_RESP_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_SOC),
    WMI_SOC_HW_MODE_TRANSITION_EVENTID,
    WMI_SOC_SET_DUAL_MAC_CONFIG_RESP_EVENTID,

    /** Motion Aided WiFi Connectivity (MAWC) events */
    WMI_MAWC_ENABLE_SENSOR_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_MAWC),

    /** pkt filter (BPF) offload relevant events */
    WMI_BPF_CAPABILIY_INFO_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_BPF_OFFLOAD),
    WMI_BPF_VDEV_STATS_INFO_EVENTID,
    WMI_BPF_GET_VDEV_WORK_MEMORY_RESP_EVENTID,

    /* RMC specific event */
    /* RMC manual leader selected event */
    WMI_RMC_NEW_LEADER_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_RMC),

    /** WMI events related to regulatory offload */
    WMI_REG_CHAN_LIST_CC_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_REGULATORY),
    WMI_11D_NEW_COUNTRY_EVENTID,
    WMI_REG_CHAN_LIST_CC_EXT_EVENTID,
    WMI_AFC_EVENTID,

    /** Events for TWT(Target Wake Time) of STA and AP  */
    WMI_TWT_ENABLE_COMPLETE_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_TWT),
    WMI_TWT_DISABLE_COMPLETE_EVENTID,
    WMI_TWT_ADD_DIALOG_COMPLETE_EVENTID,
    WMI_TWT_DEL_DIALOG_COMPLETE_EVENTID,
    WMI_TWT_PAUSE_DIALOG_COMPLETE_EVENTID,
    WMI_TWT_RESUME_DIALOG_COMPLETE_EVENTID,
    WMI_TWT_BTWT_INVITE_STA_COMPLETE_EVENTID,
    WMI_TWT_BTWT_REMOVE_STA_COMPLETE_EVENTID,
    WMI_TWT_SESSION_STATS_EVENTID,
    WMI_TWT_NUDGE_DIALOG_COMPLETE_EVENTID,
    WMI_TWT_NOTIFY_EVENTID,
    WMI_TWT_ACK_EVENTID,

    /** Events in Prototyping phase */
    WMI_NDI_CAP_RSP_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_PROTOTYPE),
    WMI_NDP_INITIATOR_RSP_EVENTID,
    WMI_NDP_RESPONDER_RSP_EVENTID,
    WMI_NDP_END_RSP_EVENTID,
    WMI_NDP_INDICATION_EVENTID,
    WMI_NDP_CONFIRM_EVENTID,
    WMI_NDP_END_INDICATION_EVENTID,
    WMI_WLAN_COEX_BT_ACTIVITY_EVENTID,
    WMI_NDL_SCHEDULE_UPDATE_EVENTID,
    WMI_NDP_EVENTID,

    /** WMI events related to motion detection */
    WMI_MOTION_DET_HOST_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_MOTION_DET),
    WMI_MOTION_DET_BASE_LINE_HOST_EVENTID,

    /** WMI events related to Estimation of Service Parameters (802.11mc) */
    WMI_ESP_ESTIMATE_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_ESP),

    /** WMI events related to Audio Frame aggregation feature **/
    WMI_AUDIO_AGGR_REPORT_STATISTICS_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_AUDIO),
    WMI_AUDIO_AGGR_SCHED_METHOD_EVENTID,

    /** Vendor defined WMI events **/
    WMI_VENDOR_PDEV_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_VENDOR),
    WMI_VENDOR_VDEV_EVENTID,
    WMI_VENDOR_PEER_EVENTID,
    /** Further vendor event IDs can be added below **/

    /** WMI event specific to MLO **/
    /** MLO link active / inactive response event */
    WMI_MLO_LINK_SET_ACTIVE_RESP_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_MLO),
    /* Response event for MLO setup cmd */
    WMI_MLO_SETUP_COMPLETE_EVENTID,
    /* Response event for MLO teardown cmd */
    WMI_MLO_TEARDOWN_COMPLETE_EVENTID,
    /* Response event for Link Removal Cmd */
    WMI_MLO_LINK_REMOVAL_EVENTID,
    /* Response event for WMI_MLO_AP_VDEV_TID_TO_LINK_MAP_CMDID */
    WMI_MLO_AP_VDEV_TID_TO_LINK_MAP_EVENTID,
    /* Response event for WMI_MLO_VDEV_GET_LINK_INFO_CMDID */
    WMI_MLO_VDEV_LINK_INFO_EVENTID,
    /** request host to do T2LM neg to the un-disabled link */
    WMI_MLO_LINK_DISABLE_REQUEST_EVENTID,
    /** request host to switch to new link for specified vdev */
    WMI_MLO_LINK_SWITCH_REQUEST_EVENTID,
    /** Response event for WMI_MLO_PRIMARY_LINK_PEER_MIGRATION_CMDID */
    WMI_MLO_PRIMARY_LINK_PEER_MIGRATION_EVENTID,
    /** WMI Event to spcify reason for link state switch */
    WMI_MLO_LINK_STATE_SWITCH_EVENTID,

    /* WMI event specific to Quiet handling */
    WMI_QUIET_HANDLING_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_QUIET_OFL),

    /* ODD events */
    WMI_ODD_LIVEDUMP_RESPONSE_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_ODD),

    /** WMI events specific to manually-triggered UL */
    /**
     * WMI Event to send Manual UL OFDMA Trigger frame status feedback to Host
     */
    WMI_MANUAL_UL_OFDMA_TRIG_FEEDBACK_EVENTID = WMI_EVT_GRP_START_ID(WMI_GRP_MANUAL_UL_TRIG),
    /**
     * WMI Event to send Manual UL OFDMA Trigger frame RX PPDU info to Host
     */
    WMI_MANUAL_UL_OFDMA_TRIG_RX_PEER_USERINFO_EVENTID,
} WMI_EVT_ID;

/* defines for OEM message sub-types */
#define WMI_OEM_CAPABILITY_REQ     0x01
#define WMI_OEM_CAPABILITY_RSP     0x02
#define WMI_OEM_MEASUREMENT_REQ    0x03
#define WMI_OEM_MEASUREMENT_RSP    0x04
#define WMI_OEM_ERROR_REPORT_RSP   0x05
#define WMI_OEM_NAN_MEAS_REQ       0x06
#define WMI_OEM_NAN_MEAS_RSP       0x07
#define WMI_OEM_NAN_PEER_INFO      0x08
#define WMI_OEM_CONFIGURE_LCR      0x09
#define WMI_OEM_CONFIGURE_LCI      0x0A


#define WMI_CHAN_LIST_TAG                0x1
#define WMI_SSID_LIST_TAG                0x2
#define WMI_BSSID_LIST_TAG               0x3
#define WMI_IE_TAG                       0x4
#define WMI_SCAN_START_OFFSET_TAG        0x5

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_channel */
    /** primary 20 MHz channel frequency in mhz */
    A_UINT32 mhz;
    /** Center frequency 1 in MHz*/
    A_UINT32 band_center_freq1;
    /** Center frequency 2 in MHz - valid only for 11acvht 80plus80 mode*/
    A_UINT32 band_center_freq2;
    /** channel info described below */
    A_UINT32 info;
    /** contains min power, max power, reg power and reg class id.  */
    A_UINT32 reg_info_1;
    /** contains antennamax, max bandwidth */
    A_UINT32 reg_info_2;
} wmi_channel;

typedef enum {
    WMI_CHANNEL_CHANGE_CAUSE_NONE = 0,
    WMI_CHANNEL_CHANGE_CAUSE_CSA,
} wmi_channel_change_cause;

/** channel info consists of 6 bits of channel mode */

#define WMI_SET_CHANNEL_MODE(pwmi_channel,val) do { \
     (pwmi_channel)->info &= 0xffffffc0;            \
     (pwmi_channel)->info |= (val);                 \
     } while (0)

#define WMI_GET_CHANNEL_MODE(pwmi_channel) ((pwmi_channel)->info & 0x0000003f)

#define WMI_CHAN_FLAG_HT40_PLUS   6
#define WMI_CHAN_FLAG_PASSIVE     7
#define WMI_CHAN_ADHOC_ALLOWED    8
#define WMI_CHAN_AP_DISABLED      9
#define WMI_CHAN_FLAG_DFS         10
#define WMI_CHAN_FLAG_ALLOW_HT    11  /* HT is allowed on this channel */
#define WMI_CHAN_FLAG_ALLOW_VHT   12  /* VHT is allowed on this channel */
#define WMI_CHANNEL_CHANGE_CAUSE_CSA 13 /*Indicate reason for channel switch */
#define WMI_CHAN_FLAG_HALF_RATE     14  /* Indicates half rate channel */
#define WMI_CHAN_FLAG_QUARTER_RATE  15  /* Indicates quarter rate channel */
#define WMI_CHAN_FLAG_DFS_CFREQ2  16 /* Enable radar event reporting for sec80 in VHT80p80 */
#define WMI_CHAN_FLAG_ALLOW_HE    17 /* HE (11ax) is allowed on this channel */
#define WMI_CHAN_FLAG_PSC         18 /* Indicate it is a PSC (preferred scanning channel) */
#define WMI_CHAN_FLAG_NAN_DISABLED 19 /* Indicates that NAN operations are disabled on this channel */
#define WMI_CHAN_FLAG_STA_DFS     20 /* Indicates if STA should process radar signals */
#define WMI_CHAN_FLAG_ALLOW_EHT   21 /* EHT (11be) is allowed on this channel */

#define WMI_SET_CHANNEL_FLAG(pwmi_channel,flag) do { \
        (pwmi_channel)->info |=  ((A_UINT32) 1 << flag);      \
     } while (0)

#define WMI_GET_CHANNEL_FLAG(pwmi_channel,flag)   \
        (((pwmi_channel)->info & ((A_UINT32) 1 << flag)) >> flag)

#define WMI_SET_CHANNEL_MIN_POWER(pwmi_channel,val) do { \
     (pwmi_channel)->reg_info_1 &= 0xffffff00;           \
     (pwmi_channel)->reg_info_1 |= (val & 0xff);         \
     } while (0)
#define WMI_GET_CHANNEL_MIN_POWER(pwmi_channel) ((pwmi_channel)->reg_info_1 & 0xff)

#define WMI_SET_CHANNEL_MAX_POWER(pwmi_channel,val) do { \
     (pwmi_channel)->reg_info_1 &= 0xffff00ff;           \
     (pwmi_channel)->reg_info_1 |= ((val & 0xff) << 8);  \
     } while (0)
#define WMI_GET_CHANNEL_MAX_POWER(pwmi_channel) ((((pwmi_channel)->reg_info_1) >> 8) & 0xff)

#define WMI_SET_CHANNEL_REG_POWER(pwmi_channel,val) do { \
     (pwmi_channel)->reg_info_1 &= 0xff00ffff;           \
     (pwmi_channel)->reg_info_1 |= ((val & 0xff) << 16); \
     } while (0)
#define WMI_GET_CHANNEL_REG_POWER(pwmi_channel) ((((pwmi_channel)->reg_info_1) >> 16) & 0xff)
#define WMI_SET_CHANNEL_REG_CLASSID(pwmi_channel,val) do { \
     (pwmi_channel)->reg_info_1 &= 0x00ffffff;             \
     (pwmi_channel)->reg_info_1 |= ((val & 0xff) << 24);   \
     } while (0)
#define WMI_GET_CHANNEL_REG_CLASSID(pwmi_channel) ((((pwmi_channel)->reg_info_1) >> 24) & 0xff)

#define WMI_SET_CHANNEL_ANTENNA_MAX(pwmi_channel,val) do { \
     (pwmi_channel)->reg_info_2 &= 0xffffff00;             \
     (pwmi_channel)->reg_info_2 |= (val & 0xff);           \
     } while (0)
#define WMI_GET_CHANNEL_ANTENNA_MAX(pwmi_channel) ((pwmi_channel)->reg_info_2 & 0xff)

/* max tx power is in 1 dBm units */
#define WMI_SET_CHANNEL_MAX_TX_POWER(pwmi_channel,val) do { \
     (pwmi_channel)->reg_info_2 &= 0xffff00ff;              \
     (pwmi_channel)->reg_info_2 |= ((val & 0xff) << 8);     \
     } while (0)
#define WMI_GET_CHANNEL_MAX_TX_POWER(pwmi_channel) ((((pwmi_channel)->reg_info_2)>>8) & 0xff)

/* max bw supported for each channel, enum wmi_channel_width as value */
#define WMI_SET_CHANNEL_MAX_BANDWIDTH(pwmi_channel,val) do { \
     (pwmi_channel)->reg_info_2 &= 0xff00ffff;              \
     (pwmi_channel)->reg_info_2 |= ((val & 0xff) << 16);     \
     } while (0)
#define WMI_GET_CHANNEL_MAX_BANDWIDTH(pwmi_channel) ((((pwmi_channel)->reg_info_2) >> 16) & 0xff)

/** HT Capabilities*/
#define WMI_HT_CAP_ENABLED                0x0001   /* HT Enabled/ disabled */
#define WMI_HT_CAP_HT20_SGI               0x0002   /* Short Guard Interval with HT20 */
#define WMI_HT_CAP_DYNAMIC_SMPS           0x0004   /* Dynamic MIMO powersave */
#define WMI_HT_CAP_TX_STBC                0x0008   /* B3 TX STBC */
#define WMI_HT_CAP_TX_STBC_MASK_SHIFT     3
#define WMI_HT_CAP_RX_STBC                0x0030   /* B4-B5 RX STBC */
#define WMI_HT_CAP_RX_STBC_MASK_SHIFT     4
#define WMI_HT_CAP_LDPC                   0x0040   /* LDPC supported */
#define WMI_HT_CAP_L_SIG_TXOP_PROT        0x0080   /* L-SIG TXOP Protection */
#define WMI_HT_CAP_MPDU_DENSITY           0x0700   /* MPDU Density */
#define WMI_HT_CAP_MPDU_DENSITY_MASK_SHIFT 8
#define WMI_HT_CAP_HT40_SGI               0x0800
#define WMI_HT_CAP_RX_LDPC                0x1000   /* LDPC RX support */
#define WMI_HT_CAP_TX_LDPC                0x2000   /* LDPC TX support */


/* These macros should be used when we wish to advertise STBC support for
 * only 1SS or 2SS or 3SS. */
#define WMI_HT_CAP_RX_STBC_1SS            0x0010   /* B4-B5 RX STBC */
#define WMI_HT_CAP_RX_STBC_2SS            0x0020   /* B4-B5 RX STBC */
#define WMI_HT_CAP_RX_STBC_3SS            0x0030   /* B4-B5 RX STBC */


#define WMI_HT_CAP_DEFAULT_ALL (WMI_HT_CAP_ENABLED       | \
                                WMI_HT_CAP_HT20_SGI      | \
                                WMI_HT_CAP_HT40_SGI      | \
                                WMI_HT_CAP_TX_STBC       | \
                                WMI_HT_CAP_RX_STBC       | \
                                WMI_HT_CAP_LDPC          | \
                                WMI_HT_CAP_TX_LDPC       | \
                                WMI_HT_CAP_RX_LDPC)

/* WMI_VHT_CAP_* these maps to ieee 802.11ac vht capability information
 field. The fields not defined here are not supported, or reserved.
 Do not change these masks and if you have to add new one follow the
 bitmask as specified by 802.11ac draft.
 */


#define WMI_VHT_CAP_MAX_MPDU_LEN_7935            0x00000001
#define WMI_VHT_CAP_MAX_MPDU_LEN_11454           0x00000002
#define WMI_VHT_CAP_MAX_MPDU_LEN_MASK            0x00000003
#define WMI_VHT_CAP_CH_WIDTH_160MHZ              0x00000004
#define WMI_VHT_CAP_CH_WIDTH_80P80_160MHZ        0x00000008
#define WMI_VHT_CAP_RX_LDPC                      0x00000010
#define WMI_VHT_CAP_SGI_80MHZ                    0x00000020
#define WMI_VHT_CAP_SGI_160MHZ                   0x00000040
#define WMI_VHT_CAP_TX_STBC                      0x00000080
#define WMI_VHT_CAP_RX_STBC_MASK                 0x00000300
#define WMI_VHT_CAP_RX_STBC_MASK_SHIFT           8
#define WMI_VHT_CAP_SU_BFORMER                   0x00000800
#define WMI_VHT_CAP_SU_BFORMEE                   0x00001000
#define WMI_VHT_CAP_MAX_CS_ANT_MASK              0x0000E000
#define WMI_VHT_CAP_MAX_CS_ANT_MASK_SHIFT        13
#define WMI_VHT_CAP_MAX_SND_DIM_MASK             0x00070000
#define WMI_VHT_CAP_MAX_SND_DIM_MASK_SHIFT       16
#define WMI_VHT_CAP_MU_BFORMER                   0x00080000
#define WMI_VHT_CAP_MU_BFORMEE                   0x00100000
#define WMI_VHT_CAP_TXOP_PS                      0x00200000
#define WMI_VHT_CAP_MAX_AMPDU_LEN_EXP            0x03800000
#define WMI_VHT_CAP_MAX_AMPDU_LEN_EXP_SHIFT      23
#define WMI_VHT_CAP_RX_FIXED_ANT                 0x10000000
#define WMI_VHT_CAP_TX_FIXED_ANT                 0x20000000
#define WMI_VHT_EXTENDED_NSS_BW_MASK             0xC0000000
#define WMI_VHT_EXTENDED_NSS_BW_MASK_SHIFT       30

/* TEMPORARY:
 * Preserve the incorrect old name as an alias for the correct new name
 * until all references to the old name have been removed from all hosts
 * and targets.
 */
#define WMI_VHT_CAP_MAX_AMPDU_LEN_EXP_SHIT WMI_VHT_CAP_MAX_AMPDU_LEN_EXP_SHIFT


/* These macros should be used when we wish to advertise STBC support for
 * only 1SS or 2SS or 3SS. */
#define WMI_VHT_CAP_RX_STBC_1SS 0x00000100
#define WMI_VHT_CAP_RX_STBC_2SS 0x00000200
#define WMI_VHT_CAP_RX_STBC_3SS 0x00000300

/* TEMPORARY:
 * Preserve the incorrect old name as an alias for the correct new name
 * until all references to the old name have been removed from all hosts
 * and targets.
 */
#define WMI_vHT_CAP_RX_STBC_3SS WMI_VHT_CAP_RX_STBC_3SS

/* TEMPORARY:
 * Spec does not have VHT TX LDPC capability bit. To Maintain backward
 * compatibility due to previous incorrect definition, the value is moved
 * from 0x4 to 0x0. No new use of WMI_VHT_CAP_TX_LDPC should be added.
 */
#define WMI_VHT_CAP_TX_LDPC 0x0


#define WMI_VHT_CAP_DEFAULT_ALL (WMI_VHT_CAP_MAX_MPDU_LEN_11454  |      \
                                 WMI_VHT_CAP_SGI_80MHZ           |      \
                                 WMI_VHT_CAP_TX_STBC             |      \
                                 WMI_VHT_CAP_RX_STBC_MASK        |      \
                                 WMI_VHT_CAP_RX_LDPC             |      \
                                 WMI_VHT_CAP_TX_LDPC             |      \
                                 WMI_VHT_CAP_MAX_AMPDU_LEN_EXP   |      \
                                 WMI_VHT_CAP_RX_FIXED_ANT        |      \
                                 WMI_VHT_CAP_TX_FIXED_ANT)

/* Interested readers refer to Rx/Tx MCS Map definition as defined in
 802.11ac
 */
#define WMI_VHT_MAX_MCS_EXT_SS_GET(vht_mcs_map, index) WMI_GET_BITS(vht_mcs_map, 16 + index, 1)
#define WMI_VHT_MAX_MCS_EXT_SS_SET(vht_mcs_map, index, value) WMI_SET_BITS(vht_mcs_map, 16 + index, 1, value)

/* Notification bit for Ext MCS 10/11 support */
#define WMI_VHT_MCS_NOTIFY_EXT_SS_GET(vht_mcs_map) WMI_GET_BITS(vht_mcs_map, 24, 1)
#define WMI_VHT_MCS_NOTIFY_EXT_SS_SET(vht_mcs_map, value) WMI_SET_BITS(vht_mcs_map, 24, 1, value)

#define WMI_VHT_MAX_MCS_4_SS_MASK(r,ss)      ((3 & (r)) << (((ss) - 1) << 1))
#define WMI_VHT_MAX_SUPP_RATE_MASK           0x1fff0000
#define WMI_VHT_MAX_SUPP_RATE_MASK_SHIFT     16

/** 11ax capabilities */
#define WMI_HE_CAP_PPE_PRESENT            0x00000001
#define WMI_HE_CAP_TWT_RESPONDER_SUPPORT  0x00000002
#define WMI_HE_CAP_TWT_REQUESTER_SUPPORT  0x00000004
#define WMI_HE_FRAG_SUPPORT_MASK          0x00000018
#define WMI_HE_FRAG_SUPPORT_SHIFT         3

#define WMI_HE_CAP_1X_LTF_400NS_GI_SUPPORT      0x00000001
#define WMI_HE_CAP_2X_LTF_400NS_GI_SUPPORT      0x00000002
#define WMI_HE_CAP_2X_LTF_160_80_80_SUPPORT     0x00000004
#define WMI_HE_CAP_RX_DL_OFDMA_SUPPORT          0x00000018
#define WMI_HE_CAP_RX_DL_MUMIMO_SUPPORT         0x00000030

#define WMI_HE_CAP_1X_LTF_400NS_GI_SUPPORT_GET(he_cap_info_dword1) \
    WMI_GET_BITS(he_cap_info_dword1, 0, 1)
#define WMI_HE_CAP_1X_LTF_400NS_GI_SUPPORT_SET(he_cap_info_dword1, value) \
    WMI_SET_BITS(he_cap_info_dword1, 0, 1, value)

#define WMI_HE_CAP_2X_LTF_400NS_GI_SUPPORT_GET(he_cap_info_dword1) \
    WMI_GET_BITS(he_cap_info_dword1, 1, 1)
#define WMI_HE_CAP_2X_LTF_400NS_GI_SUPPORT_SET(he_cap_info_dword1, value) \
    WMI_SET_BITS(he_cap_info_dword1, 1, 1, value)

#define WMI_HE_CAP_2X_LTF_160_80_80_SUPPORT_GET(he_cap_info_dword1) \
    WMI_GET_BITS(he_cap_info_dword1, 2, 1)
#define WMI_HE_CAP_2X_LTF_160_80_80_SUPPORT_SET(he_cap_info_dword1, value) \
    WMI_SET_BITS(he_cap_info_dword1, 2, 1, value)

#define WMI_HE_CAP_RX_DL_OFDMA_SUPPORT_GET(he_cap_info_dword1) \
    WMI_GET_BITS(he_cap_info_dword1, 3, 2)
#define WMI_HE_CAP_RX_DL_OFDMA_SUPPORT_SET(he_cap_info_dword1, value) \
    WMI_SET_BITS(he_cap_info_dword1, 3, 2, value)

#define WMI_HE_CAP_RX_DL_MUMIMO_SUPPORT_GET(he_cap_info_dword1) \
    WMI_GET_BITS(he_cap_info_dword1, 5, 2)
#define WMI_HE_CAP_RX_DL_MUMIMO_SUPPORT_SET(he_cap_info_dword1, value) \
    WMI_SET_BITS(he_cap_info_dword1, 5, 2, value)

/* Interested readers refer to Rx/Tx MCS Map definition as defined in 802.11ax
 */
#define WMI_HE_MAX_MCS_4_SS_MASK(r,ss)      ((3 & (r)) << (((ss) - 1) << 1))

/*
 * index ranges from 0 to 15, and is used for checking if MCS 12/13 is enabled
 * for a particular NSS.
 * The lower 8 bits (indices 0-7) within the 16 bits indicate MCS 12/13
 * enablement for BW <= 80MHz; the upper 8 bits (indices 8-15) within
 * the 16 bits indicate MCS 12/13 enablement for BW > 80MHz.
 * The 16 bits for the index values are within the upper bits (bits 31:16)
 * of a 32-bit word.
 */
#define WMI_HE_EXTRA_MCS_SS_GET(he_mcs_map_ext, index) \
    WMI_GET_BITS(he_mcs_map_ext, 16 + index, 1)
#define WMI_HE_EXTRA_MCS_SS_SET(he_mcs_map_ext, index, value) \
    WMI_SET_BITS(he_mcs_map_ext, 16 + index, 1, value)

/* fragmentation support field value */
enum {
    WMI_HE_FRAG_SUPPORT_LEVEL0, /* No Fragmentation support */
    WMI_HE_FRAG_SUPPORT_LEVEL1, /* support for fragments within a VHT single MPDU, no support for fragments within AMPDU */
    WMI_HE_FRAG_SUPPORT_LEVEL2, /* support for up to 1 fragment per MSDU within a single A-MPDU */
    WMI_HE_FRAG_SUPPORT_LEVEL3, /* support for multiple fragments per MSDU within an A-MPDU */
};

enum {
    WMI_HE_RX_DL_OFDMA_SUPPORT_DEFAULT, /* Default */
    WMI_HE_RX_DL_OFDMA_SUPPORT_DISABLE, /* RX DL OFDMA Support Disabled */
    WMI_HE_RX_DL_OFDMA_SUPPORT_ENABLE,  /* RX DL OFDMA Support Enabled */
    WMI_HE_RX_DL_OFDMA_SUPPORT_INVALID, /* INVALID  */
};

enum {
    WMI_HE_RX_DL_MUMIMO_SUPPORT_DEFAULT, /* Default */
    WMI_HE_RX_DL_MUMIMO_SUPPORT_DISABLE, /* RX DL MU-MIMO Support Disabled */
    WMI_HE_RX_DL_MUMIMO_SUPPORT_ENABLE,  /* RX DL MU-MIMO Support Enabled */
    WMI_HE_RX_DL_MUMIMO_SUPPORT_INVALID, /* INVALID  */
};

/** NOTE: This defs cannot be changed in the future without breaking WMI compatibility */
#define WMI_MAX_NUM_SS                    MAX_HE_NSS
#define WMI_MAX_NUM_RU                    MAX_HE_RU

/*
 * Figure 8 554ae: -PPE Threshold Info field format
 * we pack PPET16 and PPT8 for four RU's in one element of array.
 *
 * ppet16_ppet8_ru3_ru0 array element 0 holds:
 *     |  PPET8 | PPET16 | PPET8  | PPET16 | PPET8  | PPET16 | PPET8  | PPET16 |
 *rsvd |NSS1,RU4|NSS1,RU4|NSS1,RU3|NSS1,RU3|NSS1,RU2|NSS1,RU2|NSS1,RU1|NSS1,RU1|
 *31:23|  22:20 |  19:17 |  17:15 |  14:12 |  11:9  |   8:6  |   5:3  |   2:0  |
 *
 * ppet16_ppet8_ru3_ru0 array element 1 holds:
 *     | PPET8  | PPET16 | PPET8  | PPET16 | PPET8  | PPET16 | PPET8  | PPET16 |
 *rsvd |NSS2,RU4|NSS2,RU4|NSS2,RU3|NSS2,RU3|NSS2,RU2|NSS2,RU2|NSS2,RU1|NSS2,RU1|
 *31:23|  22:20 |  19:17 |  17:15 |  14:12 |  11:9  |   8:6  |   5:3  |   2:0  |
 *
 * etc.
 */

/*
 * Note that in these macros, "ru" is one-based, not zero-based, while
 * nssm1 is zero-based.
 */
#define WMI_SET_PPET16(ppet16_ppet8_ru3_ru0, ru, nssm1, ppet) \
    do { \
        ppet16_ppet8_ru3_ru0[nssm1] &= ~(7 << (((ru-1) & 3) * 6)); \
        ppet16_ppet8_ru3_ru0[nssm1] |= ((ppet & 7) << (((ru-1) & 3) * 6)); \
    } while (0)

#define WMI_GET_PPET16(ppet16_ppet8_ru3_ru0, ru, nssm1) \
    ((ppet16_ppet8_ru3_ru0[nssm1] >> (((ru-1) & 3) * 6)) & 7)

#define WMI_SET_PPET8(ppet16_ppet8_ru3_ru0, ru, nssm1, ppet) \
    do { \
        ppet16_ppet8_ru3_ru0[nssm1] &= ~(7 << (((ru-1) & 3) * 6 + 3)); \
        ppet16_ppet8_ru3_ru0[nssm1] |= ((ppet&7) << (((ru-1) & 3) * 6 + 3)); \
    } while (0)

#define WMI_GET_PPET8(ppet16_ppet8_ru3_ru0, ru, nssm1) \
    ((ppet16_ppet8_ru3_ru0[nssm1] >> (((ru-1) & 3) * 6 + 3)) & 7)

typedef struct _wmi_ppe_threshold {
    A_UINT32 numss_m1; /** NSS - 1*/
    union {
        A_UINT32 ru_count; /** RU COUNT OBSOLETE to be removed after few versions */
        A_UINT32 ru_mask; /** RU index mask */
    };
    A_UINT32 ppet16_ppet8_ru3_ru0[WMI_MAX_NUM_SS]; /** ppet8 and ppet16 for max num ss */
    /**************************************************
     * As this struct is embedded inside other structs,
     * it cannot be expanded without breaking backwards
     * compatibility.  Do not add new fields here.
     **************************************************/
} wmi_ppe_threshold;

#define WMI_MAX_EHTCAP_MAC_SIZE  2
#define WMI_MAX_EHTCAP_PHY_SIZE  3

/*
 * 0 – index indicated EHT-MCS map for 20Mhz only sta (4 bytes valid)
 * 1 – index for <= 80MHz bw  (only 3 bytes are valid and other is reserved)
 * 2 – index for == 160Mhz bw (only 3 bytes are valid and other is reserved)
 * 3 – index for == 320Mhz bw (only 3 bytes are valid and other is reserved)
 */
enum {
    WMI_EHT_SUPP_MCS_20MHZ_ONLY,
    WMI_EHT_SUPP_MCS_LE_80MHZ,
    WMI_EHT_SUPP_MCS_160MHZ,
    WMI_EHT_SUPP_MCS_320MHZ,
};
#define WMI_MAX_EHT_SUPP_MCS_2G_SIZE  2
#define WMI_MAX_EHT_SUPP_MCS_5G_SIZE  4

/* WMI_SYS_CAPS_* refer to the capabilities that system support
 */
#define WMI_SYS_CAP_ENABLE                       0x00000001
#define WMI_SYS_CAP_TXPOWER                      0x00000002

/*
 * WMI Dual Band Simultaneous (DBS) hardware mode list bit-mask definitions.
 * Bits 5:0 are reserved
 */
#define WMI_DBS_HW_MODE_MAC0_TX_STREAMS_BITPOS  (28)
#define WMI_DBS_HW_MODE_MAC0_RX_STREAMS_BITPOS  (24)
#define WMI_DBS_HW_MODE_MAC1_TX_STREAMS_BITPOS  (20)
#define WMI_DBS_HW_MODE_MAC1_RX_STREAMS_BITPOS  (16)
#define WMI_DBS_HW_MODE_MAC0_BANDWIDTH_BITPOS   (12)
#define WMI_DBS_HW_MODE_MAC1_BANDWIDTH_BITPOS   (8)
#define WMI_DBS_HW_MODE_DBS_MODE_BITPOS         (7)
#define WMI_DBS_HW_MODE_AGILE_DFS_MODE_BITPOS   (6)

#define WMI_DBS_HW_MODE_MAC0_TX_STREAMS_MASK    (0xf << WMI_DBS_HW_MODE_MAC0_TX_STREAMS_BITPOS)
#define WMI_DBS_HW_MODE_MAC0_RX_STREAMS_MASK    (0xf << WMI_DBS_HW_MODE_MAC0_RX_STREAMS_BITPOS)
#define WMI_DBS_HW_MODE_MAC1_TX_STREAMS_MASK    (0xf << WMI_DBS_HW_MODE_MAC1_TX_STREAMS_BITPOS)
#define WMI_DBS_HW_MODE_MAC1_RX_STREAMS_MASK    (0xf << WMI_DBS_HW_MODE_MAC1_RX_STREAMS_BITPOS)
#define WMI_DBS_HW_MODE_MAC0_BANDWIDTH_MASK     (0xf << WMI_DBS_HW_MODE_MAC0_BANDWIDTH_BITPOS)
#define WMI_DBS_HW_MODE_MAC1_BANDWIDTH_MASK     (0xf << WMI_DBS_HW_MODE_MAC1_BANDWIDTH_BITPOS)
#define WMI_DBS_HW_MODE_DBS_MODE_MASK           (0x1 << WMI_DBS_HW_MODE_DBS_MODE_BITPOS)
#define WMI_DBS_HW_MODE_AGILE_DFS_MODE_MASK     (0x1 << WMI_DBS_HW_MODE_AGILE_DFS_MODE_BITPOS)

#define WMI_DBS_HW_MODE_MAC0_TX_STREAMS_SET(hw_mode, value) \
    WMI_SET_BITS(hw_mode, WMI_DBS_HW_MODE_MAC0_TX_STREAMS_BITPOS, 4, value)
#define WMI_DBS_HW_MODE_MAC0_RX_STREAMS_SET(hw_mode, value) \
    WMI_SET_BITS(hw_mode, WMI_DBS_HW_MODE_MAC0_RX_STREAMS_BITPOS, 4, value)
#define WMI_DBS_HW_MODE_MAC1_TX_STREAMS_SET(hw_mode, value) \
    WMI_SET_BITS(hw_mode, WMI_DBS_HW_MODE_MAC1_TX_STREAMS_BITPOS, 4, value)
#define WMI_DBS_HW_MODE_MAC1_RX_STREAMS_SET(hw_mode, value) \
    WMI_SET_BITS(hw_mode, WMI_DBS_HW_MODE_MAC1_RX_STREAMS_BITPOS, 4, value)
#define WMI_DBS_HW_MODE_MAC0_BANDWIDTH_SET(hw_mode, value)  \
    WMI_SET_BITS(hw_mode, WMI_DBS_HW_MODE_MAC0_BANDWIDTH_BITPOS, 4, value)
#define WMI_DBS_HW_MODE_MAC1_BANDWIDTH_SET(hw_mode, value)  \
    WMI_SET_BITS(hw_mode, WMI_DBS_HW_MODE_MAC1_BANDWIDTH_BITPOS, 4, value)
#define WMI_DBS_HW_MODE_DBS_MODE_SET(hw_mode, value)        \
    WMI_SET_BITS(hw_mode, WMI_DBS_HW_MODE_DBS_MODE_BITPOS, 1, value)
#define WMI_DBS_HW_MODE_AGILE_DFS_SET(hw_mode, value)       \
    WMI_SET_BITS(hw_mode, WMI_DBS_HW_MODE_AGILE_DFS_MODE_BITPOS, 1, value)

#define WMI_DBS_HW_MODE_MAC0_TX_STREAMS_GET(hw_mode)    \
    ((hw_mode & WMI_DBS_HW_MODE_MAC0_TX_STREAMS_MASK) >> WMI_DBS_HW_MODE_MAC0_TX_STREAMS_BITPOS)
#define WMI_DBS_HW_MODE_MAC0_RX_STREAMS_GET(hw_mode)    \
    ((hw_mode & WMI_DBS_HW_MODE_MAC0_RX_STREAMS_MASK) >> WMI_DBS_HW_MODE_MAC0_RX_STREAMS_BITPOS)
#define WMI_DBS_HW_MODE_MAC1_TX_STREAMS_GET(hw_mode)    \
    ((hw_mode & WMI_DBS_HW_MODE_MAC1_TX_STREAMS_MASK) >> WMI_DBS_HW_MODE_MAC1_TX_STREAMS_BITPOS)
#define WMI_DBS_HW_MODE_MAC1_RX_STREAMS_GET(hw_mode)    \
    ((hw_mode & WMI_DBS_HW_MODE_MAC1_RX_STREAMS_MASK) >> WMI_DBS_HW_MODE_MAC1_RX_STREAMS_BITPOS)
#define WMI_DBS_HW_MODE_MAC0_BANDWIDTH_GET(hw_mode)     \
    ((hw_mode & WMI_DBS_HW_MODE_MAC0_BANDWIDTH_MASK) >> WMI_DBS_HW_MODE_MAC0_BANDWIDTH_BITPOS)
#define WMI_DBS_HW_MODE_MAC1_BANDWIDTH_GET(hw_mode)     \
    ((hw_mode & WMI_DBS_HW_MODE_MAC1_BANDWIDTH_MASK) >> WMI_DBS_HW_MODE_MAC1_BANDWIDTH_BITPOS)
#define WMI_DBS_HW_MODE_DBS_MODE_GET(hw_mode)           \
    ((hw_mode & WMI_DBS_HW_MODE_DBS_MODE_MASK) >> WMI_DBS_HW_MODE_DBS_MODE_BITPOS)
#define WMI_DBS_HW_MODE_AGILE_DFS_GET(hw_mode)          \
    ((hw_mode & WMI_DBS_HW_MODE_AGILE_DFS_MODE_MASK) >> WMI_DBS_HW_MODE_AGILE_DFS_MODE_BITPOS)

#define WMI_DBS_CONC_SCAN_CFG_DBS_SCAN_BITPOS        (31)
#define WMI_DBS_CONC_SCAN_CFG_AGILE_SCAN_BITPOS      (30)
#define WMI_DBS_CONC_SCAN_CFG_AGILE_DFS_SCAN_BITPOS  (29)
#define WMI_DBS_CONC_SCAN_CFG_ASYNC_DBS_SCAN_BITPOS  (28)
#define WMI_DBS_CONC_SCAN_CFG_SYNC_DBS_SCAN_BITPOS   (27)

#define WMI_DBS_CONC_SCAN_CFG_DBS_SCAN_MASK         ((A_UINT32) 0x1 << WMI_DBS_CONC_SCAN_CFG_DBS_SCAN_BITPOS)
#define WMI_DBS_CONC_SCAN_CFG_AGILE_SCAN_MASK       ((A_UINT32) 0x1 << WMI_DBS_CONC_SCAN_CFG_AGILE_SCAN_BITPOS)
#define WMI_DBS_CONC_SCAN_CFG_AGILE_DFS_SCAN_MASK   ((A_UINT32) 0x1 << WMI_DBS_CONC_SCAN_CFG_AGILE_DFS_SCAN_BITPOS)
#define WMI_DBS_CONC_SCAN_CFG_ASYC_DBS_SCAN_MASK    ((A_UINT32) 0x1 << WMI_DBS_CONC_SCAN_CFG_ASYNC_DBS_SCAN_BITPOS)
#define WMI_DBS_CONC_SCAN_CFG_SYNC_DBS_SCAN_MASK    ((A_UINT32) 0x1 << WMI_DBS_CONC_SCAN_CFG_SYNC_DBS_SCAN_BITPOS)

#define WMI_DBS_CONC_SCAN_CFG_DBS_SCAN_SET(scan_cfg, value) \
    WMI_SET_BITS(scan_cfg, WMI_DBS_CONC_SCAN_CFG_DBS_SCAN_BITPOS, 1, value)
#define WMI_DBS_CONC_SCAN_CFG_AGILE_SCAN_SET(scan_cfg, value) \
    WMI_SET_BITS(scan_cfg, WMI_DBS_CONC_SCAN_CFG_AGILE_SCAN_BITPOS, 1, value)
#define WMI_DBS_CONC_SCAN_CFG_AGILE_DFS_SCAN_SET(scan_cfg, value) \
    WMI_SET_BITS(scan_cfg, WMI_DBS_CONC_SCAN_CFG_AGILE_DFS_SCAN_BITPOS, 1, value)
#define WMI_DBS_CONC_SCAN_CFG_ASYNC_DBS_SCAN_SET(scan_cfg, value) \
    WMI_SET_BITS(scan_cfg, WMI_DBS_CONC_SCAN_CFG_ASYNC_DBS_SCAN_BITPOS, 1, value)
#define WMI_DBS_CONC_SCAN_CFG_SYNC_DBS_SCAN_SET(scan_cfg, value) \
    WMI_SET_BITS(scan_cfg, WMI_DBS_CONC_SCAN_CFG_SYNC_DBS_SCAN_BITPOS, 1, value)

#define WMI_DBS_CONC_SCAN_CFG_DBS_SCAN_GET(scan_cfg)    \
    ((scan_cfg & WMI_DBS_CONC_SCAN_CFG_DBS_SCAN_MASK) >> WMI_DBS_CONC_SCAN_CFG_DBS_SCAN_BITPOS)
#define WMI_DBS_CONC_SCAN_CFG_AGILE_SCAN_GET(scan_cfg)    \
    ((scan_cfg & WMI_DBS_CONC_SCAN_CFG_AGILE_SCAN_MASK) >> WMI_DBS_CONC_SCAN_CFG_AGILE_SCAN_BITPOS)
#define WMI_DBS_CONC_SCAN_CFG_AGILE_DFS_SCAN_GET(scan_cfg)    \
    ((scan_cfg & WMI_DBS_CONC_SCAN_CFG_AGILE_DFS_SCAN_MASK) >> WMI_DBS_CONC_SCAN_CFG_AGILE_DFS_SCAN_BITPOS)
#define WMI_DBS_CONC_SCAN_CFG_ASYNC_DBS_SCAN_GET(scan_cfg)    \
    ((scan_cfg & WMI_DBS_CONC_SCAN_CFG_ASYC_DBS_SCAN_MASK) >> WMI_DBS_CONC_SCAN_CFG_ASYNC_DBS_SCAN_BITPOS)
#define WMI_DBS_CONC_SCAN_CFG_SYNC_DBS_SCAN_GET(scan_cfg)    \
    ((scan_cfg & WMI_DBS_CONC_SCAN_CFG_SYNC_DBS_SCAN_MASK) >> WMI_DBS_CONC_SCAN_CFG_SYNC_DBS_SCAN_BITPOS)

#define WMI_DBS_FW_MODE_CFG_DBS_BITPOS                  (31)
#define WMI_DBS_FW_MODE_CFG_AGILE_DFS_BITPOS            (30)
#define WMI_DBS_FW_MODE_CFG_DBS_FOR_CXN_BITPOS          (29)
#define WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_STA_BITPOS (28)
#define WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_P2P_BITPOS (27)
#define WMI_DBS_FW_MODE_CFG_ASYNC_SBS_BITPOS            (26)

#define WMI_DBS_FW_MODE_CFG_DBS_MASK                    ((A_UINT32) 0x1 << WMI_DBS_FW_MODE_CFG_DBS_BITPOS)
#define WMI_DBS_FW_MODE_CFG_AGILE_DFS_MASK              ((A_UINT32) 0x1 << WMI_DBS_FW_MODE_CFG_AGILE_DFS_BITPOS)
#define WMI_DBS_FW_MODE_CFG_DBS_FOR_CXN_DFS_MASK        ((A_UINT32) 0x1 << WMI_DBS_FW_MODE_CFG_DBS_FOR_CXN_BITPOS)
#define WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_STA_MASK   ((A_UINT32) 0x1 << WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_STA_BITPOS)
#define WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_P2P_MASK   ((A_UINT32) 0x1 << WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_P2P_BITPOS)
#define WMI_DBS_FW_MODE_CFG_ASYNC_SBS_MASK              ((A_UINT32) 0x1 << WMI_DBS_FW_MODE_CFG_ASYNC_SBS_BITPOS)

#define WMI_DBS_FW_MODE_CFG_DBS_SET(fw_mode, value) \
    WMI_SET_BITS(fw_mode, WMI_DBS_FW_MODE_CFG_DBS_BITPOS, 1, value)
#define WMI_DBS_FW_MODE_CFG_AGILE_DFS_SET(fw_mode, value) \
    WMI_SET_BITS(fw_mode, WMI_DBS_FW_MODE_CFG_AGILE_DFS_BITPOS, 1, value)
#define WMI_DBS_FW_MODE_CFG_DBS_FOR_CXN_SET(fw_mode, value) \
    WMI_SET_BITS(fw_mode, WMI_DBS_FW_MODE_CFG_DBS_FOR_CXN_BITPOS, 1, value)
#define WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_STA_SET(fw_mode, value) \
    WMI_SET_BITS(fw_mode, WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_STA_BITPOS, 1, value)
#define WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_P2P_SET(fw_mode, value) \
    WMI_SET_BITS(fw_mode, WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_P2P_BITPOS, 1, value)
#define WMI_DBS_FW_MODE_CFG_ASYNC_SBS_SET(fw_mode, value) \
    WMI_SET_BITS(fw_mode, WMI_DBS_FW_MODE_CFG_ASYNC_SBS_BITPOS, 1, value)

#define WMI_DBS_FW_MODE_CFG_DBS_GET(fw_mode)    \
    ((fw_mode & WMI_DBS_FW_MODE_CFG_DBS_MASK) >> WMI_DBS_FW_MODE_CFG_DBS_BITPOS)
#define WMI_DBS_FW_MODE_CFG_AGILE_DFS_GET(fw_mode)    \
    ((fw_mode & WMI_DBS_FW_MODE_CFG_AGILE_DFS_MASK) >> WMI_DBS_FW_MODE_CFG_AGILE_DFS_BITPOS)
#define WMI_DBS_FW_MODE_CFG_DBS_FOR_CXN_GET(fw_mode)    \
    ((fw_mode & WMI_DBS_FW_MODE_CFG_DBS_FOR_CXN_DFS_MASK) >> WMI_DBS_FW_MODE_CFG_DBS_FOR_CXN_BITPOS)
#define WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_STA_GET(fw_mode)    \
    ((fw_mode & WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_STA_MASK) >> WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_STA_BITPOS)
#define WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_P2P_GET(fw_mode)    \
    ((fw_mode & WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_P2P_MASK) >> WMI_DBS_FW_MODE_CFG_DBS_FOR_STA_PLUS_P2P_BITPOS)
#define WMI_DBS_FW_MODE_CFG_ASYNC_SBS_GET(fw_mode)    \
    ((fw_mode & WMI_DBS_FW_MODE_CFG_ASYNC_SBS_MASK) >> WMI_DBS_FW_MODE_CFG_ASYNC_SBS_BITPOS)

/** NOTE: This structure cannot be extended in the future without breaking WMI compatibility */
typedef struct _wmi_abi_version {
    A_UINT32 abi_version_0; /** WMI Major and Minor versions */
    A_UINT32 abi_version_1; /** WMI change revision */
    A_UINT32 abi_version_ns_0; /** ABI version namespace first four dwords */
    A_UINT32 abi_version_ns_1; /** ABI version namespace second four dwords */
    A_UINT32 abi_version_ns_2; /** ABI version namespace third four dwords */
    A_UINT32 abi_version_ns_3; /** ABI version namespace fourth four dwords */
} wmi_abi_version;

/*
 * maximum number of memory requests allowed from FW.
 */
#define WMI_MAX_MEM_REQS 16

/* !!NOTE!!:
 * This HW_BD_INFO_SIZE cannot be changed without breaking compatibility.
 * Please don't change it.
 */
#define HW_BD_INFO_SIZE       5

/**
 * PDEV ID to identify the physical device,
 * value 0 reserved for SOC level commands/event
 */
#define WMI_PDEV_ID_SOC         0 /* SOC level, applicable to all PDEVs */
#define WMI_PDEV_ID_1ST         1 /* first pdev  (pdev 0) */
#define WMI_PDEV_ID_2ND         2 /* second pdev (pdev 1) */
#define WMI_PDEV_ID_3RD         3 /* third pdev  (pdev 2) */

/*
 * Enum regarding which BDF elements are provided in which elements of the
 * wmi_service_ready_event_fixed_param.hw_bd_info[] array
 */
typedef enum {
    BDF_VERSION = 0,
    REF_DESIGN_ID = 1,
    CUSTOMER_ID = 2,
    PROJECT_ID = 3,
    BOARD_DATA_REV = 4,
} wmi_hw_bd_info_e;

/*
 * Macros to get/set BDF details within the
 * wmi_service_ready_event_fixed_param.hw_bd_info[] array
 */
#define WMI_GET_BDF_VERSION(hw_bd_info)         ((hw_bd_info)[BDF_VERSION])
#define WMI_GET_REF_DESIGN(hw_bd_info)          ((hw_bd_info)[REF_DESIGN_ID])
#define WMI_GET_CUSTOMER_ID(hw_bd_info)         ((hw_bd_info)[CUSTOMER_ID])
#define WMI_GET_PROJECT_ID(hw_bd_info)          ((hw_bd_info)[PROJECT_ID])
#define WMI_GET_BOARD_DATA_REV(hw_bd_info)      ((hw_bd_info)[BOARD_DATA_REV])

#define WMI_SET_BDF_VERSION(hw_bd_info, val)    ((hw_bd_info)[BDF_VERSION]    = (val))
#define WMI_SET_REF_DESIGN(hw_bd_info, val)     ((hw_bd_info)[REF_DESIGN_ID]  = (val))
#define WMI_SET_CUSTOMER_ID(hw_bd_info, val)    ((hw_bd_info)[CUSTOMER_ID]    = (val))
#define WMI_SET_PROJECT_ID(hw_bd_info, val)     ((hw_bd_info)[PROJECT_ID]     = (val))
#define WMI_SET_BOARD_DATA_REV(hw_bd_info, val) ((hw_bd_info)[BOARD_DATA_REV] = (val))

/*
 * Enum to indicate which Tx power capability is provided in which element of
 * hw_tx_power_signed
 */
typedef enum {
    WMI_HW_MIN_TX_POWER_SIGNED = 0,
    WMI_HW_MAX_TX_POWER_SIGNED = 1,
    WMI_HW_TX_POWER_CAPS_MAX,
} wmi_hw_tx_power_caps;

/**
 * The following struct holds optional payload for
 * wmi_service_ready_event_fixed_param,e.g., 11ac pass some of the
 * device capability to the host.
 */
typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_WMI_SERVICE_READY_EVENT */
    A_UINT32 fw_build_vers; /* firmware build number */
    wmi_abi_version fw_abi_vers;
    A_UINT32 phy_capability; /* WMI_PHY_CAPABILITY */
    A_UINT32 max_frag_entry; /* Maximum number of frag table entries that SW will populate less 1 */
    A_UINT32 num_rf_chains;
    /* The following field is only valid for service type WMI_SERVICE_11AC */
    A_UINT32 ht_cap_info; /* WMI HT Capability */
    A_UINT32 vht_cap_info; /* VHT capability info field of 802.11ac */
    A_UINT32 vht_supp_mcs; /* VHT Supported MCS Set field Rx/Tx same */
    A_UINT32 hw_min_tx_power;
    A_UINT32 hw_max_tx_power;
    /* sys_cap_info:
     * bits  1:0  - RXTX LED + RFKILL enable flags (see WMI_LEDRFKILL_FLAGS)
     * bits 31:2  - reserved (must be set to zero)
     */
    A_UINT32 sys_cap_info;
    A_UINT32 min_pkt_size_enable; /* Enterprise mode short pkt enable */
    /** Max beacon and Probe Response IE offload size (includes
     *  optional P2P IEs) */
    A_UINT32 max_bcn_ie_size;
    /*
     * request to host to allocate a chuck of memory and pss it down to FW via WM_INIT.
     * FW uses this as FW extesnsion memory for saving its data structures. Only valid
     * for low latency interfaces like PCIE where FW can access this memory directly (or)
     * by DMA.
     */
    A_UINT32 num_mem_reqs;
    /* Max No. scan channels target can support
     * If FW is too old and doesn't indicate this number, host side value will default to
     * 0, and host will take the original compatible value (62) for future scan channel
     * setup.
     */
    A_UINT32 max_num_scan_channels;

    /* Hardware board specific ID. Values defined in enum WMI_HWBOARD_ID.
     * Default 0 means that hw_bd_info[] is invalid (legacy board).
     */
    A_UINT32 hw_bd_id;
    A_UINT32 hw_bd_info[HW_BD_INFO_SIZE]; /* Board specific information. Invalid if hw_hd_id is zero. */

    /*
     * Number of MACs supported, i.e. a DBS-capable device will return 2
     */
    A_UINT32 max_supported_macs;

    /*
     * FW sub-feature capabilities to be used in concurrence with wmi_service_bitmap
     */
    A_UINT32 wmi_fw_sub_feat_caps; /* values from enum WMI_FW_SUB_FEAT_CAPS */

    /*
     * Number of Dual Band Simultaneous (DBS) hardware modes
     */
    A_UINT32 num_dbs_hw_modes;

    /*
     * txrx_chainmask
     *    [7:0]   - 2G band tx chain mask
     *    [15:8]  - 2G band rx chain mask
     *    [23:16] - 5G band tx chain mask
     *    [31:24] - 5G band rx chain mask
     *
     */
    A_UINT32 txrx_chainmask;

    /*
     * default Dual Band Simultaneous (DBS) hardware mode
     */
    A_UINT32 default_dbs_hw_mode_index;

    /*
     * Number of msdu descriptors target would use
     */
    A_UINT32 num_msdu_desc;

/* This ready_event_fixed_param TLV is followed by the below TLVs:
 *     HAL_REG_CAPABILITIES   hal_reg_capabilities;
 *     A_UINT32 wmi_service_bitmap[WMI_SERVICE_BM_SIZE];
 *     wlan_host_mem_req mem_reqs[];
 *     A_UINT32 wlan_dbs_hw_mode_list[];
 */
} wmi_service_ready_event_fixed_param;

typedef enum {
    WMI_RXTX_LED_ENABLE         = 0x00000001,
    WMI_RFKILL_ENABLE           = 0x00000002,
} WMI_LEDRFKILL_FLAGS;

#define WMI_SERVICE_SEGMENT_BM_SIZE32 4 /* 4x A_UINT32 = 128 bits */
typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_service_available_event_fixed_param */
    /*
     * The wmi_service_segment offset field specifies the position within the
     * logical bitmap of WMI service flags at which the WMI service flags
     * specified within this message begin.
     * Since the first 128 WMI service flags are specified within the
     * wmi_service_bitmap field of the WMI_SERVICE_READY_EVENT message,
     * the wmi_service_segment_offset value is expected to be 128 or more.
     */
    A_UINT32 wmi_service_segment_offset;
    A_UINT32 wmi_service_segment_bitmap[WMI_SERVICE_SEGMENT_BM_SIZE32];
/*
 * This TLV is followed by the below TLVs:
 * A_UINT32 wmi_service_ext_bitmap[]
 *     The wmi_service_ext_bitmap covers WMI service flags at the offset where
 *     wmi_service_available_event_fixed_param.wmi_service_segment_bitmap
 *     leaves off.
 *     For example, if
 *         wmi_service_available_event_fixed_param.wmi_service_segment_offset
 *     is 128, then
 *         wmi_service_available_event_fixed_param.wmi_service_segment_bitmap
 *     will cover WMI service flags
 *         128 to (128 + WMI_SERVICE_SEGMENT_BM_SIZE32 * 32) = 128 to 256
 *     and wmi_service_ext_bitmap will cover WMI service flags starting at 256.
 */
} wmi_service_available_event_fixed_param;

/*
 * HDL version GET/SET APIs
 */
#define WMI_HDL_VERSION_BITPOS    0
#define WMI_HDL_VERSION_NUM_BITS 10

#define WMI_HDL_VERSION_GET(dword) WMI_GET_BITS(dword, WMI_HDL_VERSION_BITPOS, WMI_HDL_VERSION_NUM_BITS)
#define WMI_HDL_VERSION_SET(dword, value) WMI_SET_BITS(dword, WMI_HDL_VERSION_BITPOS, WMI_HDL_VERSION_NUM_BITS, value)

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_WMI_SERVICE_EXT_READY_EVENT */
    /* which WMI_DBS_CONC_SCAN_CFG setting the FW is initialized with */
    A_UINT32 default_conc_scan_config_bits;
    /* which WMI_DBS_FW_MODE_CFG setting the FW is initialized with */
    A_UINT32 default_fw_config_bits;
    wmi_ppe_threshold ppet;
    A_UINT32 he_cap_info; /* see section 8.4.2.213 from draft r8 of 802.11ax; see WMI_HE_FRAG_SUPPORT enum */
    /*
     * An HT STA shall not allow transmission of more than one MPDU start
     * within the time limit described in the MPDU maximum density field.
     */
    A_UINT32 mpdu_density; /* units are microseconds */
    /*
     * Maximum no of BSSID based RX filters host can program
     * Value 0 means FW hasn't given any limit to host.
     */
    A_UINT32 max_bssid_rx_filters;
    /*
     * Extended FW build version information:
     * bits  9:0  -> HDL version info
     * bits 12:10 -> CRM sub ID MSbs (refer to WMI_SVC_RDY_CRM_SUB_ID_GET/SET)
     * bits 27:13 -> reserved
     * bits 31:28 -> CRM sub ID LSbs (refer to WMI_SVC_RDY_CRM_SUB_ID_GET/SET)
     */
    A_UINT32 fw_build_vers_ext;
    /* max_nlo_ssids - dynamically negotiated maximum number of SSIDS for NLO
     * This limit is the maximum number of SSIDs that can be configured in the
     * target for Network List Offload (i.e. scanning for a preferred network).
     * If this value is 0x0, the target supports WMI_NLO_MAX_SSIDS (16).
     * If this value is non-zero, the host should send back in the
     * WMI_INIT message's wmi_resource_config.max_nlo_ssids a value that
     * is equal to or less than the target capability limit reported here.
     */
    A_UINT32 max_nlo_ssids;
    /* ref to section 8.4.2.48 Multiple BSSID element
     * The Max BSSID Indicator field contains a value assigned to n,
     * where 2^n is the maximum number of BSSIDs
     */
    A_UINT32 max_bssid_indicator;

    /* 2nd DWORD of HE MAC Capabilities */
    A_UINT32 he_cap_info_ext;

    /**************************************************************************
     * DON'T ADD ANY FURTHER FIELDS HERE -
     * It would cause the size of the READY_EXT message within some targets
     * to exceed the size of the buffer used for the message.
     **************************************************************************/

    /*
     * A variable-length TLV array of wmi_chan_rf_characterization_info will
     * follow this fixed_param TLV, containing rx characterization info for
     * primary channels.
     *   WMI_CHAN_RF_CHARACTERIZATION_INFO wmi_chan_rf_characterization_info[];
     */
} wmi_service_ready_ext_event_fixed_param;

#define WMI_SVC_RDY_CRM_SUB_ID_LSBS_INDEX    28
#define WMI_SVC_RDY_CRM_SUB_ID_LSBS_NUM_BITS 4
#define WMI_SVC_RDY_CRM_SUB_ID_MSBS_INDEX    10
#define WMI_SVC_RDY_CRM_SUB_ID_MSBS_NUM_BITS 3
#define WMI_SVC_RDY_CRM_SUB_ID_GET(var, val) \
    WMI_APPEND_TWO_GET_BITS( \
        var, \
        WMI_SVC_RDY_CRM_SUB_ID_LSBS_INDEX, \
        WMI_SVC_RDY_CRM_SUB_ID_LSBS_NUM_BITS, \
        WMI_SVC_RDY_CRM_SUB_ID_MSBS_INDEX, \
        WMI_SVC_RDY_CRM_SUB_ID_MSBS_NUM_BITS, \
        val)
#define WMI_SVC_RDY_CRM_SUB_ID_SET(var, val) \
    WMI_APPEND_TWO_SET_BITS( \
        var, \
        WMI_SVC_RDY_CRM_SUB_ID_LSBS_INDEX, \
        WMI_SVC_RDY_CRM_SUB_ID_LSBS_NUM_BITS, \
        WMI_SVC_RDY_CRM_SUB_ID_MSBS_INDEX, \
        WMI_SVC_RDY_CRM_SUB_ID_MSBS_NUM_BITS, \
        val)

/*
 * regdb version GET/SET APIs
 */
#define WMI_REG_DB_VERSION_MAJOR_BITPOS 0
#define WMI_REG_DB_VERSION_MINOR_BITPOS 8
#define WMI_BDF_REG_DB_VERSION_MAJOR_BITPOS 16
#define WMI_BDF_REG_DB_VERSION_MINOR_BITPOS 24
#define WMI_REG_DB_VERSION_NUM_BITS 8

#define WMI_REG_DB_VERSION_MAJOR_GET(dword) \
    WMI_GET_BITS(dword, WMI_REG_DB_VERSION_MAJOR_BITPOS, WMI_REG_DB_VERSION_NUM_BITS)
#define WMI_REG_DB_VERSION_MAJOR_SET(dword, value) \
    WMI_SET_BITS(dword, WMI_REG_DB_VERSION_MAJOR_BITPOS, WMI_REG_DB_VERSION_NUM_BITS, value)

#define WMI_REG_DB_VERSION_MINOR_GET(dword) \
    WMI_GET_BITS(dword, WMI_REG_DB_VERSION_MINOR_BITPOS, WMI_REG_DB_VERSION_NUM_BITS)
#define WMI_REG_DB_VERSION_MINOR_SET(dword, value) \
    WMI_SET_BITS(dword, WMI_REG_DB_VERSION_MINOR_BITPOS, WMI_REG_DB_VERSION_NUM_BITS, value)

#define WMI_BDF_REG_DB_VERSION_MAJOR_GET(dword) \
    WMI_GET_BITS(dword, WMI_BDF_REG_DB_VERSION_MAJOR_BITPOS, WMI_REG_DB_VERSION_NUM_BITS)
#define WMI_BDF_REG_DB_VERSION_MAJOR_SET(dword, value) \
    WMI_SET_BITS(dword, WMI_BDF_REG_DB_VERSION_MAJOR_BITPOS, WMI_REG_DB_VERSION_NUM_BITS, value)

#define WMI_BDF_REG_DB_VERSION_MINOR_GET(dword) \
    WMI_GET_BITS(dword, WMI_BDF_REG_DB_VERSION_MINOR_BITPOS, WMI_REG_DB_VERSION_NUM_BITS)
#define WMI_BDF_REG_DB_VERSION_MINOR_SET(dword, value) \
    WMI_SET_BITS(dword, WMI_BDF_REG_DB_VERSION_MINOR_BITPOS, WMI_REG_DB_VERSION_NUM_BITS, value)

#define WMI_REG_DB_VERSION_SET(dword, reg_db_ver_major, reg_db_ver_minor, bdf_reg_db_ver_major, bdf_reg_db_ver_minor) \
    do { \
        WMI_REG_DB_VERSION_MAJOR_SET(dword, reg_db_ver_major); \
        WMI_REG_DB_VERSION_MINOR_SET(dword, reg_db_ver_minor); \
        WMI_BDF_REG_DB_VERSION_MAJOR_SET(dword, bdf_reg_db_ver_major); \
        WMI_BDF_REG_DB_VERSION_MINOR_SET(dword, bdf_reg_db_ver_minor); \
    } while (0)

#define WMI_HW_MIN_TX_POWER_BITPOS  0
#define WMI_HW_MAX_TX_POWER_BITPOS  16

#define WMI_HW_MIN_TX_POWER_GET(dword) \
    ((A_INT16) WMI_GET_BITS(dword, WMI_HW_MIN_TX_POWER_BITPOS, 16))
#define WMI_HW_MIN_TX_POWER_SET(dword, value) \
    WMI_SET_BITS(dword, WMI_HW_MIN_TX_POWER_BITPOS, 16, value)
#define WMI_HW_MAX_TX_POWER_GET(dword) \
    ((A_INT16) WMI_GET_BITS(dword, WMI_HW_MAX_TX_POWER_BITPOS, 16))
#define WMI_HW_MAX_TX_POWER_SET(dword, value) \
    WMI_SET_BITS(dword, WMI_HW_MAX_TX_POWER_BITPOS, 16, value)

#define WMI_MAX_USER_PER_PPDU_UL_OFDMA_GET(dword) \
        WMI_GET_BITS(dword, 0, 16)

#define WMI_MAX_USER_PER_PPDU_UL_OFDMA_SET(dword, value) \
        WMI_SET_BITS(dword, 0, 16, value)

#define WMI_MAX_USER_PER_PPDU_DL_OFDMA_GET(dword) \
        WMI_GET_BITS(dword, 16, 16)

#define WMI_MAX_USER_PER_PPDU_DL_OFDMA_SET(dword, value) \
        WMI_SET_BITS(dword, 16, 16, value)

#define WMI_MAX_USER_PER_PPDU_UL_MUMIMO_GET(dword) \
        WMI_GET_BITS(dword, 0, 16)

#define WMI_MAX_USER_PER_PPDU_UL_MUMIMO_SET(dword, value) \
        WMI_SET_BITS(dword, 0, 16, value)

#define WMI_MAX_USER_PER_PPDU_DL_MUMIMO_GET(dword) \
        WMI_GET_BITS(dword, 16, 16)

#define WMI_MAX_USER_PER_PPDU_DL_MUMIMO_SET(dword, value) \
        WMI_SET_BITS(dword, 16, 16, value)

#define WMI_TARGET_CAP_FLAGS_RX_PEER_METADATA_VERSION_GET(target_cap_flags) \
        WMI_GET_BITS(target_cap_flags, 0, 2)
#define WMI_TARGET_CAP_FLAGS_RX_PEER_METADATA_VERSION_SET(target_cap_flags, value) \
        WMI_SET_BITS(target_cap_flags, 0, 2, value)

#define WMI_TARGET_CAP_UL_MU_MIMO_RX_SUPPORT_2GHZ_GET(target_cap_flags) \
    WMI_GET_BITS(target_cap_flags, 2, 1)
#define WMI_TARGET_CAP_UL_MU_MIMO_RX_SUPPORT_2GHZ_SET(target_cap_flags, value) \
    WMI_SET_BITS(target_cap_flags, 2, 1, value)

#define WMI_TARGET_CAP_UL_MU_MIMO_TX_SUPPORT_2GHZ_GET(target_cap_flags) \
    WMI_GET_BITS(target_cap_flags, 3, 1)
#define WMI_TARGET_CAP_UL_MU_MIMO_TX_SUPPORT_2GHZ_SET(target_cap_flags, value) \
    WMI_SET_BITS(target_cap_flags, 3, 1, value)

#define WMI_TARGET_CAP_UL_MU_MIMO_RX_SUPPORT_5GHZ_GET(target_cap_flags) \
    WMI_GET_BITS(target_cap_flags, 4, 1)
#define WMI_TARGET_CAP_UL_MU_MIMO_RX_SUPPORT_5GHZ_SET(target_cap_flags, value) \
    WMI_SET_BITS(target_cap_flags, 4, 1, value)

#define WMI_TARGET_CAP_UL_MU_MIMO_TX_SUPPORT_5GHZ_GET(target_cap_flags) \
    WMI_GET_BITS(target_cap_flags, 5, 1)
#define WMI_TARGET_CAP_UL_MU_MIMO_TX_SUPPORT_5GHZ_SET(target_cap_flags, value) \
    WMI_SET_BITS(target_cap_flags, 5, 1, value)

#define WMI_TARGET_CAP_UL_MU_MIMO_RX_SUPPORT_6GHZ_GET(target_cap_flags) \
    WMI_GET_BITS(target_cap_flags, 6, 1)
#define WMI_TARGET_CAP_UL_MU_MIMO_RX_SUPPORT_6GHZ_SET(target_cap_flags, value) \
    WMI_SET_BITS(target_cap_flags, 6, 1, value)

#define WMI_TARGET_CAP_UL_MU_MIMO_TX_SUPPORT_6GHZ_GET(target_cap_flags) \
    WMI_GET_BITS(target_cap_flags, 7, 1)
#define WMI_TARGET_CAP_UL_MU_MIMO_TX_SUPPORT_6GHZ_SET(target_cap_flags, value) \
    WMI_SET_BITS(target_cap_flags, 7, 1, value)

#define WMI_TARGET_CAP_MAX_ML_BSS_NUM_GET(target_cap_flags) \
    WMI_GET_BITS(target_cap_flags, 8, 3)
#define WMI_TARGET_CAP_MAX_ML_BSS_NUM_SET(target_cap_flags, value) \
    WMI_SET_BITS(target_cap_flags, 8, 3, value)

#define WMI_TARGET_CAP_CONCURRENCE_SUPPORT_GET(target_cap_flags) \
    WMI_GET_BITS(target_cap_flags, 11, 2)
#define WMI_TARGET_CAP_CONCURRENCE_SUPPORT_SET(target_cap_flags, value) \
    WMI_SET_BITS(target_cap_flags, 11, 2, value)

#define WMI_TARGET_CAP_MULTIPASS_SAP_SUPPORT_GET(target_cap_flags) \
    WMI_GET_BITS(target_cap_flags, 13, 1)
#define WMI_TARGET_CAP_MULTIPASS_SAP_SUPPORT_SET(target_cap_flags, value) \
    WMI_SET_BITS(target_cap_flags, 13, 1, value)

/*
 * wmi_htt_msdu_idx_to_htt_msdu_qtype GET/SET APIs
 */
#define WMI_HTT_MSDUQ_IDX_TO_MSDUQ_QTYPE_INDEX_GET(index_and_type) \
    WMI_GET_BITS(index_and_type, 0, 8)
#define WMI_HTT_MSDUQ_IDX_TO_MSDUQ_QTYPE_INDEX_SET(index_and_type, value) \
    WMI_SET_BITS(index_and_type, 0, 8, value)

#define WMI_HTT_MSDUQ_IDX_TO_MSDUQ_QTYPE_TYPE_GET(index_and_type) \
    WMI_GET_BITS(index_and_type, 8, 8)
#define WMI_HTT_MSDUQ_IDX_TO_MSDUQ_QTYPE_TYPE_SET(index_and_type, value) \
    WMI_SET_BITS(index_and_type, 8, 8, value)

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_htt_msdu_idx_to_htt_msdu_qtype.*/
    /**
     * index_and_type
     *
     * [7:0]   : htt_msduq_index
     * [15:8]  : htt_msduq_type
     * [31:16] : reserved
     */
    A_UINT32 index_and_type;
} wmi_htt_msdu_idx_to_htt_msdu_qtype;

typedef enum {
    WMI_AFC_FEATURE_6G_DEPLOYMENT_UNSPECIFIED = 0,
    WMI_AFC_FEATURE_6G_DEPLOYMENT_INDOOR_ONLY =  1,
    WMI_AFC_FEATURE_6G_DEPLOYMENT_OUTDOOR_ONLY = 2,
} WMI_AFC_FEATURE_6G_DEPLOYMENT_TYPE;

typedef enum {
    WMI_BDF_VERSION_CHECK_DISABLED = 0,

    /* WMI_BDF_VERSION_CHECK_GOOD:
     * BDF version is matched with FW.
     */
    WMI_BDF_VERSION_CHECK_GOOD = 1,

    /* WMI_BDF_VERSION_TEMPLATE_TOO_OLD:
     * BDF template version is older than the oldest version supported by FW.
     */
    WMI_BDF_VERSION_TEMPLATE_TOO_OLD = 2,

    /* WMI_BDF_VERSION_TEMPLATE_TOO_NEW:
     * BDF template version is newer than the newest version supported by FW.
     */
    WMI_BDF_VERSION_TEMPLATE_TOO_NEW = 3,

    /* WMI_BDF_VERSION_FW_TOO_OLD:
     * FW version is older than the major version supported by BDF.
     */
    WMI_BDF_VERSION_FW_TOO_OLD = 4,

    /* WMI_BDF_VERSION_FW_TOO_NEW:
     * FW version is newer than the minor version supported by BDF.
     */
    WMI_BDF_VERSION_FW_TOO_NEW = 5,
} wmi_bdf_version_status_type;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_service_ready_ext2_event_fixed_param.*/

    /*
     * regDB Version to be sent to Host on WMI service ready ext2 event.
     *    [7:0]   - regDbVersionMajor
     *    [15:8]  - regDbVersionMinor
     *    [23:16] - bdfRegDbVersionMajor
     *    [31:24] - bdfRegDbVersionMinor
     * The WMI_*REG_DB_VERSION_[MAJOR,MINOR]_[SET,GET] macros are used to
     * access these bitfields.
     */
    A_UINT32 reg_db_version;

    /* Min & Max Tx power (in dBm) supported in 2.4 GHz band
     *  [15:0]   - Min Tx Power in 2.4 GHz band
     *  [31:16]  - Max Tx Power in 2.4 GHz band
     * WMI_HW_[MIN,MAX]_TX_POWER_[GET,SET] macros are used to access
     * these bitfields.
     * If Min Tx Power = Max Tx Power = 0 means Min Tx Power & Max Tx Power
     * are not specified.
     */
    A_UINT32 hw_min_max_tx_power_2g;

    /* Min & Max Tx power (in dBm) supported in 5 GHz band
     *  [15:0]   - Min Tx Power in 5 GHz band
     *  [31:16]  - Max Tx Power in 5 GHz band
     * WMI_HW_[MIN,MAX]_TX_POWER_[GET,SET] macros are used to access
     * these bitfields.
     * If Min Tx Power = Max Tx Power = 0 means Min Tx Power & Max Tx Power
     * are not specified.
     */
    A_UINT32 hw_min_max_tx_power_5g;

    /*
     * Number of peers supported per WMI_PEER_CHAN_WIDTH_SWITCH_CMDID
     * 0 - not enabled
     */
    A_UINT32 chwidth_num_peer_caps;

    /*
     * Whether preamble puncturing is supported by FW, and if so, for which
     * bandwidths.  The possible values for this field are listed below.
     *   0: preamble puncturing is not supported
     *  80: puncturing supported within channels of at least 80 MHz bandwidth
     * 160: puncturing supported within channels of at least 160 MHz bandwidth
     * 320: puncturing supported within 320 MHz channels
     */
    A_UINT32 preamble_puncture_bw;

    /*
     * [15:0]  - ULOFDMA Refer WMI_MAX_USER_PER_PPDU_UL_OFDMA_GET & SET
     * [31:16] - DLOFDMA Refer WMI_MAX_USER_PER_PPDU_DL_OFDMA_GET & SET
     * If max_user_per_ppdu_ofdma == 0 the UL/DL max users are unspecified.
     */
    A_UINT32 max_user_per_ppdu_ofdma;

    /*
     * [15:0]  - ULMUMIMO Refer WMI_MAX_USER_PER_PPDU_UL_MUMIMO_GET & SET
     * [31:16] - DLMUMIMO Refer WMI_MAX_USER_PER_PPDU_DL_MUMIMO_GET & SET
     * If max_user_per_ppdu_mumimo == 0 the UL/DL max users are unspecified.
     */
    A_UINT32 max_user_per_ppdu_mumimo;

    /**
     * @brief target_cap_flags - flags containing information about target capabilities.
     * Bits 1:0
     *    Rx peer metadata version number used by target
     *    0-> legacy case
     *    1-> MLO support
     *    2,3-> reserved
     *    Refer to WMI_TARGET_CAP_FLAGS_PEER_METADATA_VERSION macros.
     * Bit 2 - UL MUMIMO Rx support on 2.4 GHz (AP Mode)
     * Bit 3 - UL MUMIMO Tx support on 2.4 GHz (STA Mode)
     * Bit 4 - UL MUMIMO Rx support on 5 GHz (AP Mode)
     * Bit 5 - UL MUMIMO Tx support on 5 GHz (STA Mode)
     * Bit 6 - UL MUMIMO Rx support on 6 GHz (AP Mode)
     * Bit 7 - UL MUMIMO Tx support on 6 GHz (STA Mode)
     * Bits 10:8 - max ML BSS number supported, range [0-7]
     * Bits 12:11  concurrence support capability
     *      Bit11 - [ML-STA + SL-STA]  0: not supported; 1:supported
     *      Bit12 - [ML-STA + SL-SAP]  0: not supported; 1:supported
     * Bit 13 - Support for multipass SAP
     * Bits 31:14 - Reserved
     */
    A_UINT32 target_cap_flags;

    /* EHT MAC Capabilities: total WMI_MAX_EHTCAP_MAC_SIZE*A_UINT32 bits
     * those bits actually are max mac capabilities = cap_mac_2g | cap_mac_5g
     * The actual cap mac info per mac (2g/5g) in the TLV -- WMI_MAC_PHY_CAPABILITIES_EXT
     */
    A_UINT32 eht_cap_mac_info[WMI_MAX_EHTCAP_MAC_SIZE];

    /* Following this struct are the TLV's:
     *     WMI_DMA_RING_CAPABILITIES;
     *     wmi_spectral_bin_scaling_params;
     *     WMI_MAC_PHY_CAPABILITIES_EXT;  <-- EHT mac capabilities and phy capabilities info
     *     WMI_HAL_REG_CAPABILITIES_EXT2;
     *     wmi_nan_capabilities;
     *     WMI_SCAN_RADIO_CAPABILITIES_EXT2;
     */

    /*
     * Max number of LinkView peers supported by target
     */
    A_UINT32 max_num_linkview_peers;

    /*
     * Max number of msduq's per TID per peer supported by target,
     * defines LinkView peers number
     */
    A_UINT32 max_num_msduq_supported_per_tid;

    /*
     * Number of peers support default flowqs
     */
    A_UINT32 default_num_msduq_supported_per_tid;

    /*
     * Indoor/Outdoor specification for AFC support -
     * refer to WMI_AFC_FEATURE_6G_DEPLOYMENT_TYPE enum
     */
    A_UINT32 afc_deployment_type;

    /*
     * Board data check report. Please see wmi_hw_bd_status_type enum values.
     */
    A_UINT32 hw_bd_status;

    /*
     * max block ack window size FW supports for tx.
     */
    A_UINT32 tx_aggr_ba_win_size_max;

    /*
     * max block ack window size FW supports for rx.
     */
    A_UINT32 rx_aggr_ba_win_size_max;

    /*
     * max link number per MLD FW supports.
     */
    A_UINT32 num_max_mlo_link_per_ml_bss_supp;

    /* Followed by next TLVs:
     *     WMI_DMA_RING_CAPABILITIES          dma_ring_caps[];
     *     wmi_spectral_bin_scaling_params    wmi_bin_scaling_params[];
     *     WMI_MAC_PHY_CAPABILITIES_EXT       mac_phy_caps[];
     *     WMI_HAL_REG_CAPABILITIES_EXT2      hal_reg_caps[];
     *     wmi_nan_capabilities               nan_cap;
     *     WMI_SCAN_RADIO_CAPABILITIES_EXT2   wmi_scan_radio_caps[];
     *     wmi_htt_msdu_idx_to_htt_msdu_qtype htt_msdu_idx_to_qtype_map[];
     *     wmi_dbs_or_sbs_cap_ext             dbs_or_sbs_cap_ext;
     *     A_INT32 hw_tx_power_signed[WMI_HW_TX_POWER_CAPS_MAX];
     *     wmi_aux_dev_capabilities           aux_dev_caps[];
     */
} wmi_service_ready_ext2_event_fixed_param;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_chan_rf_characterization_info_event_fixed_param */
    /*
     * A variable-length TLV array of wmi_chan_rf_characterization_info will
     * follow this fixed_param TLV, containing rx characterization info for
     * primary channels.
     * WMI_CHAN_RF_CHARACTERIZATION_INFO wmi_chan_rf_characterization_info[];
     */
} wmi_chan_rf_characterization_info_event_fixed_param;

typedef enum {
    WMI_FW_STA_RTT_INITR =     0x00000001,
    WMI_FW_STA_RTT_RESPR =     0x00000002,
    WMI_FW_P2P_CLI_RTT_INITR = 0x00000004,
    WMI_FW_P2P_CLI_RTT_RESPR = 0x00000008,
    WMI_FW_P2P_GO_RTT_INITR =  0x00000010,
    WMI_FW_P2P_GO_RTT_RESPR =  0x00000020,
    WMI_FW_AP_RTT_INITR =      0x00000040,
    WMI_FW_AP_RTT_RESPR =      0x00000080,
    WMI_FW_NAN_RTT_INITR =     0x00000100,
    WMI_FW_NAN_RTT_RESPR =     0x00000200,
    WMI_FW_SCAN_DBS_POLICY =   0x00000400,
    /*
     * New fw sub feature capabilities before
     * WMI_FW_MAX_SUB_FEAT_CAP
     */
    WMI_FW_MAX_SUB_FEAT_CAP =  0x80000000,
} WMI_FW_SUB_FEAT_CAPS;

typedef enum {
    WMI_HWBD_NONE       = 0,            /* No hw board information is given */
    WMI_HWBD_QCA6174    = 1,            /* Rome(AR6320) */
    WMI_HWBD_QCA2582    = 2,            /* Killer 1525*/
} WMI_HWBD_ID;

#define ATH_BD_DATA_REV_MASK            0x000000FF
#define ATH_BD_DATA_REV_SHIFT           0

#define ATH_BD_DATA_PROJ_ID_MASK        0x0000FF00
#define ATH_BD_DATA_PROJ_ID_SHIFT       8

#define ATH_BD_DATA_CUST_ID_MASK        0x00FF0000
#define ATH_BD_DATA_CUST_ID_SHIFT       16

#define ATH_BD_DATA_REF_DESIGN_ID_MASK  0xFF000000
#define ATH_BD_DATA_REF_DESIGN_ID_SHIFT 24

#define SET_BD_DATA_REV(bd_data_ver, value)     \
    ((bd_data_ver) &= ~ATH_BD_DATA_REV_MASK, (bd_data_ver) |= ((value) << ATH_BD_DATA_REV_SHIFT))

#define GET_BD_DATA_REV(bd_data_ver)            \
    (((bd_data_ver) & ATH_BD_DATA_REV_MASK) >> ATH_BD_DATA_REV_SHIFT)

#define SET_BD_DATA_PROJ_ID(bd_data_ver, value) \
    ((bd_data_ver) &= ~ATH_BD_DATA_PROJ_ID_MASK, (bd_data_ver) |= ((value) << ATH_BD_DATA_PROJ_ID_SHIFT))

#define GET_BD_DATA_PROJ_ID(bd_data_ver)        \
    (((bd_data_ver) & ATH_BD_DATA_PROJ_ID_MASK) >> ATH_BD_DATA_PROJ_ID_SHIFT)

#define SET_BD_DATA_CUST_ID(bd_data_ver, value) \
    ((bd_data_ver) &= ~ATH_BD_DATA_CUST_ID_MASK, (bd_data_ver) |= ((value) << ATH_BD_DATA_CUST_ID_SHIFT))

#define GET_BD_DATA_CUST_ID(bd_data_ver)        \
    (((bd_data_ver) & ATH_BD_DATA_CUST_ID_MASK) >> ATH_BD_DATA_CUST_ID_SHIFT)

#define SET_BD_DATA_REF_DESIGN_ID(bd_data_ver, value)   \
    ((bd_data_ver) &= ~ATH_BD_DATA_REF_DESIGN_ID_MASK, (bd_data_ver) |= ((value) << ATH_BD_DATA_REF_DESIGN_ID_SHIFT))

#define GET_BD_DATA_REF_DESIGN_ID(bd_data_ver)          \
    (((bd_data_ver) & ATH_BD_DATA_REF_DESIGN_ID_MASK) >> ATH_BD_DATA_REF_DESIGN_ID_SHIFT)


#ifdef ROME_LTE_COEX_FREQ_AVOID
typedef struct {
    A_UINT32 start_freq; /* start frequency, not channel center freq */
    A_UINT32 end_freq; /* end frequency */
} avoid_freq_range_desc;

typedef struct {
    /* bad channel range count, multi range is allowed, 0 means all channel clear */
    A_UINT32 num_freq_ranges;
    /* multi range with num_freq_ranges, LTE advance multi carrier, CDMA,etc */
    avoid_freq_range_desc avd_freq_range[0];
} wmi_wlan_avoid_freq_ranges_event;
#endif

/** status consists of  upper 16 bits fo A_STATUS status and lower 16 bits of module ID that returned status */
#define WLAN_INIT_STATUS_SUCCESS   0x0
#define WLAN_INIT_STATUS_GEN_FAILED   0x1
#define WLAN_GET_INIT_STATUS_REASON(status)    ((status) & 0xffff)
#define WLAN_GET_INIT_STATUS_MODULE_ID(status) (((status) >> 16) & 0xffff)

typedef A_UINT32 WLAN_INIT_STATUS;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_ready_event_fixed_param */
    wmi_abi_version fw_abi_vers;
    /*
     * mac_addr is always filled; in addition, there can be a mac_addr_list
     * TLV following this fixed_param TLV to specify additional MAC addresses,
     * for cases where the target specifies one MAC address per pdev
     * (so the host can treat the pdevs within the target as separately
     * as possible) rather than one MAC address for the whole SOC.
     */
    wmi_mac_addr mac_addr;
    A_UINT32 status;
    A_UINT32 num_dscp_table;
    /* num_extra_mac_addr -
     * how many additional MAC addresses besides the above mac_addr
     * are provided in the subsequent mac_addr_list TLV
     */
    A_UINT32 num_extra_mac_addr;
    /*
     * Total number of "real" peers (remote peers of an AP vdev,
     * BSS peer of a STA vdev, TDLS peer of a STA vdev) that FW supports.
     * If 0, then Host can use param_tlv->resource_config->num_peers as
     * total number of peers.
     */
    A_UINT32 num_total_peers;
    /*
     * Number of extra peers that Firmware adds.
     * These are self peers and/or other FW only peers that don't represent
     * a 802.11 transceiver, but instead are used for convenience, e.g. to
     * provide a pseudo-peer object for an AP vdev's bcast/mcast tx queues,
     * to allow each tx queue to belong to a peer object.
     * Peer ID can be up to num_total_peers + num_extra_peers.
     */
    A_UINT32 num_extra_peers;
    /*
     * max_ast_index - max AST index that Firmware can generate
     * max_ast_index = (ast_table_size-1), ast_table_size is dynamically chosen
     * based on num_peers configuration from Host. Hence Host needs to know the
     * max_ast_index that Firmware can generate.
     * A 0x0 value for max_ast_index means the target has not specified a limit.
     */
    A_UINT32 max_ast_index;
    /* pktlog_defs_checksum:
     * checksum computed from the definitions of the enums and structs
     * used within pktlog traces.
     * This pktlog defs checksum needs to be embedded into pktlog trace files
     * (specifically in ath_pktlog_bufhdr.version).
     *
     * If pktlog_defs_checksum is zero then it is invalid; it should be ignored
     * and ath_pktlog_bufhdr.magic_num needs to be PKTLOG_MAGIC_NUM_LEGACY
     * (i.e. 7735225).
     *
     * If pktlog_defs_checksum is non-zero then it is valid, and the host
     * should put it into the pktlog trace file header and set
     * ath_pktlog_bufhdr.magic_num as PKTLOG_MAGIC_NUM_VERSION_IS_CHECKSUM
     * (i.e. 2453506), to indicate that the file header version field contains
     * a checksum.
     */
    A_UINT32 pktlog_defs_checksum;

    /*
     * max_onchip_ast_index - max AST index that Firmware can generate
     * max_onchip_ast_index = (ast_table_size-1), where ast_table_size is
     * dynamically chosen based on num_peers configuration from Host.
     * Hence Host needs to know the max_onchip_ast_index that Firmware can
     * generate.
     * A 0x0 value for max_onchip_ast_index means the target has not specified
     * a limit.
     */
    A_UINT32 max_onchip_ast_index;

    /*
     * The maximum number of LinkView peers can be supported onsite of target,
     * based on proposed by the host configuration,
     * total number of available resources, configured peers number,
     * number of MSDUQs per LinkView peer's TID.
     * Target can reduce proposed by WMI_INIT_CMDID number, depending on
     * the target's resources availability.
     */
    A_UINT32 num_of_linkview_peers;

    /* Total number of "real" max_active_vdevs that FW supports. */
    A_UINT32 num_max_active_vdevs;

/*
 * This fixed_param TLV is followed by these additional TLVs:
 * mac_addr_list[num_extra_mac_addr];
 */
} wmi_ready_event_fixed_param;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_resource_config */
    /**
     * @brief num_vdev - number of virtual devices (VAPs) to support
     */
    A_UINT32 num_vdevs;
    /**
     * @brief num_peers - number of peer nodes to support
     */
    A_UINT32 num_peers;
    /*
     * @brief In offload mode target supports features like WOW, chatter and other
     * protocol offloads. In order to support them some functionalities like
     * reorder buffering, PN checking need to be done in target. This determines
     * maximum number of peers supported by target in offload mode
     */
    A_UINT32 num_offload_peers;
    /* @brief Number of reorder buffers available for doing target based reorder
     * Rx reorder buffering
     */
    A_UINT32 num_offload_reorder_buffs;
    /**
     * @brief num_peer_keys - number of keys per peer
     */
    A_UINT32 num_peer_keys;
    /**
     * @brief num_peer_tids - number of TIDs to provide storage for per peer.
     */
    A_UINT32 num_tids;
    /**
     * @brief ast_skid_limit - max skid for resolving hash collisions
     * @details
     *     The address search table is sparse, so that if two MAC addresses
     *     result in the same hash value, the second of these conflicting
     *     entries can slide to the next index in the address search table,
     *     and use it, if it is unoccupied.  This ast_skid_limit parameter
     *     specifies the upper bound on how many subsequent indices to search
     *     over to find an unoccupied space.
     */
    A_UINT32 ast_skid_limit;
    /**
     * @brief tx_chain_mask - the nominal chain mask for transmit
     * @details
     *     The chain mask may be modified dynamically, e.g. to operate AP tx with
     *     a reduced number of chains if no clients are associated.
     *     This configuration parameter specifies the nominal chain-mask that
     *     should be used when not operating with a reduced set of tx chains.
     */
    A_UINT32 tx_chain_mask;
    /**
     * @brief rx_chain_mask - the nominal chain mask for receive
     * @details
     *     The chain mask may be modified dynamically, e.g. for a client to use
     *     a reduced number of chains for receive if the traffic to the client
     *     is low enough that it doesn't require downlink MIMO or antenna
     *     diversity.
     *     This configuration parameter specifies the nominal chain-mask that
     *     should be used when not operating with a reduced set of rx chains.
     */
    A_UINT32 rx_chain_mask;
    /**
     * @brief rx_timeout_pri - what rx reorder timeout (ms) to use for the AC
     * @details
     *     Each WMM access class (voice, video, best-effort, background) will
     *     have its own timeout value to dictate how long to wait for missing
     *     rx MPDUs to arrive before flushing subsequent MPDUs that have already
     *     been received.
     *     This parameter specifies the timeout in milliseconds for each class .
     *     NOTE: the number of class (defined as 4) cannot be
     *     changed in the future without breaking WMI compatibility.
     */
    A_UINT32 rx_timeout_pri[4];
    /**
     * @brief rx_decap mode - what mode the rx should decap packets to
     * @details
     *     MAC can decap to RAW (no decap), native wifi or Ethernet types
     *     THis setting also determines the default TX behavior, however TX
     *     behavior can be modified on a per VAP basis during VAP init
     */
    A_UINT32 rx_decap_mode;
    /**
     * @brief  scan_max_pending_req - what is the maximum scan requests than can be queued
     */
    A_UINT32 scan_max_pending_req;

    /**
     * @brief maximum VDEV that could use BMISS offload
     */
    A_UINT32 bmiss_offload_max_vdev;

    /**
     * @brief maximum VDEV that could use offload roaming
     */
    A_UINT32 roam_offload_max_vdev;

    /**
     * @brief maximum AP profiles that would push to offload roaming
     */
    A_UINT32 roam_offload_max_ap_profiles;

    /**
     * @brief num_mcast_groups - how many groups to use for mcast->ucast conversion
     * @details
     *     The target's WAL maintains a table to hold information regarding which
     *     peers belong to a given multicast group, so that if multicast->unicast
     *     conversion is enabled, the target can convert multicast tx frames to a
     *     series of unicast tx frames, to each peer within the multicast group.
     *     This num_mcast_groups configuration parameter tells the target how
     *     many multicast groups to provide storage for within its multicast
     *     group membership table.
     */
    A_UINT32 num_mcast_groups;

    /**
     * @brief num_mcast_table_elems - size to alloc for the mcast membership table
     * @details
     *     This num_mcast_table_elems configuration parameter tells the target
     *     how many peer elements it needs to provide storage for in its
     *     multicast group membership table.
     *     These multicast group membership table elements are shared by the
     *     multicast groups stored within the table.
     */
    A_UINT32 num_mcast_table_elems;

    /**
     * @brief mcast2ucast_mode - whether/how to do multicast->unicast conversion
     * @details
     *     This configuration parameter specifies whether the target should
     *     perform multicast --> unicast conversion on transmit, and if so,
     *     what to do if it finds no entries in its multicast group membership
     *     table for the multicast IP address in the tx frame.
     *     Configuration value:
     *     0 -> Do not perform multicast to unicast conversion.
     *     1 -> Convert multicast frames to unicast, if the IP multicast address
     *          from the tx frame is found in the multicast group membership
     *          table.  If the IP multicast address is not found, drop the frame.
     *     2 -> Convert multicast frames to unicast, if the IP multicast address
     *          from the tx frame is found in the multicast group membership
     *          table.  If the IP multicast address is not found, transmit the
     *          frame as multicast.
     */
    A_UINT32 mcast2ucast_mode;


    /**
     * @brief tx_dbg_log_size - how much memory to allocate for a tx PPDU dbg log
     * @details
     *     This parameter controls how much memory the target will allocate to
     *     store a log of tx PPDU meta-information (how large the PPDU was,
     *     when it was sent, whether it was successful, etc.)
     */
    A_UINT32 tx_dbg_log_size;

    /**
     * @brief num_wds_entries - how many AST entries to be allocated for WDS
     */
    A_UINT32 num_wds_entries;

    /**
     * @brief dma_burst_size - MAC DMA burst size, e.g., on Peregrine on PCI
     * this limit can be 0 -default, 1 256B
     */
    A_UINT32 dma_burst_size;

    /**
     * @brief mac_aggr_delim - Fixed delimiters to be inserted after every MPDU
     * to account for interface latency to avoid underrun.
     */
    A_UINT32 mac_aggr_delim;
    /**
     * @brief rx_skip_defrag_timeout_dup_detection_check
     * @details
     *  determine whether target is responsible for detecting duplicate
     *  non-aggregate MPDU and timing out stale fragments.
     *
     *  A-MPDU reordering is always performed on the target.
     *
     *  0: target responsible for frag timeout and dup checking
     *  1: host responsible for frag timeout and dup checking
     */
    A_UINT32 rx_skip_defrag_timeout_dup_detection_check;

    /**
     * @brief vow_config - Configuration for VoW : No of Video Nodes to be supported
     * and Max no of descriptors for each Video link (node).
     */
    A_UINT32 vow_config;

    /**
     * @brief maximum VDEV that could use GTK offload
     */
    A_UINT32 gtk_offload_max_vdev;

    /**
     * @brief num_msdu_desc - Number of msdu descriptors target should use
     */
    A_UINT32 num_msdu_desc; /* Number of msdu desc */
    /**
     * @brief max_frag_entry - Max. number of Tx fragments per MSDU
     * @details
     *     This parameter controls the max number of Tx fragments per MSDU.
     *     This is sent by the target as part of the WMI_SERVICE_READY event
     *     and is overridden by the OS shim as required.
     */
    A_UINT32 max_frag_entries;

    /**
     * @brief num_tdls_vdevs - Max. number of vdevs that can support TDLS
     * @brief num_msdu_desc - Number of vdev that can support beacon offload
     */

    A_UINT32 num_tdls_vdevs; /* number of vdevs allowed to do tdls */

    /**
     * @brief num_tdls_conn_table_entries - Number of peers tracked by tdls vdev
     * @details
     *      Each TDLS enabled vdev can track outgoing transmits/RSSI/rates to/of
     *      peers in a connection tracking table for possible TDLS link creation
     *      or deletion. This controls the number of tracked peers per vdev.
     */
    A_UINT32  num_tdls_conn_table_entries; /* number of peers to track per TDLS vdev */
    /**
     * @brief beacon_tx_offload_max_vdev - Number of maximum beaconing vaps at any time.
     */
     A_UINT32 beacon_tx_offload_max_vdev;
     A_UINT32 num_multicast_filter_entries;
     A_UINT32 num_wow_filters; /*host can configure the number of wow filters*/

     /**
     * @brief num_keep_alive_pattern - Num of keep alive patterns configured
     * from host.
     */
     A_UINT32 num_keep_alive_pattern;
    /**
     * @brief keep_alive_pattern_size - keep alive pattern size.
     */
     A_UINT32 keep_alive_pattern_size;
    /**
     * @brief max_tdls_concurrent_sleep_sta - Number of tdls sleep sta supported
     * @details
     *      Each TDLS STA can become a sleep STA independently. This parameter
     *      mentions how many such sleep STAs can be supported concurrently.
     */
    A_UINT32 max_tdls_concurrent_sleep_sta;

    /**
     * @brief max_tdls_concurrent_buffer_sta - Number of tdls buffer sta supported
     * @details
     *      Each TDLS STA can become a buffer STA independently. This parameter
     *      mentions how many such buffer STAs can be supported concurrently.
     */
    A_UINT32 max_tdls_concurrent_buffer_sta;

    /**
     * @brief wmi_send_separate - host configures fw to send the wmi separately
     */
    A_UINT32 wmi_send_separate;

    /**
     * @brief num_ocb_vdevs - Number of vdevs used for OCB support
     */
    A_UINT32 num_ocb_vdevs;

    /**
     * @brief num_ocb_channels - The supported number of simultaneous OCB channels
     */
    A_UINT32 num_ocb_channels;

    /**
     * @brief num_ocb_schedules - The supported number of OCB schedule segments
     */
    A_UINT32 num_ocb_schedules;

    /**
     * @brief specific configuration from host, such as per platform configuration
     */
    #define WMI_RSRC_CFG_FLAG_WOW_IGN_PCIE_RST_S 0
    #define WMI_RSRC_CFG_FLAG_WOW_IGN_PCIE_RST_M 0x1

    #define WMI_RSRC_CFG_FLAG_LTEU_SUPPORT_S 1
    #define WMI_RSRC_CFG_FLAG_LTEU_SUPPORT_M 0x2

    #define WMI_RSRC_CFG_FLAG_COEX_GPIO_SUPPORT_S 2
    #define WMI_RSRC_CFG_FLAG_COEX_GPIO_SUPPORT_M 0x4

    #define WMI_RSRC_CFG_FLAG_AUX_RADIO_SPECTRAL_INTF_S 3
    #define WMI_RSRC_CFG_FLAG_AUX_RADIO_SPECTRAL_INTF_M 0x8

    #define WMI_RSRC_CFG_FLAG_AUX_RADIO_CHAN_LOAD_INTF_S 4
    #define WMI_RSRC_CFG_FLAG_AUX_RADIO_CHAN_LOAD_INTF_M 0x10

    #define WMI_RSRC_CFG_FLAG_BSS_CHANNEL_INFO_64_S 5
    #define WMI_RSRC_CFG_FLAG_BSS_CHANNEL_INFO_64_M 0x20

    #define WMI_RSRC_CFG_FLAG_ATF_CONFIG_ENABLE_S 6
    #define WMI_RSRC_CFG_FLAG_ATF_CONFIG_ENABLE_M 0x40

    #define WMI_RSRC_CFG_FLAG_IPHR_PAD_CONFIG_ENABLE_S 7
    #define WMI_RSRC_CFG_FLAG_IPHR_PAD_CONFIG_ENABLE_M 0x80

    #define WMI_RSRC_CFG_FLAG_QWRAP_MODE_ENABLE_S 8
    #define WMI_RSRC_CFG_FLAG_QWRAP_MODE_ENABLE_M 0x100

    #define WMI_RSRC_CFG_FLAG_MGMT_COMP_EVT_BUNDLE_SUPPORT_S 9
    #define WMI_RSRC_CFG_FLAG_MGMT_COMP_EVT_BUNDLE_SUPPORT_M 0x200

    #define WMI_RSRC_CFG_FLAG_TX_MSDU_ID_NEW_PARTITION_SUPPORT_S 10
    #define WMI_RSRC_CFG_FLAG_TX_MSDU_ID_NEW_PARTITION_SUPPORT_M 0x400

    #define WMI_RSRC_CFG_FLAG_TX_PPDU_STATS_ENABLE_S 11
    #define WMI_RSRC_CFG_FLAG_TX_PPDU_STATS_ENABLE_M 0x800

    #define WMI_RSRC_CFG_FLAG_TCL_CCE_DISABLE_S 12
    #define WMI_RSRC_CFG_FLAG_TCL_CCE_DISABLE_M 0x1000

    #define WMI_RSRC_CFG_FLAG_TIM_V2_SUPPORT_ENABLE_S 13
    #define WMI_RSRC_CFG_FLAG_TIM_V2_SUPPORT_ENABLE_M 0x2000

    #define WMI_RSRC_CFG_FLAG_EAPOL_REKEY_MINRATE_SUPPORT_ENABLE_S 14
    #define WMI_RSRC_CFG_FLAG_EAPOL_REKEY_MINRATE_SUPPORT_ENABLE_M 0x4000

    #define WMI_RSRC_CFG_FLAG_EAPOL_AC_OVERRIDE_VALID_S 15
    #define WMI_RSRC_CFG_FLAG_EAPOL_AC_OVERRIDE_VALID_M 0x8000

    /*
     * If the AC override valid bit is set then this field will specify the
     * access category to use for EAPOL frames
     * 0 - WMM_AC_BE
     * 1 - WMM_AC_BK
     * 2 - WMM_AC_VI
     * 3 - WMM_AC_VO
     */
    #define WMI_RSRC_CFG_FLAG_EAPOL_AC_OVERRIDE_S 16
    #define WMI_RSRC_CFG_FLAG_EAPOL_AC_OVERRIDE_M 0x30000

    /*
     * If TX_ACK_RSSI is set, then the target should populate the ack_rssi
     * field within the WMI_MGMT_TX_COMPLETION_EVENT message, the ack_rssi
     * TLV within the WMI_MGMT_TX_BUNDLE_COMPLETION_EVENT message, and the
     * "MSDU ACK RSSI" array within the HTT_T2H TX_COMPL_IND message.
     */
    #define WMI_RSRC_CFG_FLAG_TX_ACK_RSSI_S 18
    #define WMI_RSRC_CFG_FLAG_TX_ACK_RSSI_M 0x40000

    /*
     * If HTT_H2T_NO_HTC_HDR_LEN_IN_MSG_LEN is set, the host will not
     * include the HTC header length in the payload length for all HTT_H2T
     * messages.
     * Otherwise, only when sending HTT_H2T_MSG_TYPE_TX_FRM message,
     * payload length includes HTC header length. Other HTT_H2T messages'
     * payload length does not include HTC header length.
     * The host will only set this HTT_H2T_NO_HTC_HDR_LEN_IN_MSG_LEN flag
     * if the target has set the WMI_SERVICE_HTT_H2T_NO_HTC_HDR_LEN_IN_MSG_LEN
     * flag to indicate its support for this option.
     */
    #define WMI_RSRC_CFG_FLAG_HTT_H2T_NO_HTC_HDR_LEN_IN_MSG_LEN_S 19
    #define WMI_RSRC_CFG_FLAG_HTT_H2T_NO_HTC_HDR_LEN_IN_MSG_LEN_M 0x80000

    #define WMI_RSRC_CFG_FLAG_PEER_UNMAP_RESPONSE_SUPPORT_S 20
    #define WMI_RSRC_CFG_FLAG_PEER_UNMAP_RESPONSE_SUPPORT_M 0x100000

    /*
     * If this HTT_PEER_STATS is set, then the target should use the
     * the HTT_T2H_MSG_TYPE_PEER_STATS_IND message to upload peer stats;
     * else the target should avoid sending the PEER_STATS_IND message.
     */
    #define WMI_RSRC_CFG_FLAG_HTT_PEER_STATS_S 21
    #define WMI_RSRC_CFG_FLAG_HTT_PEER_STATS_M 0x200000

    /*
     * If this BIT is set, then the target should use peer_tid_ext to analyze
     * per peer per tid extended configurations
     */
    #define WMI_RSRC_CFG_FLAG_PEER_TID_EXT_S 22
    #define WMI_RSRC_CFG_FLAG_PEER_TID_EXT_M 0x400000

    /*
     * If the VIDEO_OVER_WIFI_ENABLE flag is set, the target will use a
     * series of algorithmic adjustments to optimize Video performance
     * by reducing latency, reducing latency jitter, and minimizing
     * dropped packets.
     */
    #define WMI_RSRC_CFG_FLAG_VIDEO_OVER_WIFI_ENABLE_S 23
    #define WMI_RSRC_CFG_FLAG_VIDEO_OVER_WIFI_ENABLE_M 0x800000

    /*
     * If the THREE_WAY_COEX_CONFIG_LEGACY flag is set, the target will use
     * the configuration parameters given by Host driver to WLAN FW and
     * apply them along with the existing CoEx Weights Override logic to
     * prioritize the WLAN-BT-Zigbee packets accordingly.
     *
     * The host shall only set the THREE_WAY_COEX_CONFIG_LEGACY_SUPPORT
     * RSRC_CFG flag if the target has set the WMI_SERVICE
     * THREE_WAY_COEX_CONFIG_LEGACY_SUPPORT flag.
     *
     * The logic to send GPM to BT-SOC with BT-ZB priorities remains the same.
     */
    #define WMI_RSRC_CFG_FLAG_THREE_WAY_COEX_CONFIG_LEGACY_SUPPORT_S 24
    #define WMI_RSRC_CFG_FLAG_THREE_WAY_COEX_CONFIG_LEGACY_SUPPORT_M 0x1000000

    /*
     * If the THREE_WAY_COEX_CONFIG_OVERRIDE flag is set, the target will use
     * the configuration parameters given by Host driver to WLAN FW and
     * apply them by OVERRIDing the existing CoEx Weights Override logic to
     * prioritize the WLAN-BT-Zigbee packets accordingly.
     *
     * The host shall only set the THREE_WAY_COEX_CONFIG_OVERRIDE_SUPPORT
     * RSRC_CFG flag if the target has set the WMI_SERVICE
     * THREE_WAY_COEX_CONFIG_OVERRIDE_SUPPORT flag.
     *
     * The logic to send GPM to BT-SOC with BT-ZB priorities remains the same.
     */
    #define WMI_RSRC_CFG_FLAG_THREE_WAY_COEX_CONFIG_OVERRIDE_SUPPORT_S 25
    #define WMI_RSRC_CFG_FLAG_THREE_WAY_COEX_CONFIG_OVERRIDE_SUPPORT_M 0x2000000

    /*
     * If the TX_COMPLETION_TX_TSF64 flag is set, the target should
     * populate the htt_tx_compl_ind_append_tx_tsf64 array within the
     * HTT_T2H TX_COMPL_IND message.
     */
    #define WMI_RSRC_CFG_FLAG_TX_COMPLETION_TX_TSF64_ENABLE_S 26
    #define WMI_RSRC_CFG_FLAG_TX_COMPLETION_TX_TSF64_ENABLE_M 0x4000000

    /*
     * If this BIT is set, then the target should support Packet capture(SMART MU FR)
     */
    #define WMI_RSRC_CFG_FLAG_PACKET_CAPTURE_SUPPORT_S 27
    #define WMI_RSRC_CFG_FLAG_PACKET_CAPTURE_SUPPORT_M 0x8000000

    /*
     * If this BIT is set, then target should support BSS Max Idle period.
     */
    #define WMI_RSRC_CFG_FLAG_BSS_MAX_IDLE_TIME_SUPPORT_S 28
    #define WMI_RSRC_CFG_FLAG_BSS_MAX_IDLE_TIME_SUPPORT_M 0x10000000

    /*
     * If this bit is set, then target should use the audio sync feature.
     * Host should only set this bit if the target has indicated via the
     * WMI_SERVICE_AUDIO_SYNC_SUPPORT flag that it supports audio sync.
     */
    #define WMI_RSRC_CFG_FLAG_AUDIO_SYNC_SUPPORT_S  29
    #define WMI_RSRC_CFG_FLAG_AUDIO_SYNC_SUPPORT_M 0x20000000

    /*
     * If this BIT is set, then the target should disable IPA
     */
    #define WMI_RSRC_CFG_FLAG_IPA_DISABLE_S 30
    #define WMI_RSRC_CFG_FLAG_IPA_DISABLE_M 0x40000000

    /*
     * If this bit is set, target should use the PCIe GEN switching feature.
     */
    #define WMI_RSRC_CFG_FLAG_PCIE_GEN_SWITCH_CAPABLITY_S 31
    #define WMI_RSRC_CFG_FLAG_PCIE_GEN_SWITCH_CAPABLITY_M 0x80000000

    A_UINT32 flag1;

    /** @brief smart_ant_cap - Smart Antenna capabilities information
     * @details
     *        1 - Smart antenna is enabled.
     *        0 - Smart antenna is disabled.
     * In future this can contain smart antenna specific capabilities.
     */
    A_UINT32 smart_ant_cap;

    /**
     * User can configure the buffers allocated for each AC (BE, BK, VI, VO)
     * during init
     */
    A_UINT32 BK_Minfree;
    A_UINT32 BE_Minfree;
    A_UINT32 VI_Minfree;
    A_UINT32 VO_Minfree;

    /**
     * @brief alloc_frag_desc_for_data_pkt . Controls data packet fragment
     * descriptor memory allocation.
     *   1 - Allocate fragment descriptor memory for data packet in firmware.
     *       If host wants to transmit data packet at its desired rate,
     *       this field must be set.
     *   0 - Don't allocate fragment descriptor for data packet.
     */
    A_UINT32 alloc_frag_desc_for_data_pkt;

    /** how much space to allocate for NDP NS (neighbor solicitation) specs */
    A_UINT32 num_ns_ext_tuples_cfg;

    /**
     * size (in bytes) of the buffer the FW shall allocate per vdev
     * firmware can dynamic allocate memory (or disable)
     * packet filtering feature.
     * 0 - fw chooses its default value
     * -1 (0XFFFFFFFF) - disable APF
     */
    A_UINT32 bpf_instruction_size;

    /**
     * Maximum no of BSSID based RX filters host would program
     * Value 0 means host doesn't given any limit to FW.
     */
    A_UINT32 max_bssid_rx_filters;
    /**
     * Use PDEV ID instead of MAC ID, added for backward compatibility with older host
     * which is using MAC ID. 1 means PDEV ID, 0 means MAC ID.
     */
    A_UINT32 use_pdev_id;

    /** Maximum number of scan clients whose DBS scan duty cycle can be configured */
    A_UINT32 max_num_dbs_scan_duty_cycle;

    /** Maximum number of Multi group key to support */
    A_UINT32 max_num_group_keys;

    union {
        A_UINT32 peer_map_unmap_v2_support; /* old name */
        /**
         * HTT peer map/unmap format support (map 4bits and unmap 4bits)
         * 0x00 -> host use default map/unmap only.
         * 0x01 -> legacy value that is interpreted the same as 0x22.
         * 0x22 -> host supports HTT peer map/unmap v2 format; the target is
         *         allowed but not required to use peer map/unmap v2 format.
         * 0x32 -> host supports HTT peer map v3 format; the target is
         *         allowed but not required to use peer map v3 format and
         *         peer unmap v2 format.
         */
        A_UINT32 peer_map_unmap_versions;
    };

    /** Sched config params for all pdevs
     * These tx scheduling configuration parameters are currently only
     * used for internal testing purposes; therefore the non-default
     * values for this field are not currently documented.
     * For regular use, this field should be set to 0x0.
     */
    A_UINT32 sched_params;

    /* Number of MAC on which AP TWT feature is supported */
    A_UINT32 twt_ap_pdev_count;

    /* Max no of STA with which TWT sessions can be formed by the AP */
    A_UINT32 twt_ap_sta_count;

    /* max_nlo_ssids - dynamically negotiated maximum number of SSIDS for NLO
     * This parameter provides the final specification for the maximum number
     * of SSIDs for the target to support for Network List Offload's scanning
     * for preferred networks.
     * This wmi_resource_config.max_nlo_ssids must be <= the max_nlo_ssids
     * field from the target's WMI_SERVICE_READY_EXT_EVENT message.
     * (If the target didn't provide a max_nlo_ssids field in the
     * WMI_SERVICE_READY_EXT message, or if the SERVICE_READY_EXT msg's
     * max_nlo_ssids value was 0x0, the target doesn't support dynamic
     * negotiation of max NLO SSIDs, and WMI_NLO_MAX_SSIDS (=16) applies.)
     * If this wmi_resource_config.max_nlo_ssids field is absent or 0x0,
     * the host does not support dynamic negotiation of max NLO SSIDs.
     * In such a case, the target will respond as follows:
     * If the target supports at least WMI_NLO_MAX_SSIDS, the target will
     * use the statically-configured WMI_NLO_MAX_SSIDS value.
     * If the target supports less than WMI_NLO_MAX_SSIDS, the target will
     * abort its boot-up, due to receiving an invalid/unsupported
     * configuration specification.
     */
    A_UINT32 max_nlo_ssids;

    /**
     * num_packet_filters: the num that host requests fw to support for
     * pktfilter in total, then firmware can dynamic allocate
     * memory(or disable) pktfilter feature.
     *
     * 0 -  fw chooses its default value.
     * -1(0XFFFFFFFF)- disable pktfilter.
     */
    A_UINT32 num_packet_filters;

    /**
     * num_max_sta_vdevs: the max num for the sta vdevs
     * fw will use it to config the memory of offload features that
     * are only for sta vdevs.
     * p2p client should be included.
     *
     *  0 - fw chooses its default value: 'num_vdevs' of this structure.
     */
    A_UINT32 num_max_sta_vdevs;

    /* ref to section 8.4.2.48 Multiple BSSID element
     * The Max BSSID Indicator field contains a value assigned to n,
     * where 2^n is the maximum number of BSSIDs
     */
    A_UINT32 max_bssid_indicator;

    /** @brief ul_resp_config - Configures the 11ax uplink ofdma feature on STA.
     *         I.e. sending uplink response to a trigger frame sent by AP.
     *  @details
     *        0 - fw default behavior, based on chipset
     *        1 - UL_RESP is disabled.
     *        2 - UL_RESP is enabled.
     *        other - reserved.
     */
    A_UINT32 ul_resp_config;

    /* msdu_flow_override_config0 - contains AST enable bitmask
     * AST0 is unconditionally enabled, unless the MSDU flow override feature
     * is entirely disabled.
     * AST1 through AST3 are conditionally enabled, based on bits 0-2 in
     * msdu_flow_override_config0.
     * If all three bits are 0, no msdu flow override feature at all in FW.
     *
     * The WMI_MSDU_FLOW_AST_ENABLE_GET and WMI_MSDU_FLOW_AST_ENABLE_SET
     * macros are used to read and write these bitfields.
     */
    A_UINT32 msdu_flow_override_config0;

     /* msdu_flow_override_config1:
      * Bits 3:0   - AST0_FLOW_MASK(4)
      * Bits 7:4   - AST1_FLOW_MASK(4)
      * Bits 11:8  - AST2_FLOW_MASK(4)
      * Bits 15:12 - AST3_FLOW_MASK(4)
      * Bits 23:16 - TID_VALID_HI_PRI (8)
      * Bits 31:24 - TID_VALID_LOW_PRI (8)
      *
      * The macros
      * WMI_MSDU_FLOW_ASTX_MSDU_FLOW_MASKS_GET
      * WMI_MSDU_FLOW_ASTX_MSDU_FLOW_MASKS_SET
      * WMI_MSDU_FLOW_TID_VALID_HI_MASKS_GET
      * WMI_MSDU_FLOW_TID_VALID_HI_MASKS_SET
      * WMI_MSDU_FLOW_TID_VALID_LOW_MASKS_GET
      * WMI_MSDU_FLOW_TID_VALID_LOW_MASKS_SET
      * are used to read and write these bitfields.
      */
    A_UINT32 msdu_flow_override_config1;

    /** @brief flags2 - contains flags used for the following purposes:
     *  Configure 11ax uplink ofdma/MU-MIMO feature in FW, when chipsets
     *  are brought up in Repeater/STA mode.
     *
     *  @details
     *  Bits  3:0
     *      Enable UL MU-OFDMA/MIMO for PDEVs WIFI0, WIFI1, WIFI2
     *      This flags should only be set when a pdev has STA VAP
     *      in repeater/self-organizing-network modes.
     *      E.g. to enable UL RESP for 5G and 2G radios, value shall be
     *      0b00000000000000000000000000000011 = 0x3.
     *      Host shall use UCI config for a radio to populate this value,
     *      each radio entry shall have "config re_ul_resp 1" value set.
     *      Hence this can be configured dynamically.
     *
     *      Refer to the below WMI_RSRC_CFG_FLAGS2_RE_ULRESP_PDEV_CFG_GET/SET
     *      macros.
     *  Bits 5:4
     *      HTT rx peer metadata version number that host supports.
     *      Firmware initially sends the target supported version number
     *      as part of service_ready_ext2 message.
     *      Host can ack the version number that it is using as part of
     *      this message.
     *      0-> legacy case
     *      1-> MLO support
     *      2-3-> Reserved
     *      Refer to the WMI_RSRC_CFG_FLAGS2_RX_PEER_METADATA_VERSION macros.
     *  Bit 6 - is_sap_connected_d3wow_enabled
     *      Enable D3WoW for SAP w/ clients connected
     *      0-> disable the feature
     *      1-> enable the feature
     *      Refer to the WMI_RSRC_CFG_FLAGS2_IS_SAP_CONNECTED_D3WOW_ENABLED
     *      GET/SET macros.
     *  Bit 7 - is_go_connected_d3wow_enabled
     *      Enable D3WoW for GO w/ clients connected
     *      0-> disable the feature
     *      1-> enable the feature
     *      Refer to the WMI_RSRC_CFG_FLAGS2_IS_GO_CONNECTED_D3WOW_ENABLED
     *      GET/SET macros.
     *  Bit 8 - enable_dynamic_pcie_gen_speed_switch
     *      enable dynamic pcie gen speed switch
     *      0-> disable the feature
     *      1-> enable the feature
     *      Refer to the WMI_RSRC_CFG_FLAGS2_IS_DYNAMIC_PCIE_GEN_SPEED_SWITCH_ENABLED
     *      GET/SET macros.
     *  Bit 9 - calc_next_dtim_count
     *      0 -> disable calculation of DTIM count for MBSSID_NON_TX_VAP
     *      1 -> Used by some hosts to indicate calculation of DTIM count
     *           for MBSSID_NON_TX_VAP
     *      Refer to WMI_RSRC_CFG_FLAGS2_CALC_NEXT_DTIM_COUNT_GET/SET macros.
     *
     * Bit 10 - arp_ac_override_valid
     *      0 -> arp_ac_override field is invalid
     *      1 -> arp_ac_override field is valid
     *      Refer to WMI_RSRC_CFG_FLAGS2_ARP_AC_OVERRIDE_VALID_GET/SET macros.
     * Bit 12:11 - arp_ac_override
     *      If the AC override valid bit is set then this field will specify the
     *      access category to use for ARP frames
     *      0 - WMM_AC_BE
     *      1 - WMM_AC_BK
     *      2 - WMM_AC_VI
     *      3 - WMM_AC_VO
     *      Refer to WMI_RSRC_CFG_FLAGS2_ARP_AC_OVERRIDE_GET/SET macros.
     * Bit 13 - sawf_sched
     *      SAWF feature flag for scheduler
     *      0  -> disable SAWF based scheduling
     *      1  -> enable SAWF based scheduling
     *      Refer to WMI_RSRC_CFG_FLAGS2_SAWF_CONFIG_ENABLE_GET/SET macros.
     * Bit 14 - notify_frame_support
     *      Flag to enable notify_frame_support from host.
     *      0  -> disable notify_frame_support feature
     *      1  -> enable_notify_frame_support feature
     *      Refer to WMI_RSRC_CFG_FLAGS2_NOTIFY_FRAME_CONFIG_ENABLE_GET/SET
     *      macros.
     * Bit 15 - disable_wds_mec_intrabss
     *      Flag to disable wds learning, MEC, intrabss offload.
     *      By default, it is enabled.
     *      0  -> enable wds_mec_intrabss offload
     *      1  -> disable wds_mec_intrabss offload
     *      Refer to WMI_RSRC_CFG_FLAGS2_DISABLE_WDS_MEC_INTRABSS_OFFLOAD_GET /
     *      SET macros.
     * Bit 16 - latency_flowq_support
     *      Flag to indicate whether host supports latency tolerant queue.
     *      By default, it is disabled.
     *      0  -> disable latency_flowq_support
     *      1  -> enable latency_flowq_support
     *      Refer to WMI_RSRC_CFG_FLAGS2_LATENCY_FLOWQ_SUPPORT_GET/SET macros.
     * Bit 17 - rf_path_mode
     *      Flag to indicate overlapping_freq_mode
     *      By default, it will be primary mode (0)
     *      0 - Primary
     *      1 - Secondary
     *      Refer to WMI_RSRC_CFG_FLAGS2_RF_PATH_MODE_GET/SET macros.
     * Bit 18 - disable_wds_peer_map_unmap_event
     *      Flag to indicate whether the WDS peer map/unmap event should be
     *      processed or ignored.
     *      0 - leave the WDS peer map/unmap event enabled
     *      1 - disable the WDS peer map/unmap event
     *      This flag shall only be set if the target has set the
     *      WMI_SERVICE_DISABLE_WDS_PEER_MAP_UNMAP_EVENT_SUPPORT flag.
     *      Refer to WMI_RSRC_CFG_FLAGS2_DISABLE_WDS_PEER_MAP_UNMAP_EVENT_GET
     *      and _SET macros.
     *
     *  Bits 31:19 - Reserved
     */
    A_UINT32 flags2;
    /** @brief host_service_flags - can be used by Host to indicate
     * services that host can support.
     *
     *  @details
     *  Bit 0
     *      The bit will be set when Host HDD supports separate iface creation
     *      for NAN.  More specifically Host can support creation of NAN vdev
     *      in firmware.
     *
     *      Refer to WMI_RSRC_CFG_HOST_SERVICE_FLAG_NAN_IFACE_SUPPORT_GET/SET
     *      macros defined below.
     *  Bit 1
     *      The bit will be set when HOST is capable of processing multiple
     *      radio events per radio. More specifically whenever Firmware is
     *      sending multiple radio events (WMI_RADIO_LINK_STATS_EVENTID
     *      = 0x16004) for a single radio,
     *      Through this flag Firmware will know that HOST is able to support
     *      delivery of RADIO_LINK_STATS across multiple event messages,
     *      and Firmware can send multiple radio events.
     *
     *      Refer to WMI_RSRC_CFG_HOST_SERVICE_FLAG_HOST_SUPPORT_MULTI_RADIO_EVTS_PER_RADIO_GET/SET
     *      macros defined below.
     *  Bit 2
     *      This bit will be set when host is able to handle split AST feature.
     *      Refer to the below definitions of the
     *      WMI_RSRC_CFG_HOST_SERVICE_FLAG_SPLIT_AST_FEATURE_HOST_SUPPORT_GET
     *      and _SET macros.
     *  Bit 3
     *      This bit will be set when host is able to enable EAPOL offload to
     *      FW for SAE roaming feature.
     *      Refer to the below definitions of the
     *      WMI_RSRC_CFG_HOST_SERVICE_FLAG_SAE_EAPOL_OFFLOAD_SUPPORT_GET
     *      and _SET macros.
     *  Bit 4
     *      This bit will be set when host is able to process the
     *      WMI_REG_CC_EXT_EVENT.
     *      Refer to the below definitions of the
     *      WMI_RSRC_CFG_HOST_SERVICE_FLAG_REG_CC_EXT_SUPPORT_GET
     *      and _SET macros.
     *  Bit 5
     *      This bit will be set when the host supports NAN channels.
     *      Refer to WMI_RSRC_CFG_HOST_SERVICE_FLAG_NAN_CHANNEL_SUPPORT_GET/SET
     *  Bit 6
     *      This bit will be set when the host supports synchronous TWT events.
     *      Refer to WMI_RSRC_CFG_HOST_SERVICE_FLAG_STA_TWT_SYNC_EVT_SUPPORT_GET
     *      and _SET.
     *  Bit 7
     *      This bit will be set when host supports both LPI and SP mode.
     *      when this bit is set to 0 - Indicate LPI only mode
     *                  when set to 1 - Indicate both SP mode and LPI mode
     *                                  both are supported
     *      Refer to the below definitions of the
     *      WMI_RSRC_CFG_HOST_SERVICE_FLAG_LPI_SP_MODE_SUPPORT_GET
     *      and _SET macros.
     *  Bit 8
     *      This bit will be set when host wants to disable timer check in
     *      reg for AFC.
     *      when set to 1 - Disable timer check
     *      Refer to the below definitions of the
     *      WMI_RSRC_CFG_HOST_SERVICE_FLAG_REG_DISCARD_TIMER_CHECK_GET
     *      and _SET macros.
     *  Bit 9
     *      This bit will be set when host host wants to disable request id
     *      check in reg for AFC.
     *      when set to 1 - Disable Request ID check
     *      Refer to the below definitions of the
     *      WMI_RSRC_CFG_HOST_SERVICE_FLAG_REG_DISCARD_REQ_ID_CHECK_GET
     *      and _SET macros.
     *  Bit 10
     *      This bit will be set when host wants to enable indoor for AFC
     *      when this bit is set to 0 indoor mode not enabled
     *      when this bit is set to 1 indoor mode is enabled
     *      WMI_RSRC_CFG_HOST_SERVICE_FLAG_AFC_INDOOR_SUPPORT_CHECK_GET
     *      and SET macros
     *  Bit 11
     *      This bit will be set when host wants to enable outdoor for AFC
     *      when this bit is set to 0 outdoor mode not enabled
     *      when this bit is set to 1 outdoor mode is enabled
     *      WMI_RSRC_CFG_HOST_SERVICE_FLAG_AFC_OUTDOOR_SUPPORT_CHECK_GET
     *      and SET macros.
     *  Bit 12
     *      This bit will be set when host host wants to enable/disable
     *      REO QREF feature
     *      when set to 1 - Enable the REO QREF feature
     *      when set to 0 - Disable the REO QREF feature
     *      Refer to the below definitions of the
     *      WMI_RSRC_CFG_HOST_SERVICE_FLAG_REO_QREF_FEATURE_SUPPORT_GET
     *      and _SET macros.
     *  Bit 13
     *      This bit will be set when host host wants to enable/disable
     *      bang radar 320M support feature
     *      when set to 1 - Enable the bang radar 320M support
     *      when set to 0 - Disable the bang radar 320M support
     *      Refer to the below definitions of the
     *      WMI_RSRC_CFG_HOST_SERVICE_FLAG_BANG_RADAR_320M_SUPPORT_GET
     *      and _SET macros.
     *  Bit 14
     *      This bit will be set when host wants to enable/disable
     *      full BW NOL feature.
     *      When set to 1: Enable full BW NOL feature.
     *      When set to 0: Disable the full BW NOL feature.
     *      Refer to the below definitions of the
     *      WMI_RSRC_CFG_HOST_SERVICE_FLAG_RADAR_FLAGS_FULL_BW_NOL_GET
     *      and _SET macros.
     *  Bit 15
     *      This bit will be set if the host has qms_dlkm support enabled.
     *      Refer to the below definitions of the
     *      WMI_RSRC_CFG_HOST_SERVICE_FLAG_QMS_DLKM_SUPPORT_GET
     *      and _SET macros.
     *  Bits 31:16 - Reserved
     */
    A_UINT32 host_service_flags;

    /** @brief max_rnr_neighbours -
     * The Maximum number of neighbour RNR's from other SoC.
     * This limits the field @num_bss in @wmi_pdev_tbtt_offset_sync_cmd_fixed_param.
     * Value of 0 means cross SoC TBTT offset synchronization not required and
     * @PDEV_TBTT_OFFSET_SYNC_CMD wouldn't be used.
     */
    A_UINT32 max_rnr_neighbours;

    /** @brief ema_max_vap_cnt - number of maximum EMA Tx vaps (VAPs having both
     *  VDEV_FLAGS_EMA_MODE and VDEV_FLAGS_TRANSMIT_AP set) at any instance
     * of time across SOC. Legacy MBSS Vaps are not accounted in this field.
     */
    A_UINT32 ema_max_vap_cnt;

    /** @brief ema_max_profile_period - maximum profile periodicity
     * (maximum number of beacons after which VAP profiles repeat)
     * for any EMA VAP on any pdev.
     */

    A_UINT32 ema_max_profile_period;
    /** @brief max_ndp_sessions
     * This is the max ndp sessions sent by the host which is the minimum
     * of the value requested within the host's ini configurations and
     * the max ndp sessions supported by the firmware (as reported in the
     * SERVICE_READY_EXT2_EVENT message).
     */
    A_UINT32 max_ndp_sessions;

    /** @brief max_ndi_supported
     * This is the max ndi interfaces sent by the host based on the value
     * specified by the host's ini configuration.
     */
    A_UINT32 max_ndi_interfaces;

    /** @brief max_ap_vaps
     * Maximum number of AP mode vdevs created at any time.
     * This value is minimum of the number of AP vdevs supported by
     * the target and host.
     */
    A_UINT32 max_ap_vaps;

    /** @brief cbc_flow_ena
     * When cbc_flow_ena is se, halphy will do Cold Boot Calibration flow.
     * Otherwise, halphy will do normal flow.
     */
    A_UINT32 cbc_flow_ena;

    /** @brief ema_init_config - can be used by Host to indicate beacon size
     *  @details
     *  Bit   0 : 15 - Size of beacon, currently it can be sent as
     *                 1500 or 2500 or 4000 bytes
     *  Bit  16 : 31 - Reserved
     *  In case of Backward compatibility, if this variable is 0 then
     *  default beacon size (1500) is used.
     */
    A_UINT32 ema_init_config;

    /** @brief carrier_config
     * Carrier profile configuration
     * BIT 0 -> enable/disable charter configurations
     * BIT 1 : 31 Reserved
     */
    A_UINT32 carrier_config;

    /** @brief num_of_linkview_peers - proposed by the host value of
     *      the peers with the num_of_linkview_msduqs_per_tid allocation
     *  @details
     *  Host can request what the number of 'num_peers' should use
     *  num_of_linkview_msduqs_per_tid. All other peers will use
     *  default number of MSDUQs allocated.
     */
    A_UINT32 num_of_linkview_peers;

    /** @brief num_of_linkview_msduqs_per_tid - proposed by the host value of
     *      MSDUQs per each LinkView peer's TID
     *  @details
     *  Host sends the number of MSDUQs per each LinkView peer's TID.
     *  This number will be used during resources allocation for
     * LinkView peer in the target.
     */
    A_UINT32 num_of_linkview_msduqs_per_tid;

    /**
     * @brief num_max_active_vdevs -
     * number of max active virtual devices (VAPs) to support
     */
    A_UINT32 num_max_active_vdevs;

    /**
     * @brief num_max_mlo_link_per_ml_bss
     * number of max partner links of a ML BSS
     */
    A_UINT32 num_max_mlo_link_per_ml_bss;

    /**
     * @brief num_max_active_mlo_link_per_ml_bss
     * number of max active partner links of a ML BSS
     */
    A_UINT32 num_max_active_mlo_link_per_ml_bss;
} wmi_resource_config;

#define WMI_MSDU_FLOW_AST_ENABLE_GET(msdu_flow_config0, ast_x) \
    (((ast_x) == 0) ? 1 : ((msdu_flow_config0) & (1 << ((ast_x) - 1))))
#define WMI_MSDU_FLOW_AST_ENABLE_SET(msdu_flow_config0, ast_x, enable) \
    do { \
        if ((ast_x) == 0) break;  \
        if ((enable)) { \
            (msdu_flow_config0) |= (1 << ((ast_x) - 1)); \
        } else { \
            (msdu_flow_config0) &= ~(1 << ((ast_x) - 1)); \
        } \
    } while(0)

#define WMI_MSDU_FLOW_ASTX_MSDU_FLOW_MASKS_GET(msdu_flow_config1, ast_x) \
    (((msdu_flow_config1) & (0x0f << ((ast_x) * 4))) >> ((ast_x) * 4))
#define WMI_MSDU_FLOW_ASTX_MSDU_FLOW_MASKS_SET( \
    msdu_flow_config1, ast_x, mask) \
    do { \
        (msdu_flow_config1) &= ~(0xF << ((ast_x) * 4)); \
        (msdu_flow_config1) |= ((mask) << ((ast_x) * 4)); \
    } while(0)

#define WMI_MSDU_FLOW_TID_VALID_HI_MASKS_GET(msdu_flow_config1) \
    (((msdu_flow_config1) & 0xff0000) >> 16)
#define WMI_MSDU_FLOW_TID_VALID_HI_MASKS_SET(msdu_flow_config1, mask) \
    do { \
        (msdu_flow_config1) &= ~0xff0000; \
        (msdu_flow_config1) |= ((mask) << 16); \
    } while(0)

#define WMI_MSDU_FLOW_TID_VALID_LOW_MASKS_GET(msdu_flow_config1) \
    ((msdu_flow_config1 & 0xff000000) >> 24)
#define WMI_MSDU_FLOW_TID_VALID_LOW_MASKS_SET(msdu_flow_config1, mask) \
    do { \
        (msdu_flow_config1) &= ~0xff000000; \
        (msdu_flow_config1) |= ((mask) << 24); \
    } while(0)

#define WMI_RSRC_CFG_FLAG_SET(word32, flag, value) \
    do { \
        (word32) &= ~WMI_RSRC_CFG_FLAG_ ## flag ## _M; \
        (word32) |= ((value) << WMI_RSRC_CFG_FLAG_ ## flag ## _S) & \
            WMI_RSRC_CFG_FLAG_ ## flag ## _M; \
    } while (0)
#define WMI_RSRC_CFG_FLAG_GET(word32, flag) \
    (((word32) & WMI_RSRC_CFG_FLAG_ ## flag ## _M) >> \
    WMI_RSRC_CFG_FLAG_ ## flag ## _S)

#define WMI_RSRC_CFG_FLAG_WOW_IGN_PCIE_RST_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), WOW_IGN_PCIE_RST, (value))
#define WMI_RSRC_CFG_FLAG_WOW_IGN_PCIE_RST_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), WOW_IGN_PCIE_RST)

#define WMI_RSRC_CFG_FLAG_LTEU_SUPPORT_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), LTEU_SUPPORT, (value))
#define WMI_RSRC_CFG_FLAG_LTEU_SUPPORT_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), LTEU_SUPPORT)

#define WMI_RSRC_CFG_FLAG_COEX_GPIO_SUPPORT_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), COEX_GPIO_SUPPORT, (value))
#define WMI_RSRC_CFG_FLAG_COEX_GPIO_SUPPORT_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), COEX_GPIO_SUPPORT)

#define WMI_RSRC_CFG_FLAG_AUX_RADIO_SPECTRAL_INTF_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), AUX_RADIO_SPECTRAL_INTF, (value))
#define WMI_RSRC_CFG_FLAG_AUX_RADIO_SPECTRAL_INTF_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), AUX_RADIO_SPECTRAL_INTF)

#define WMI_RSRC_CFG_FLAG_AUX_RADIO_CHAN_LOAD_INTF_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), AUX_RADIO_CHAN_LOAD_INTF, (value))
#define WMI_RSRC_CFG_FLAG_AUX_RADIO_CHAN_LOAD_INTF_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), AUX_RADIO_CHAN_LOAD_INTF)

#define WMI_RSRC_CFG_FLAG_BSS_CHANNEL_INFO_64_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), BSS_CHANNEL_INFO_64, (value))
#define WMI_RSRC_CFG_FLAG_BSS_CHANNEL_INFO_64_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), BSS_CHANNEL_INFO_64)

#define WMI_RSRC_CFG_FLAG_ATF_CONFIG_ENABLE_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), ATF_CONFIG_ENABLE, (value))
#define WMI_RSRC_CFG_FLAG_ATF_CONFIG_ENABLE_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), ATF_CONFIG_ENABLE)

#define WMI_RSRC_CFG_FLAG_IPHR_PAD_CONFIG_ENABLE_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), IPHR_PAD_CONFIG_ENABLE, (value))
#define WMI_RSRC_CFG_FLAG_IPHR_PAD_CONFIG_ENABLE_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), IPHR_PAD_CONFIG_ENABLE)

#define WMI_RSRC_CFG_FLAG_QWRAP_MODE_ENABLE_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), QWRAP_MODE_ENABLE, (value))
#define WMI_RSRC_CFG_FLAG_QWRAP_MODE_ENABLE_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), QWRAP_MODE_ENABLE)

#define WMI_RSRC_CFG_FLAG_MGMT_COMP_EVT_BUNDLE_SUPPORT_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), MGMT_COMP_EVT_BUNDLE_SUPPORT, (value))
#define WMI_RSRC_CFG_FLAG_MGMT_COMP_EVT_BUNDLE_SUPPORT_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), MGMT_COMP_EVT_BUNDLE_SUPPORT)

#define WMI_RSRC_CFG_FLAG_TX_MSDU_ID_NEW_PARTITION_SUPPORT_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), TX_MSDU_ID_NEW_PARTITION_SUPPORT, (value))
#define WMI_RSRC_CFG_FLAG_TX_MSDU_ID_NEW_PARTITION_SUPPORT_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), TX_MSDU_ID_NEW_PARTITION_SUPPORT)

#define WMI_RSRC_CFG_FLAG_TCL_CCE_DISABLE_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), TCL_CCE_DISABLE, (value))
#define WMI_RSRC_CFG_FLAG_TCL_CCE_DISABLE_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), TCL_CCE_DISABLE)

#define WMI_RSRC_CFG_FLAG_TIM_V2_SUPPORT_ENABLE_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), TIM_V2_SUPPORT_ENABLE, (value))
#define WMI_RSRC_CFG_FLAG_TIM_V2_SUPPORT_ENABLE_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), TIM_V2_SUPPORT_ENABLE)

#define WMI_RSRC_CFG_FLAG_EAPOL_REKEY_MINRATE_SUPPORT_ENABLE_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), EAPOL_REKEY_MINRATE_SUPPORT_ENABLE, (value))
#define WMI_RSRC_CFG_FLAG_EAPOL_REKEY_MINRATE_SUPPORT_ENABLE_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), EAPOL_REKEY_MINRATE_SUPPORT_ENABLE)

#define WMI_RSRC_CFG_FLAG_EAPOL_AC_OVERRIDE_VALID_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), EAPOL_AC_OVERRIDE_VALID, (value))
#define WMI_RSRC_CFG_FLAG_EAPOL_AC_OVERRIDE_VALID_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), EAPOL_AC_OVERRIDE_VALID)

#define WMI_RSRC_CFG_FLAG_EAPOL_AC_OVERRIDE_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), EAPOL_AC_OVERRIDE, (value))
#define WMI_RSRC_CFG_FLAG_EAPOL_AC_OVERRIDE_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), EAPOL_AC_OVERRIDE)

#define WMI_RSRC_CFG_FLAG_TX_ACK_RSSI_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), TX_ACK_RSSI, (value))
#define WMI_RSRC_CFG_FLAG_TX_ACK_RSSI_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), TX_ACK_RSSI)

#define WMI_RSRC_CFG_FLAG_HTT_H2T_NO_HTC_HDR_LEN_IN_MSG_LEN_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), HTT_H2T_NO_HTC_HDR_LEN_IN_MSG_LEN, (value))
#define WMI_RSRC_CFG_FLAG_HTT_H2T_NO_HTC_HDR_LEN_IN_MSG_LEN_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), HTT_H2T_NO_HTC_HDR_LEN_IN_MSG_LEN)

#define WMI_RSRC_CFG_FLAG_PEER_UNMAP_RESPONSE_SUPPORT_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), PEER_UNMAP_RESPONSE_SUPPORT, (value))
#define WMI_RSRC_CFG_FLAG_PEER_UNMAP_RESPONSE_SUPPORT_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), PEER_UNMAP_RESPONSE_SUPPORT)

#define WMI_RSRC_CFG_FLAG_HTT_PEER_STATS_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), HTT_PEER_STATS, (value))
#define WMI_RSRC_CFG_FLAG_HTT_PEER_STATS_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), HTT_PEER_STATS)

#define WMI_RSRC_CFG_FLAG_PEER_TID_EXT_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), PEER_TID_EXT, (value))
#define WMI_RSRC_CFG_FLAG_PEER_TID_EXT_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), PEER_TID_EXT)

#define WMI_RSRC_CFG_FLAG_VIDEO_OVER_WIFI_ENABLE_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), VIDEO_OVER_WIFI_ENABLE, (value))
#define WMI_RSRC_CFG_FLAG_VIDEO_OVER_WIFI_ENABLE_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), VIDEO_OVER_WIFI_ENABLE)

#define WMI_RSRC_CFG_FLAG_THREE_WAY_COEX_CONFIG_LEGACY_SUPPORT_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), THREE_WAY_COEX_CONFIG_LEGACY_SUPPORT, (value))
#define WMI_RSRC_CFG_FLAG_THREE_WAY_COEX_CONFIG_LEGACY_SUPPORT_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), THREE_WAY_COEX_CONFIG_LEGACY_SUPPORT)

#define WMI_RSRC_CFG_FLAG_THREE_WAY_COEX_CONFIG_OVERRIDE_SUPPORT_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), THREE_WAY_COEX_CONFIG_OVERRIDE_SUPPORT, (value))
#define WMI_RSRC_CFG_FLAG_THREE_WAY_COEX_CONFIG_OVERRIDE_SUPPORT_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), THREE_WAY_COEX_CONFIG_OVERRIDE_SUPPORT)

#define WMI_RSRC_CFG_FLAG_TX_COMPLETION_TX_TSF64_ENABLE_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), TX_COMPLETION_TX_TSF64_ENABLE, (value))
#define WMI_RSRC_CFG_FLAG_TX_COMPLETION_TX_TSF64_ENABLE_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), TX_COMPLETION_TX_TSF64_ENABLE)

#define WMI_RSRC_CFG_FLAG_PACKET_CAPTURE_SUPPORT_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), PACKET_CAPTURE_SUPPORT, (value))
#define WMI_RSRC_CFG_FLAG_PACKET_CAPTURE_SUPPORT_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), PACKET_CAPTURE_SUPPORT)

#define WMI_RSRC_CFG_FLAG_BSS_MAX_IDLE_TIME_SUPPORT_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), BSS_MAX_IDLE_TIME_SUPPORT, (value))
#define WMI_RSRC_CFG_FLAG_BSS_MAX_IDLE_TIME_SUPPORT_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), BSS_MAX_IDLE_TIME_SUPPORT)

#define WMI_RSRC_CFG_FLAG_AUDIO_SYNC_SUPPORT_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), AUDIO_SYNC_SUPPORT, (value))
#define WMI_RSRC_CFG_FLAG_AUDIO_SYNC_SUPPORT_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), AUDIO_SYNC_SUPPORT)

#define WMI_RSRC_CFG_FLAG_IPA_DISABLE_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), IPA_DISABLE, (value))
#define WMI_RSRC_CFG_FLAG_IPA_DISABLE_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), IPA_DISABLE)

#define WMI_RSRC_CFG_FLAG_PCIE_GEN_SWITCH_CAPABLITY_SET(word32, value) \
    WMI_RSRC_CFG_FLAG_SET((word32), PCIE_GEN_SWITCH_CAPABLITY, (value))
#define WMI_RSRC_CFG_FLAG_PCIE_GEN_SWITCH_CAPABLITY_GET(word32) \
    WMI_RSRC_CFG_FLAG_GET((word32), PCIE_GEN_SWITCH_CAPABLITY)

#define WMI_RSRC_CFG_FLAGS2_RE_ULRESP_PDEV_CFG_GET(flags2, pdev_id) \
    WMI_GET_BITS(flags2, pdev_id, 1)
#define WMI_RSRC_CFG_FLAGS2_RE_ULRESP_PDEV_CFG_SET(flags2, pdev_id, value) \
    WMI_SET_BITS(flags2, pdev_id, 1, value)

#define WMI_RSRC_CFG_FLAGS2_RX_PEER_METADATA_VERSION_GET(flags2) \
    WMI_GET_BITS(flags2, 4, 2)
#define WMI_RSRC_CFG_FLAGS2_RX_PEER_METADATA_VERSION_SET(flags2, value) \
    WMI_SET_BITS(flags2, 4, 2, value)

#define WMI_RSRC_CFG_FLAGS2_IS_SAP_CONNECTED_D3WOW_ENABLED_GET(flags2) \
    WMI_GET_BITS(flags2, 6, 1)
#define WMI_RSRC_CFG_FLAGS2_IS_SAP_CONNECTED_D3WOW_ENABLED_SET(flags2, value) \
    WMI_SET_BITS(flags2, 6, 1, value)

#define WMI_RSRC_CFG_FLAGS2_IS_GO_CONNECTED_D3WOW_ENABLED_GET(flags2) \
    WMI_GET_BITS(flags2, 7, 1)
#define WMI_RSRC_CFG_FLAGS2_IS_GO_CONNECTED_D3WOW_ENABLED_SET(flags2, value) \
    WMI_SET_BITS(flags2, 7, 1, value)

#define WMI_RSRC_CFG_FLAGS2_IS_DYNAMIC_PCIE_GEN_SPEED_SWITCH_ENABLED_GET(flags2) \
    WMI_GET_BITS(flags2, 8, 1)
#define WMI_RSRC_CFG_FLAGS2_IS_DYNAMIC_PCIE_GEN_SPEED_SWITCH_ENABLED_SET(flags2, value) \
    WMI_SET_BITS(flags2, 8, 1, value)

#define WMI_RSRC_CFG_FLAGS2_CALC_NEXT_DTIM_COUNT_GET(flags2) \
    WMI_GET_BITS(flags2, 9, 1)
#define WMI_RSRC_CFG_FLAGS2_CALC_NEXT_DTIM_COUNT_SET(flags2, value) \
    WMI_SET_BITS(flags2, 9, 1, value)

#define WMI_RSRC_CFG_FLAGS2_ARP_AC_OVERRIDE_VALID_GET(flags2) \
    WMI_GET_BITS(flags2, 10, 1)
#define WMI_RSRC_CFG_FLAGS2_ARP_AC_OVERRIDE_VALID_SET(flags2, value) \
    WMI_SET_BITS(flags2, 10, 1, value)

#define WMI_RSRC_CFG_FLAGS2_ARP_AC_OVERRIDE_GET(flags2) \
    WMI_GET_BITS(flags2, 11, 2)
#define WMI_RSRC_CFG_FLAGS2_ARP_AC_OVERRIDE_SET(flags2, value) \
    WMI_SET_BITS(flags2, 11, 2, value)

#define WMI_RSRC_CFG_FLAGS2_SAWF_CONFIG_ENABLE_GET(flags2) \
    WMI_GET_BITS(flags2, 13, 1)
#define WMI_RSRC_CFG_FLAGS2_SAWF_CONFIG_ENABLE_SET(flags2, value) \
    WMI_SET_BITS(flags2, 13, 1, value)

#define WMI_RSRC_CFG_FLAGS2_NOTIFY_FRAME_CONFIG_ENABLE_GET(flags2) \
    WMI_GET_BITS(flags2, 14, 1)
#define WMI_RSRC_CFG_FLAGS2_NOTIFY_FRAME_CONFIG_ENABLE_SET(flags2, value) \
    WMI_SET_BITS(flags2, 14, 1, value)

#define WMI_RSRC_CFG_FLAGS2_DISABLE_WDS_MEC_INTRABSS_OFFLOAD_GET(flags2) \
    WMI_GET_BITS(flags2, 15, 1)
#define WMI_RSRC_CFG_FLAGS2_DISABLE_WDS_MEC_INTRABSS_OFFLOAD_SET(flags2, value) \
    WMI_SET_BITS(flags2, 15, 1, value)

#define WMI_RSRC_CFG_FLAGS2_LATENCY_FLOWQ_SUPPORT_GET(flags2) \
    WMI_GET_BITS(flags2, 16, 1)
#define WMI_RSRC_CFG_FLAGS2_LATENCY_FLOWQ_SUPPORT_SET(flags2, value) \
    WMI_SET_BITS(flags2, 16, 1, value)

#define WMI_RSRC_CFG_FLAGS2_RF_PATH_MODE_GET(flags2) \
    WMI_GET_BITS(flags2, 17, 1)
#define WMI_RSRC_CFG_FLAGS2_RF_PATH_MODE_SET(flags2, value) \
    WMI_SET_BITS(flags2, 17, 1, value)

#define WMI_RSRC_CFG_FLAGS2_DISABLE_WDS_PEER_MAP_UNMAP_EVENT_GET(flags2) \
    WMI_GET_BITS(flags2, 18, 1)
#define WMI_RSRC_CFG_FLAGS2_DISABLE_WDS_PEER_MAP_UNMAP_EVENT_SET(flags2, value) \
    WMI_SET_BITS(flags2, 18, 1, value)


#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_NAN_IFACE_SUPPORT_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 0, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_NAN_IFACE_SUPPORT_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 0, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_HOST_SUPPORT_MULTI_RADIO_EVTS_PER_RADIO_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 1, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_HOST_SUPPORT_MULTI_RADIO_EVTS_PER_RADIO_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 1, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_SPLIT_AST_FEATURE_HOST_SUPPORT_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 2, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_SPLIT_AST_FEATURE_HOST_SUPPORT_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 2, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_SAE_EAPOL_OFFLOAD_SUPPORT_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 3, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_SAE_EAPOL_OFFLOAD_SUPPORT_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 3, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_REG_CC_EXT_SUPPORT_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 4, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_REG_CC_EXT_SUPPORT_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 4, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_NAN_CHANNEL_SUPPORT_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 5, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_NAN_CHANNEL_SUPPORT_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 5, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_STA_TWT_SYNC_EVT_SUPPORT_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 6, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_STA_TWT_SYNC_EVT_SUPPORT_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 6, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_LPI_SP_MODE_SUPPORT_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 7, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_LPI_SP_MODE_SUPPORT_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 7, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_REG_DISCARD_AFC_TIMER_CHECK_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 8, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_REG_DISCARD_AFC_TIMER_CHECK_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 8, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_REG_DISCARD_AFC_REQ_ID_CHECK_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 9, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_REG_DISCARD_AFC_REQ_ID_CHECK_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 9, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_AFC_INDOOR_SUPPORT_CHECK_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 10, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_AFC_INDOOR_SUPPORT_CHECK_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 10, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_AFC_OUTDOOR_SUPPORT_CHECK_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 11, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_AFC_OUTDOOR_SUPPORT_CHECK_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 11, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_REO_QREF_FEATURE_SUPPORT_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 12, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_REO_QREF_FEATURE_SUPPORT_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 12, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_BANG_RADAR_320M_SUPPORT_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 13, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_BANG_RADAR_320M_SUPPORT_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 13, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_RADAR_FLAGS_FULL_BW_NOL_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 14, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_RADAR_FLAGS_FULL_BW_NOL_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 14, 1, val)

#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_QMS_DLKM_SUPPORT_GET(host_service_flags) \
    WMI_GET_BITS(host_service_flags, 15, 1)
#define WMI_RSRC_CFG_HOST_SERVICE_FLAG_QMS_DLKM_SUPPORT_SET(host_service_flags, val) \
    WMI_SET_BITS(host_service_flags, 15, 1, val)


#define WMI_RSRC_CFG_CARRIER_CFG_CHARTER_ENABLE_GET(carrier_config) \
    WMI_GET_BITS(carrier_config, 0, 1)
#define WMI_RSRC_CFG_CARRIER_CFG_CHARTER_ENABLE_SET(carrier_config, val) \
    WMI_SET_BITS(carrier_config, 0, 1, val)

/** Top nibble can be used to diff between HE and EHT: 0xVXXXXXXX
 *  If V == 0b0000: format is HE.
 *  If V == 0b0001: format is EHT.
 */
#define WMI_RSRC_CFG_IS_EHT_GET(param_value) \
    WMI_GET_BITS(param_value, 28, 4)
#define WMI_RSRC_CFG_IS_EHT_SET(param_value, val) \
    WMI_SET_BITS(param_value, 28, 4, val)

/* Used along with the above macro to set the value. */
#define WMI_RSRC_CFG_PARAM_VALUE_GET(param_value) \
    WMI_GET_BITS(param_value, 0, 28)
#define WMI_RSRC_CFG_PARAM_VALUE_SET(param_value, val) \
    WMI_SET_BITS(param_value, 0, 28, val)

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_init_cmd_fixed_param */

    /** The following indicate the WMI versions to be supported by
     *  the host driver. Note that the host driver decide to
     *  "downgrade" its WMI version support and this may not be the
     *  native version of the host driver. */
    wmi_abi_version host_abi_vers;

    A_UINT32 num_host_mem_chunks; /** size of array host_mem_chunks[] */
/* This init_cmd_fixed_param TLV is followed by the below TLVs:
 *     wmi_resource_config   resource_config;
 *     wlan_host_memory_chunk host_mem_chunks[];
 *     wmi_pdev_set_hw_mode_cmd_fixed_param hw_mode_config;
 *         Note that the hw_mode_config, in spite of its "pdev" name,
 *         applies to the entire target rather than for a single pdev
 *         within the target.
 *         To avoid specifying a HW mode for the target, the host should
 *         fill hw_mode_config's fields with 0x0.
 *     wmi_pdev_band_to_mac                 band_to_mac[];
 *     wmi_htt_msdu_idx_to_htt_msdu_qtype   htt_msdu_idx_to_qtype_map[];
 */
} wmi_init_cmd_fixed_param;

typedef enum {
    WMI_WIFI_STANDARD_4     = 0,
    WMI_WIFI_STANDARD_5     = 1,
    WMI_WIFI_STANDARD_6     = 2,
    WMI_WIFI_STANDARD_6E    = 3,
    WMI_WIFI_STANDARD_7     = 4,
} WMI_WIFI_STANDARD;

typedef enum {
    WMI_HOST_NONE       = 0, /* No concurrency mode supported */
    WMI_HOST_DBS        = 1, /* When 2.4G + 5G & 2.4G + 6G if 6G is supported */
    WMI_HOST_DBS_SBS    = 2, /* When 2.4G + 5G, 2.4G + 6G, 5G + 6G & 5G + 5G is supported */
} WMI_BAND_CONCURRENCY;

typedef enum {
    WMI_SISO        = 1, /* When 1x1 is supported */
    WMI_MIMO_2X2    = 2, /* When 2x2 MIMO is supported */
} WMI_NUM_ANTENNAS;

typedef enum {
    WMI_VENDOR1_REQ1_VERSION_3_00   = 0,
    WMI_VENDOR1_REQ1_VERSION_3_01   = 1,
    WMI_VENDOR1_REQ1_VERSION_3_20   = 2,
    WMI_VENDOR1_REQ1_VERSION_3_30   = 3,
    WMI_VENDOR1_REQ1_VERSION_3_40   = 4,
    WMI_VENDOR1_REQ1_VERSION_4_00   = 5,
} WMI_VENDOR1_REQ1_VERSION;

typedef enum {
    WMI_VENDOR1_REQ2_VERSION_3_00   = 0,
    WMI_VENDOR1_REQ2_VERSION_3_01   = 1,
    WMI_VENDOR1_REQ2_VERSION_3_20   = 2,
    WMI_VENDOR1_REQ2_VERSION_3_50   = 3,
} WMI_VENDOR1_REQ2_VERSION;

typedef enum {
    WMI_HOST_BAND_CAP_2GHZ = 0x01,
    WMI_HOST_BAND_CAP_5GHZ = 0x02,
    WMI_HOST_BAND_CAP_6GHZ = 0x04,
} WMI_HOST_BAND_CAP;

/* HW features supported info */
/* enum WMI_WIFI_STANDARD are possible values for WiFi standard bitfield */
#define WMI_GET_WIFI_STANDARD(var, feature_bitmap)              \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 0, 4)
#define WMI_SET_WIFI_STANDARD(feature_bitmap, val)              \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 0, 4, val)
/* enum WMI_BAND_CONCURRENCY are possible values for band concurrency support bitfield */
#define WMI_GET_BAND_CONCURRENCY_SUPPORT(var, feature_bitmap)   \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 4, 3)
#define WMI_SET_BAND_CONCURRENCY_SUPPORT(feature_bitmap, val)   \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 4, 3, val)

/* PNO feature supported info */
#define WMI_GET_PNO_SCAN_IN_UNASSOC_STATE(var, feature_bitmap)          \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 7, 1)
#define WMI_SET_PNO_SCAN_IN_UNASSOC_STATE(feature_bitmap, val)          \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 7, 1, val)
#define WMI_GET_PNO_SCAN_IN_ASSOC_STATE(var, feature_bitmap)            \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 8, 1)
#define WMI_SET_PNO_SCAN_IN_ASSOC_STATE(feature_bitmap, val)            \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 8, 1, val)

/* TWT feature supported info */
#define WMI_GET_TWT_FEATURE_SUPPORT(var, feature_bitmap)           \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 9, 1)
#define WMI_SET_TWT_FEATURE_SUPPORT(feature_bitmap, val)           \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 9, 1, val)
#define WMI_GET_TWT_REQUESTOR(var, feature_bitmap)                 \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 10, 1)
#define WMI_SET_TWT_REQUESTER(feature_bitmap, val)                 \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 10, 1, val)
#define WMI_GET_TWT_BROADCAST(var, feature_bitmap)                 \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 11, 1)
#define WMI_SET_TWT_BROADCAST(feature_bitmap, val)                 \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 11, 1, val)
#define WMI_GET_TWT_FLEXIBLE(var, feature_bitmap)                  \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 12, 1)
#define WMI_SET_TWT_FLEXIBLE(feature_bitmap, val)                  \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 12, 1, val)

/* WIFI optimizer feature supported info */
#define WMI_GET_WIFI_OPT_FEATURE_SUPPORT(var, feature_bitmap)                   \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 13, 1)
#define WMI_SET_WIFI_OPT_FEATURE_SUPPORT(feature_bitmap, val)         \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 13, 1, val)

/* RFC8325 feature supported info */
#define WMI_GET_RFC8325_FEATURE_SUPPORT(var, feature_bitmap)                \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 14, 1)
#define WMI_SET_RFC8325_FEATURE_SUPPORT(feature_bitmap, val)                \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 14, 1, val)

/* MHS feature supported info */
#define WMI_GET_MHS_5G_SUPPORT(var, feature_bitmap)                     \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 15, 1)
#define WMI_SET_MHS_5G_SUPPORT(feature_bitmap, val)                     \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 15, 1, val)
#define WMI_GET_MHS_6G_SUPPORT(var, feature_bitmap)                     \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 16, 1)
#define WMI_SET_MHS_6G_SUPPORT(feature_bitmap, val)                     \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 16, 1, val)
#define WMI_GET_MHS_MAX_CLIENTS_SUPPORT(var, feature_bitmap)            \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 17, 8)
#define WMI_SET_MHS_MAX_CLIENTS_SUPPORT(feature_bitmap, val)            \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 17, 8, val)
#define WMI_GET_MHS_SET_COUNTRY_CODE_HAL_SUPPORT(var, feature_bitmap)   \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 25, 1)
#define WMI_SET_MHS_SET_COUNTRY_CODE_HAL_SUPPORT(feature_bitmap, val)   \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 25, 1, val)
#define WMI_GET_MHS_GETVALID_CHANNELS_SUPPORT(var, feature_bitmap)      \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 26, 1)
#define WMI_SET_MHS_GETVALID_CHANNELS_SUPPORT(feature_bitmap, val)      \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 26, 1, val)
/* enum WMI_WIFI_STANDARD are possible values for MHS DOT11 mode support bitfield */
#define WMI_GET_MHS_DOT11_MODE_SUPPORT(var, feature_bitmap)             \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 27, 4)
#define WMI_SET_MHS_DOT11_MODE_SUPPORT(feature_bitmap, val)             \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 27, 4, val)
#define WMI_GET_MHS_WPA3_SUPPORT(var, feature_bitmap)                   \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 31, 1)
#define WMI_SET_MHS_WPA3_SUPPORT(feature_bitmap, val)                   \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 31, 1, val)

/* Vendor requirement1 supported version info */
/* enum's WMI_VENDORxx_REQxx_VERSION are the possible vaues for below bitfield*/
#define WMI_GET_VENDOR_REQ_1_VERSION(var, feature_bitmap)          \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 32, 8)
#define WMI_SET_VENDOR_REQ_1_VERSION(feature_bitmap, val)          \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 32, 8, val)

/* Roaming feature supported info */
#define WMI_GET_ROAMING_HIGH_CU_ROAM_TRIGGER(var, feature_bitmap)          \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 40, 1)
#define WMI_SET_ROAMING_HIGH_CU_ROAM_TRIGGER(feature_bitmap, val)          \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 40, 1, val)
#define WMI_GET_ROAMING_EMERGENCY_TRIGGER(var, feature_bitmap)             \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 41, 1)
#define WMI_SET_ROAMING_EMERGENCY_TRIGGER(feature_bitmap, val)             \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 41, 1, val)
#define WMI_GET_ROAMING_BTM_TRIGGER(var, feature_bitmap)                   \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 42, 1)
#define WMI_SET_ROAMING_BTM_TRIGGER(feature_bitmap, val)                   \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 42, 1, val)
#define WMI_GET_ROAMING_IDLE_TRIGGER(var, feature_bitmap)                  \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 43, 1)
#define WMI_SET_ROAMING_IDLE_TRIGGER(feature_bitmap, val)                  \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 43, 1, val)
#define WMI_GET_ROAMING_WTC_TRIGGER(var, feature_bitmap)                   \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 44, 1)
#define WMI_SET_ROAMING_WTC_TRIGGER(feature_bitmap, val)                   \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 44, 1, val)
#define WMI_GET_ROAMING_BTCOEX_TRIGGER(var, feature_bitmap)                \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 45, 1)
#define WMI_SET_ROAMING_BTCOEX_TRIGGER(feature_bitmap, val)                \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 45, 1, val)
#define WMI_GET_ROAMING_BTW_WPA_WPA2(var, feature_bitmap)                  \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 46, 1)
#define WMI_SET_ROAMING_BTW_WPA_WPA2(feature_bitmap, val)                  \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 46, 1, val)
#define WMI_GET_ROAMING_MANAGE_CHAN_LIST_API(var, feature_bitmap)          \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 47, 1)
#define WMI_SET_ROAMING_MANAGE_CHAN_LIST_API(feature_bitmap, val)          \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 47, 1, val)
#define WMI_GET_ROAMING_ADAPTIVE_11R(var, feature_bitmap)                  \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 48, 1)
#define WMI_SET_ROAMING_ADAPTIVE_11R(feature_bitmap, val)                  \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 48, 1, val)
#define WMI_GET_ROAMING_CTRL_API_GET_SET(var, feature_bitmap)              \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 49, 1)
#define WMI_SET_ROAMING_CTRL_API_GET_SET(feature_bitmap, val)              \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 49, 1, val)
#define WMI_GET_ROAMING_CTRL_API_REASSOC(var, feature_bitmap)              \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 50, 1)
#define WMI_SET_ROAMING_CTRL_API_REASSOC(feature_bitmap, val)              \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 50, 1, val)
#define WMI_GET_ROAMING_CTRL_GET_CU(var, feature_bitmap)                   \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 51, 1)
#define WMI_SET_ROAMING_CTRL_GET_CU(feature_bitmap, val)                   \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 51, 1, val)

/* Vendor requirement2 supported version info */
/* enum's WMI_VENDORxx_REQxx_VERSION are the possible vaues for below bitfield*/
#define WMI_GET_VENDOR_REQ_2_VERSION(var, feature_bitmap)          \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 52, 8)
#define WMI_SET_VENDOR_REQ_2_VERSION(feature_bitmap, val)          \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 52, 8, val)

#define WMI_GET_ASSURANCE_DISCONNECT_REASON_API(var, feature_bitmap)       \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 60, 1)
#define WMI_SET_ASSURANCE_DISCONNECT_REASON_API(feature_bitmap, val)       \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 60, 1, val)

/* Frame pcap logging */
#define WMI_GET_FRAME_PCAP_LOG_MGMT(var, feature_bitmap)          \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 61, 1)
#define WMI_SET_FRAME_PCAP_LOG_MGMT(feature_bitmap, val)          \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 61, 1, val)
#define WMI_GET_FRAME_PCAP_LOG_CTRL(var, feature_bitmap)          \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 62, 1)
#define WMI_SET_FRAME_PCAP_LOG_CTRL(feature_bitmap, val)          \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 62, 1, val)
#define WMI_GET_FRAME_PCAP_LOG_DATA(var, feature_bitmap)          \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 63, 1)
#define WMI_SET_FRAME_PCAP_LOG_DATA(feature_bitmap, val)          \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 63, 1, val)

/* Security features supported info */
#define WMI_GET_SECURITY_WPA3_SAE_H2E(var, feature_bitmap)                \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 64, 1)
#define WMI_SET_SECURITY_WPA3_SAE_H2E(feature_bitmap, val)                \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 64, 1, val)
#define WMI_GET_SECURITY_WPA3_SAE_FT(var, feature_bitmap)                 \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 65, 1)
#define WMI_SET_SECURITY_WPA3_SAE_FT(feature_bitmap, val)                 \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 65, 1, val)
#define WMI_GET_SECURITY_WPA3_ENTERP_SUITEB(var, feature_bitmap)          \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 66, 1)
#define WMI_SET_SECURITY_WPA3_ENTERP_SUITEB(feature_bitmap, val)          \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 66, 1, val)
#define WMI_GET_SECURITY_WPA3_ENTERP_SUITEB_192bit(var, feature_bitmap)   \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 67, 1)
#define WMI_SET_SECURITY_WPA3_ENTERP_SUITEB_192bit(feature_bitmap, val)   \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 67, 1, val)
#define WMI_GET_SECURITY_FILS_SHA256(var, feature_bitmap)                 \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 68, 1)
#define WMI_SET_SECURITY_FILS_SHA256(feature_bitmap, val)                 \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 68, 1, val)
#define WMI_GET_SECURITY_FILS_SHA384(var, feature_bitmap)                 \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 69, 1)
#define WMI_SET_SECURITY_FILS_SHA384(feature_bitmap, val)                 \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 69, 1, val)
#define WMI_GET_SECURITY_FILS_SHA256_FT(var, feature_bitmap)              \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 70, 1)
#define WMI_SET_SECURITY_FILS_SHA256_FT(feature_bitmap, val)              \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 70, 1, val)
#define WMI_GET_SECURITY_FILS_SHA384_FT(var, feature_bitmap)              \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 71, 1)
#define WMI_SET_SECURITY_FILS_SHA384_FT(feature_bitmap, val)              \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 71, 1, val)
#define WMI_GET_SECURITY_ENCHANCED_OPEN(var, feature_bitmap)              \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 72, 1)
#define WMI_SET_SECURITY_ENCHANCED_OPEN(feature_bitmap, val)              \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 72, 1, val)

/* Peer protocol features supported info */
#define WMI_GET_NAN_SUPPORT(var, feature_bitmap)            \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 73, 1)
#define WMI_SET_NAN_SUPPORT(feature_bitmap, val)            \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 73, 1, val)
#define WMI_GET_TDLS_SUPPORT(var, feature_bitmap)           \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 74, 1)
#define WMI_SET_TDLS_SUPPORT(feature_bitmap, val)           \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 74, 1, val)
#define WMI_GET_P2P6E_SUPPORT(var, feature_bitmap)          \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 75, 1)
#define WMI_SET_P2P6E_SUPPORT(feature_bitmap, val)          \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 75, 1, val)
#define WMI_GET_TDLS_OFFCHAN_SUPPORT(var, feature_bitmap)   \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 76, 1)
#define WMI_SET_TDLS_OFFCHAN_SUPPORT(feature_bitmap, val)   \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 76, 1, val)
#define WMI_GET_TDLS_CAP_ENHANCE(var, feature_bitmap)       \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 77, 1)
#define WMI_SET_TDLS_CAP_ENHANCE(feature_bitmap, val)       \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 77, 1, val)
#define WMI_GET_MAX_TDLS_PEERS_SUPPORT(var, feature_bitmap) \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 78, 4)
#define WMI_SET_MAX_TDLS_PEERS_SUPPORT(feature_bitmap, val) \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 78, 4, val)
#define WMI_GET_STA_DUAL_P2P_SUPPORT(var, feature_bitmap)   \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 82, 1)
#define WMI_SET_STA_DUAL_P2P_SUPPORT(feature_bitmap, val)   \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 82, 1, val)

/* Big data feature supported info */
#define WMI_GET_PEER_BIGDATA_GETBSSINFO_API_SUPPORT(var, feature_bitmap)            \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 83, 1)
#define WMI_SET_PEER_BIGDATA_GETBSSINFO_API_SUPPORT(feature_bitmap, val)            \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 83, 1, val)
#define WMI_GET_PEER_BIGDATA_GETASSOCREJECTINFO_API_SUPPORT(var, feature_bitmap)    \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 84, 1)
#define WMI_SET_PEER_BIGDATA_GETASSOCREJECTINFO_API_SUPPORT(feature_bitmap, val)    \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 84, 1, val)
#define WMI_GET_PEER_BIGDATA_GETSTAINFO_API_SUPPORT(var, feature_bitmap)            \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 85, 1)
#define WMI_SET_PEER_BIGDATA_GETSTAINFO_API_SUPPORT(feature_bitmap, val)            \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 85, 1, val)

/* Feature set requirement supported version info */
#define WMI_GET_FEATURE_SET_VERSION(var, feature_bitmap)                 \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 86, 16)
#define WMI_SET_FEATURE_SET_VERSION(feature_bitmap, val)                 \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 86, 16, val)

/*
 * enum WMI_NUM_ANTENNAS are possible values for number of antennas supported bitfield.
 * Bitfield value 0 means invalid, 1 means SISO, 2 means MIMO, and values 3+ are reserved.
 */
#define WMI_GET_NUM_ANTENNAS(var, feature_bitmap)   \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 102, 4)
#define WMI_SET_NUM_ANTENNAS(feature_bitmap, val)   \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 102, 4, val)

/* enum WMI_HOST_BAND_CAP are possible values for below bitfield */
#define WMI_GET_HOST_BAND_CAP(var, feature_bitmap)   \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 106, 6)
#define WMI_SET_HOST_BAND_CAP(feature_bitmap, val)   \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 106, 6, val)

#define WMI_GET_STA_DUMP_SUPPORT(var, feature_bitmap)   \
        WMI_GET_BITS_ARRAY_LEN32_BYTES(var, feature_bitmap, 112, 1)
#define WMI_SET_STA_DUMP_SUPPORT(feature_bitmap, val)   \
        WMI_SET_BITS_ARRAY_LEN32_BYTES(feature_bitmap, 112, 1, val)

/*
 * Specify how many A_UINT32 words are needed to hold the feature bitmap flags.
 * This value may change over time.
 * It is not directly used in any WMI message definition.
 * It is provided simply as a convenience for the feature_set_bitmap sender to
 * know how many 32-bit words to allocate for the bitmap.
 */
#define WMI_FEATURE_SET_BITMAP_ARRAY_LEN32 4

/**
 * TLV for channel list
 */
typedef struct {
    /** WMI_CHAN_LIST_TAG */
    A_UINT32 tag;
    /** # of channels to scan */
    A_UINT32 num_chan;
    /** channels in Mhz */
    A_UINT32 channel_list[1];
} wmi_chan_list;

/**
 * TLV for bssid list
 */
typedef struct {
    /** WMI_BSSID_LIST_TAG */
    A_UINT32 tag;
    /** number of bssids   */
    A_UINT32 num_bssid;
    /** bssid list         */
    wmi_mac_addr bssid_list[1];
} wmi_bssid_list;

/**
 * TLV for  ie data.
 */
typedef struct {
    /** WMI_IE_TAG */
    A_UINT32 tag;
    /** number of bytes in ie data   */
    A_UINT32 ie_len;
    /** ie data array  (ie_len adjusted to  number of words  (ie_len + 4)/4)  */
    A_UINT32 ie_data[1];
} wmi_ie_data;

/**
 * TLV used for length/buffer
 */
typedef struct {
    A_UINT32 tlv_header;    /** TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_tlv_buf_len_param */
    A_UINT32 buf_len;       /** Length of buf */
    /**
     * Following this structure is the TLV byte stream of buf of length buf_len:
     * A_UINT8 buf[];
     *
     */
} wmi_tlv_buf_len_param;

/**
 * TLV used for specifying the dimensions of a multi-dimensional array
 * that has been stored in a flat buffer
 */
typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_tlv_arrays_len_param */
    /**
     * d1_len, d2_len, d3_len, and d4_len are the lengths of each dimension
     * for a multi-dimensional array.
     * If the length of outer dimension is not 1, the inner dimension
     * shouldn't be 1.
     * If the multi-dimensional array has less than 4 dimensions, the outer
     * dimensions' lengths should be 1.  For example, a buf[3][4] array
     * would have d1_len = 4, d2_len = 3, d3_len = 1, d4_len = 1.
     * The outermost dimension of the array can be inferred from the array
     * length; thus, this struct supports up to 5-D arrays.  For a 5-D array,
     * the outermost (5th) dimension would be
     *     array length / (d1_len * d2_len * d3_len * d4_len)
     *
     * For security (to ensure no out-of-bounds memory access),
     * the receiver shall validate that the product of all dimensions
     * is equal to (or less than) the array length.
     */
    A_UINT32 d1_len;  /* the length of 1st (innermost) dimension array */
    A_UINT32 d2_len;  /* the length of 2nd dimension array */
    A_UINT32 d3_len;  /* the length of 3rd dimension array */
    A_UINT32 d4_len;  /* the length of 4th dimension array */
    /**
     * Following this structure is the TLV multi-dimension array buffer:
     * <type> buf[L1*L2*L3*L4];
     * where, L1, L2, L3, and L4 are the values of
     * d1_len, d2_len, d3_len and d4_len.
     * To access the 4-D element a[i][j][k][l], the buf[] array would be
     * indexed as buf[i*L3*L2*L1 + j*L2*L1 + k*L1 + l].
     */
} wmi_tlv_arrays_len_param;

typedef struct {
    /** Len of the SSID */
    A_UINT32 ssid_len;
    /** SSID */
    A_UINT32 ssid[8];
} wmi_ssid;

typedef struct {
    /** WMI_SSID_LIST_TAG */
    A_UINT32 tag;
    A_UINT32 num_ssids;
    wmi_ssid ssids[1];
} wmi_ssid_list;

typedef struct {
    /** WMI_SCAN_START_OFFSET_TAG */
    A_UINT32     tag;
    /** Number of start TSF offsets */
    A_UINT32     num_offset;
    /** Array of start TSF offsets provided in milliseconds */
    A_UINT32     start_tsf_offset[1];
} wmi_scan_start_offset;

/**
 * WLAN_SCAN_CHAN_MODE Macros defined for A_UINT8 phymode_list[]
 */
/** enum WLAN_PHY_MODE _mode starts from 0, but the WMI message requires
 * 0 to be used to represent unspecified / don't care / default values.
 * Therefore, WMI phy mode = WLAN phy mode + 1.
 */
/** If the received WMI phy mode is 0 then it is ignored by the FW,
 * and the FW will use any mode as long as the frequency matches.
 */
/** The number of phy_mode's (BW+mode) passed in the TLV phymode_list[] must
 * be equal to num_chan.  (Unless the host does not specify phymode_list values
 * at all, in which case the number of phymode_list elements will be zero.)
 * The indexing of the phymode_list[] array corresponds to same index of
 * the chan_list[] array.
 */
#define WMI_SCAN_CHAN_SET_MODE(_c) ((_c) + 1)
#define WMI_SCAN_CHAN_GET_MODE(_c) ((_c) - 1)
#define WMI_SCAN_CHAN_MODE_IS_SET(_c) (_c)

typedef struct {
    /*
     * freq unit: MHz (upper 16bits -- value)
     * flags (lower 16bits -- bitfield): valid for the freq short ssid
     *     The flags bitfield contains a bitmask of WMI_SCAN_HINT_FLAG_ values.
     */
    A_UINT32 freq_flags;
    /* per spec, only 4 bytes*/
    A_UINT32 short_ssid;
} wmi_hint_freq_short_ssid;

/** following bssid mac address same as wmi_mac_addr
 *  one example: freq -- 5980(0x175c), flags -- 0x1, mac -- 00:03:7f:12:34:56
 *  freq_flags     will be: 0x175c0001
 *  macaddr31to00 will be: 0x127f0300
 *  macaddr47to32 will be: 0x00005634
 */
typedef struct {
    /*
     * freq unit: MHz (upper 16bits -- value)
     * flags (lower 16bits -- bitfield): valid for the freq bssid
     *     The flags bitfield contains a bitmask of WMI_SCAN_HINT_FLAG_ values.
     */
    A_UINT32 freq_flags;
    /* legacy bssid addr, use same macro to convert: WMI_MAC_ADDR_TO_CHAR_ARRAY, WMI_CHAR_ARRAY_TO_MAC_ADDR */
    wmi_mac_addr bssid;
} wmi_hint_freq_bssid;

/** macro to get freq and corresponding flags from wmi_hint_freq_short_ssid */
#define WMI_GET_FREQ_FROM_HINT_FREQ_SHORT_SSID(pwmi_hint_freq_short_ssid_addr) ((((pwmi_hint_freq_short_ssid_addr)->freq_flags) >> 16) & 0xffff)
#define WMI_GET_FLAGS_FROM_HINT_FREQ_SHORT_SSID(pwmi_hint_freq_short_ssid_addr) (((pwmi_hint_freq_short_ssid_addr)->freq_flags) & 0xffff)

/** macro to set freq and corresponding flags in wmi_hint_freq_short_ssid */
#define WMI_SET_FREQ_IN_HINT_FREQ_SHORT_SSID(freq, pwmi_hint_freq_short_ssid_addr) (((pwmi_hint_freq_short_ssid_addr)->freq_flags) |= ((freq) << 16))
#define WMI_SET_FLAGS_IN_HINT_FREQ_SHORT_SSID(flags, pwmi_hint_freq_short_ssid_addr) (((pwmi_hint_freq_short_ssid_addr)->freq_flags) |= (flags))

/** macro to get freq and corresponding flags from wmi_hint_freq_bssid */
#define WMI_GET_FREQ_FROM_HINT_FREQ_BSSID(pwmi_hint_freq_bssid_addr) ((((pwmi_hint_freq_bssid_addr)->freq_flags) >> 16) & 0xffff)
#define WMI_GET_FLAGS_FROM_HINT_FREQ_BSSID(pwmi_hint_freq_bssid_addr) (((pwmi_hint_freq_bssid_addr)->freq_flags) & 0xffff)

/** macro to set freq and corresponding flags in wmi_hint_freq_bssid */
#define WMI_SET_FREQ_IN_HINT_FREQ_BSSID(freq, pwmi_hint_freq_bssid_addr) (((pwmi_hint_freq_bssid_addr)->freq_flags) |= ((freq) << 16))
#define WMI_SET_FLAGS_IN_HINT_FREQ_BSSID(flags, pwmi_hint_freq_bssid_addr) (((pwmi_hint_freq_bssid_addr)->freq_flags) |= (flags))

/** other macro for 6 GHZ, TU (time unit), 20TU normally it is 20ms */
#define MAX_NUM_20TU_EACH_CH      6
#define MAX_NUM_S_SSID_EACH_20TU  1
#define MAX_NUM_BSSID_EACH_20TU   3

/* prefix used by scan requestor ids on the host */
#define WMI_HOST_SCAN_REQUESTOR_ID_PREFIX 0xA000
/* prefix used by scan request ids generated on the host */
/* host cycles through the lower 12 bits to generate ids */
#define WMI_HOST_SCAN_REQ_ID_PREFIX 0xA000

#define WLAN_SCAN_PARAMS_MAX_SSID    16
#define WLAN_SCAN_PARAMS_MAX_BSSID   4
#define WLAN_SCAN_PARAMS_MAX_IE_LEN  512

/* NOTE: This constant cannot be changed without breaking WMI compatibility */
#define WMI_IE_BITMAP_SIZE             8

#define WMI_SCAN_MLD_PARAM_MLD_ID_GET(mld_param) WMI_GET_BITS(mld_param, 0, 8)
#define WMI_SCAN_MLD_PARAM_MLD_ID_SET(mld_param, val) WMI_SET_BITS(mld_param, 0, 8, val)

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_start_scan_cmd_fixed_param */
    /** Scan ID (lower 16 bits) MSB 4 bits is used to identify scan client based on enum WMI_SCAN_CLIENT_ID */
    A_UINT32 scan_id;
    /** Scan requestor ID (lower 16 bits) is used by scan client to classify the scan source, reason, ...etc */
    A_UINT32 scan_req_id;
    /** VDEV id(interface) that is requesting scan */
    A_UINT32 vdev_id;
    /** Scan Priority, input to scan scheduler */
    A_UINT32 scan_priority;
    /** Scan events subscription */
    A_UINT32 notify_scan_events;
    /** dwell time in msec on active channels */
    A_UINT32 dwell_time_active;
    /** dwell time in msec on passive channels */
    A_UINT32 dwell_time_passive;
    /** min time in msec on the BSS channel,only valid if atleast one VDEV is active*/
    A_UINT32 min_rest_time;
    /** max rest time in msec on the BSS channel,only valid if at least one VDEV is active*/
    /** the scanner will rest on the bss channel at least min_rest_time. after min_rest_time the scanner
     *  will start checking for tx/rx activity on all VDEVs. if there is no activity the scanner will
     *  switch to off channel. if there is activity the scanner will let the radio on the bss channel
     *  until max_rest_time expires.at max_rest_time scanner will switch to off channel
     *  irrespective of activity. activity is determined by the idle_time parameter.
     */
    A_UINT32 max_rest_time;
    /** time before sending next set of probe requests.
     *   The scanner keeps repeating probe requests transmission with period specified by repeat_probe_time.
     *   The number of probe requests specified depends on the ssid_list and bssid_list
     */
    A_UINT32 repeat_probe_time;
    /** time in msec between 2 consecutive probe requests with in a set. */
    A_UINT32 probe_spacing_time;
    /** data inactivity time in msec on bss channel that will be used by scanner for measuring the inactivity  */
    A_UINT32 idle_time;
    /** maximum time in msec allowed for scan  */
    A_UINT32 max_scan_time;
    /** delay in msec before sending first probe request after switching to a channel */
    A_UINT32 probe_delay;
    /** Scan control flags */
    A_UINT32 scan_ctrl_flags;
    /** Burst duration time in msec*/
    A_UINT32 burst_duration;

    /** # if channels to scan. In the TLV channel_list[] */
    A_UINT32 num_chan;
    /** number of bssids. In the TLV bssid_list[] */
    A_UINT32 num_bssid;
    /** number of ssid. In the TLV ssid_list[] */
    A_UINT32 num_ssids;
    /** number of bytes in ie data. In the TLV ie_data[]. Max len is defined by WLAN_SCAN_PARAMS_MAX_IE_LEN */
    A_UINT32 ie_len;
    /** Max number of probes to be sent */
    A_UINT32 n_probes;
    /** MAC Address to use in Probe Req as SA **/
    wmi_mac_addr mac_addr;
    /** Mask on which MAC has to be randomized **/
    wmi_mac_addr mac_mask;
    /**  ie bitmap to use in probe req **/
    A_UINT32 ie_bitmap[WMI_IE_BITMAP_SIZE];
    /** Number of vendor OUIs. In the TLV vendor_oui[] **/
    A_UINT32 num_vendor_oui;
    /** Scan control flags extended **/
    A_UINT32 scan_ctrl_flags_ext;
    /** dwell time in msec on active 2G channels, if it's not zero */
    A_UINT32 dwell_time_active_2g;
    /**
     * dwell time in msec when 6 GHz channel (PSC or non-PSC) is marked
     * as an active channel
     */
    A_UINT32 dwell_time_active_6ghz;
    /**
     * dwell time in msec when 6 GHz channel (PSC or non-PSC) is marked
     * as a passive channel
     */
    A_UINT32 dwell_time_passive_6ghz;
    /**
     * Offset time is in milliseconds per channel.
     */
    A_UINT32 scan_start_offset;
    /**
     * minimum dwell time in msec for 6 GHz channel
     * We'll listen for this time on the channel; if no beacon /
     * probe response / FILS frame are received during this time
     * we'll move to next channel.
     */
    A_UINT32 min_dwell_time_6ghz;
    /**
     * dwell time in msec for 6 GHz channel of spectral scan channel list
     */
    A_UINT32 dwell_time_spectral_ch;
    /**
     * B0-B7: mld id to be inserted in ML probe request
     * B8-B31: reserved
     */
    A_UINT32 mld_parameter;

/**
 * TLV (tag length value) parameters follow the scan_cmd
 * structure. The TLV's are:
 *     channel_list:
 *         If FW supports WMI_SERVICE_SCAN_CONFIG_PER_CHANNEL,
 *             then channel_list may fill the upper 12 bits with channel flags,
 *             while using only the lower 20 bits for channel frequency.
 *             Check WMI_SCAN_CHANNEL_FLAG macros for the channel flags
 *         If FW doesn't support WMI_SERVICE_SCAN_CONFIG_PER_CHANNEL,
 *             then channel_list only holds the frequency value
 *         Use WMI_SCAN_CHANNEL_FREQ_MASK & WMI_SCAN_CHANNEL_FLAGS_MASK
 *     A_UINT32 channel_list[num_chan]; // in MHz
 *     wmi_ssid ssid_list[num_ssids];
 *     wmi_mac_addr bssid_list[num_bssid];
 *     A_UINT8 ie_data[ie_len];
 *     wmi_vendor_oui vendor_oui[num_vendor_oui];
 *     A_UINT8 phymode_list[0 or num_chan]; // see WMI_SCAN_CHAN_MODE macros
 *     wmi_hint_freq_short_ssid hint_freq_short_ssid[num]; // the num can be calculated by TLV len
 *     wmi_hint_freq_bssid hint_freq_bssid[num]; // the num can be calculated by TLV len
 *     A_UINT32 spectral_chan_list[num]; // in MHz, the num can be calculated by TLV len
 *     *** NOTE:
 *     *** Use caution when using further TLVs, in case the additional
 *     *** TLVs cause the message size to exceed the of the buffer to
 *     *** hold the message.
 */
} wmi_start_scan_cmd_fixed_param;

/**
 * scan control flags.
 */

/** passively scan all channels including active channels */
#define WMI_SCAN_FLAG_PASSIVE        0x1
/** add wild card ssid probe request even though ssid_list is specified. */
#define WMI_SCAN_ADD_BCAST_PROBE_REQ 0x2
/** add cck rates to rates/xrate ie for the generated probe request */
#define WMI_SCAN_ADD_CCK_RATES       0x4
/** add ofdm rates to rates/xrate ie for the generated probe request */
#define WMI_SCAN_ADD_OFDM_RATES      0x8
/** To enable indication of Chan load and Noise floor to host */
#define WMI_SCAN_CHAN_STAT_EVENT     0x10
/** Filter Probe request frames  */
#define WMI_SCAN_FILTER_PROBE_REQ    0x20
/**When set, not to scan DFS channels*/
#define WMI_SCAN_BYPASS_DFS_CHN      0x40
/**When set, certain errors are ignored and scan continues.
 * Different FW scan engine may use its own logic to decide what errors to ignore*/
#define WMI_SCAN_CONTINUE_ON_ERROR   0x80
/** Enable promiscuous mode for CCXv4 */
#define WMI_SCAN_FILTER_PROMISCOUS   0x100
/** allow to send probe req on DFS channel */
#define WMI_SCAN_FLAG_FORCE_ACTIVE_ON_DFS 0x200
/** add TPC content in probe req frame */
#define WMI_SCAN_ADD_TPC_IE_IN_PROBE_REQ  0x400
/** add DS content in probe req frame */
#define WMI_SCAN_ADD_DS_IE_IN_PROBE_REQ   0x800
/** use random mac address for TA for probe request frame and add
 * oui specified by WMI_SCAN_PROB_REQ_OUI_CMDID to the probe req frame.
 * if oui is not set by WMI_SCAN_PROB_REQ_OUI_CMDID  then the flag is ignored*/
#define WMI_SCAN_ADD_SPOOFED_MAC_IN_PROBE_REQ   0x1000
/** allow mgmt transmission during off channel scan */
#define WMI_SCAN_OFFCHAN_MGMT_TX    0x2000
/** allow data transmission during off channel scan */
#define WMI_SCAN_OFFCHAN_DATA_TX    0x4000
/** allow capture ppdu with phy errors */
#define WMI_SCAN_CAPTURE_PHY_ERROR  0x8000
/** always do passive scan on passive channels */
#define WMI_SCAN_FLAG_STRICT_PASSIVE_ON_PCHN 0x10000
/** set HALF (10MHz) rate support */
#define WMI_SCAN_FLAG_HALF_RATE_SUPPORT      0x20000
/** set Quarter (5MHz) rate support */
#define WMI_SCAN_FLAG_QUARTER_RATE_SUPPORT   0x40000
#define WMI_SCAN_RANDOM_SEQ_NO_IN_PROBE_REQ 0x80000
#define WMI_SCAN_ENABLE_IE_WHTELIST_IN_PROBE_REQ 0x100000
/** pause home channel when scan channel is same as home channel */
#define WMI_SCAN_FLAG_PAUSE_HOME_CHANNEL            0x200000
/**
 * report CCA busy for each possible 20Mhz subbands of the wideband scan channel
 */
#define WMI_SCAN_FLAG_REPORT_CCA_BUSY_FOREACH_20MHZ 0x400000

/** for adaptive scan mode using 3 bits (21 - 23 bits) */
#define WMI_SCAN_DWELL_MODE_MASK 0x00E00000
#define WMI_SCAN_DWELL_MODE_SHIFT        21

typedef enum {
    WMI_SCAN_DWELL_MODE_DEFAULT      = 0,
    WMI_SCAN_DWELL_MODE_CONSERVATIVE = 1,
    WMI_SCAN_DWELL_MODE_MODERATE     = 2,
    WMI_SCAN_DWELL_MODE_AGGRESSIVE   = 3,
    WMI_SCAN_DWELL_MODE_STATIC       = 4,
} WMI_SCAN_DWELL_MODE;

#define WMI_SCAN_SET_DWELL_MODE(flag, mode) \
    do { \
        (flag) |= (((mode) << WMI_SCAN_DWELL_MODE_SHIFT) & \
            WMI_SCAN_DWELL_MODE_MASK); \
    } while (0)

#define WMI_SCAN_GET_DWELL_MODE(flag) \
    (((flag) & WMI_SCAN_DWELL_MODE_MASK) >> WMI_SCAN_DWELL_MODE_SHIFT)

/** WMI_SCAN_CLASS_MASK must be the same value as IEEE80211_SCAN_CLASS_MASK */
#define WMI_SCAN_CLASS_MASK 0xFF000000

/*
 * Masks identifying types/ID of scans
 * Scan_Stop macros should be the same value as below defined in UMAC
 * #define IEEE80211_SPECIFIC_SCAN       0x00000000
 * #define IEEE80211_VAP_SCAN            0x01000000
 * #define IEEE80211_ALL_SCANS           0x04000000
 */
/* WMI_SCAN_STOP_ONE:
 * Stop one scan which matches with scan_id provided in scan stop command.
 */
#define WMI_SCAN_STOP_ONE         0x00000000
/* WMI_SCN_STOP_VAP_ALL:
 * Stop all scans (host scans and FW internal scans) on provided vdev.
 */
#define WMI_SCN_STOP_VAP_ALL      0x01000000
/* WMI_SCN_STOP_HOST_VAP_ALL:
 * Stop all host scans on provided vdev.
 */
#define WMI_SCN_STOP_HOST_VAP_ALL 0x02000000
/* WMI_SCAN_STOP_ALL:
 * Stop all scans (host scans and FW internal scans) on all vdevs.
 */
#define WMI_SCAN_STOP_ALL         0x04000000

/** extended Scan ctrl flags **/
#define WMI_SCAN_FLAG_EXT_DBS_SCAN_POLICY_MASK 0x00000003 /* Bit 0-1 reserved for DBS scan selection policy.*/

#define WMI_SCAN_DBS_POLICY_DEFAULT             0x0 /** Select duty cycle if configured, else fall back to whatever
                                                        policy scan manager computes */
#define WMI_SCAN_DBS_POLICY_FORCE_NONDBS        0x1 /** Force to select Non-DBS scan */
#define WMI_SCAN_DBS_POLICY_IGNORE_DUTY         0x2 /** Ignore duty cycle even if configured and fall back to whatever
                                                        policy scan manager computes*/
#define WMI_SCAN_DBS_POLICY_RESERVED            0x3
#define WMI_SCAN_DBS_POLICY_MAX                 0x3

/* Enable Reception of Public Action frame with this flag */
#define WMI_SCAN_FLAG_EXT_FILTER_PUBLIC_ACTION_FRAME  0x00000004

/* Indicate to scan all PSC channel */
#define WMI_SCAN_FLAG_EXT_6GHZ_SCAN_ALL_PSC_CH        0x00000008

/* Indicate to scan all NON-PSC channel */
#define WMI_SCAN_FLAG_EXT_6GHZ_SCAN_ALL_NON_PSC_CH    0x00000010

/* Indicate to save scan result matching hint from scan client */
#define WMI_SCAN_FLAG_EXT_6GHZ_MATCH_HINT             0x00000020

/* Skip any ch on which no any RNR had been received */
#define WMI_SCAN_FLAG_EXT_6GHZ_SKIP_NON_RNR_CH        0x00000040

/* Indicate client hint req is high priority than fw rnr or FILS disc */
#define WMI_SCAN_FLAG_EXT_6GHZ_CLIENT_HIGH_PRIORITY   0x00000080

/* Force all 6 GHz scan channels to active channel */
#define WMI_SCAN_FLAG_EXT_6GHZ_FORCE_CHAN_ACTIVE      0x00000100

/* Force broadcast address in RA even though specified bssid */
#define WMI_SCAN_FLAG_EXT_FORCE_BRCAST_RA             0x00000200

/* Extend 6 GHz channel measure time */
#define WMI_SCAN_FLAG_EXT_6GHZ_EXTEND_MEASURE_TIME    0x00000400

/**
 * Currently passive scan has higher priority than beacon and
 * beacon miss would happen irrespective of dwell time.
 * Below flag ensures there would not be beacon miss if the dwell
 * time is lesser than beacon interval - channel switch time combined.
 * For dwell time greater than beacon interval, bmiss is expected.
 */
#define WMI_SCAN_FLAG_EXT_PASSIVE_SCAN_START_TIME_ENHANCE   0x00000800

/* Force unicast address in RA */
#define WMI_SCAN_FLAG_EXT_FORCE_UNICAST_RA            0x00001000

/**
 * Indicate to add 10 Mhz offset to spectral scan center frequencies
 * sent by Host when checking against support channel list in FW
 */
#define WMI_SCAN_FLAG_EXT_SPECTRAL_CFREQ_PLUS_10MHZ_IN_SUPP_CH_LIST 0x00002000

/* Include MLO IE in Probe req */
#define WMI_SCAN_FLAG_EXT_INCL_MLIE_PRB_REQ           0x00004000

#define WMI_SCAN_FLAG_EXT_LOW_LATENCY_SCAN    0x00008000
#define WMI_SCAN_FLAG_EXT_RELIABLE_SCAN       0x00010000
#define WMI_SCAN_FLAG_EXT_FAST_SCAN           0x00020000
#define WMI_SCAN_FLAG_EXT_LOW_POWER_SCAN      0x00040000
#define WMI_SCAN_FLAG_EXT_STOP_IF_BSSID_FOUND 0x00080000


/**
 * new 6 GHz flags per chan (short ssid or bssid) in struct
 * wmi_hint_freq_short_ssid or wmi_hint_freq_bssid
 */
/* Indicate not to send probe req for short_ssid or bssid on that channel */
#define WMI_SCAN_HINT_FLAG_SKIP_TX_PROBE_REQ    0x00000001

/* Force channel in WMI hint to active channel */
#define WMI_SCAN_HINT_FLAG_FORCE_CHAN_ACTIVE    0x00000002

/* Combine short SSID with legacy bssid list */
#define WMI_SCAN_HINT_FLAG_COMBINE_BSSID_LIST   0x00000004


#define WMI_SCAN_CHANNEL_FREQ_MASK  0x000FFFFF
#define WMI_SCAN_CHANNEL_FLAGS_MASK 0xFFF00000

/**
 * Per channel configuration flags
 */

/**
 * WMI_SCAN_CHANNEL_FLAG_SCAN_ONLY_IF_RNR_FOUND:
 *     If this flag is set, then scan only if the corresponding channel
 *     is found via RNR IE during 2g/5g scan.
 *     If this flag is not set, then FW always scans the channel
 *     irrespective of RNR and also FW ignores
 *     WMI_SCAN_FLAG_EXT_6GHZ_SKIP_NON_RNR_CH flag
 */
#define WMI_SCAN_CHANNEL_FLAG_SCAN_ONLY_IF_RNR_FOUND 0x001

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_stop_scan_cmd_fixed_param */
    /** requestor requesting cancel  */
    A_UINT32 requestor;
    /** Scan ID */
    A_UINT32 scan_id;
    /**
     * Req Type
     * req_type should be WMI_SCAN_STOP_ONE, WMI_SCN_STOP_VAP_ALL or WMI_SCAN_STOP_ALL
     * WMI_SCAN_STOP_ONE indicates to stop a specific scan with scan_id (on a specific pdev in DBDC)
     * WMI_SCN_STOP_VAP_ALL indicates to stop all scan requests on a specific vDev with vdev_id
     * WMI_SCAN_STOP_ALL indicates to stop all scan requests in both Scheduler's queue and Scan Engine (on a specific pdev in DBDC)
     */
    A_UINT32 req_type;
    /**
     * vDev ID
     * used when req_type equals to WMI_SCN_STOP_VAP_ALL, it indexed the vDev on which to stop the scan
     */
    A_UINT32 vdev_id;
    /** pdev_id for identifying the MAC
     * See macros starting with WMI_PDEV_ID_ for values.
     * In non-DBDC case host should set it to 0
     */
    A_UINT32 pdev_id;
} wmi_stop_scan_cmd_fixed_param;


#define MAX_NUM_CHAN_PER_WMI_CMD     58    /* each WMI cmd can hold 58 channel entries at most */
#define APPEND_TO_EXISTING_CHAN_LIST 1
#define CHANNEL_MAX_BANDWIDTH_VALID  2

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_scan_chan_list_cmd_fixed_param */
    A_UINT32 num_scan_chans; /** no of elements in chan_info[] */
    A_UINT32 flags; /* Flags used to control the behavior of channel list update on target side */
    A_UINT32 pdev_id; /** pdev_id for identifying the MAC.  See macros starting with WMI_PDEV_ID_ for values. In non-DBDC case host should set it to 0. */
/** Followed by the variable length TLV chan_info:
 *  wmi_channel chan_info[] */
} wmi_scan_chan_list_cmd_fixed_param;

/*
 * Priority numbers must be sequential, starting with 0.
 */
/* NOTE: WLAN SCAN_PRIORITY_COUNT can't be changed without breaking the compatibility */
typedef enum {
    WMI_SCAN_PRIORITY_VERY_LOW = 0,
    WMI_SCAN_PRIORITY_LOW,
    WMI_SCAN_PRIORITY_MEDIUM,
    WMI_SCAN_PRIORITY_HIGH,
    WMI_SCAN_PRIORITY_VERY_HIGH,

    WMI_SCAN_PRIORITY_COUNT /* number of priorities supported */
} wmi_scan_priority;

/* Five Levels for Requested Priority */
/* VERY_LOW LOW  MEDIUM   HIGH  VERY_HIGH */
typedef A_UINT32 WLAN_PRIORITY_MAPPING[WMI_SCAN_PRIORITY_COUNT];

/**
 * to keep align with UMAC implementation, we pass only vdev_type but not vdev_subtype when we overwrite an entry for a specific vdev_subtype
 * ex. if we need overwrite P2P Client prority entry, we will overwrite the whole table for WLAN_M_STA
 * we will generate the new WLAN_M_STA table with modified P2P Client Entry but keep STA entry intact
 */
typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_scan_sch_priority_table_cmd_fixed_param */
    /**
     * used as an index to find the proper table for a specific vdev type in default_scan_priority_mapping_table
     * vdev_type should be one of enum in WLAN_OPMODE which inculdes WLAN_M_IBSS, WLAN_M_STA, WLAN_M_AP and WLAN_M_MONITOR currently
     */
    A_UINT32 vdev_type;
    /**
     * number of rows in mapping_table for a specific vdev
     * for WLAN_M_STA type, there are 3 entries in the table (refer to default_scan_priority_mapping_table definition)
     */
    A_UINT32 number_rows;
    /**
     * pdev_id for identifying the MAC.  See macros starting with WMI_PDEV_ID_ for values.
     * In non-DBDC case host should set it to 0
     */
    A_UINT32 pdev_id;
/**  mapping_table for a specific vdev follows this TLV
 *   WLAN_PRIORITY_MAPPING mapping_table[]; */
} wmi_scan_sch_priority_table_cmd_fixed_param;

/** update flags */
#define WMI_SCAN_UPDATE_SCAN_PRIORITY           0x1
#define WMI_SCAN_UPDATE_SCAN_MIN_REST_TIME      0x2
#define WMI_SCAN_UPDATE_SCAN_MAX_REST_TIME      0x4

typedef struct {
    A_UINT32 tlv_header;
    /** requestor requesting update scan request  */
    A_UINT32 requestor;
    /** Scan ID of the scan request that need to be update */
    A_UINT32 scan_id;
    /** update flags, indicating which of the following fields are valid and need to be updated*/
    A_UINT32 scan_update_flags;
    /** scan priority. Only valid if WMI_SCAN_UPDATE_SCAN_PRIORITY flag is set in scan_update_flag */
    A_UINT32 scan_priority;
    /** min rest time. Only valid if WMI_SCAN_UPDATE_MIN_REST_TIME flag is set in scan_update_flag */
    A_UINT32 min_rest_time;
    /** min rest time. Only valid if WMI_SCAN_UPDATE_MAX_REST_TIME flag is set in scan_update_flag */
    A_UINT32 max_rest_time;
    /** pdev_id for identifying the MAC.  See macros starting with WMI_PDEV_ID_ for values. In non-DBDC case host should set it to 0 */
    A_UINT32 pdev_id;
} wmi_scan_update_request_cmd_fixed_param;

#define WMI_SCAN_PROBE_OUI_SPOOFED_MAC_IN_PROBE_REQ 0x1
#define WMI_SCAN_PROBE_OUI_RANDOM_SEQ_NO_IN_PROBE_REQ 0x2
#define WMI_SCAN_PROBE_OUI_ENABLE_IE_WHITELIST_IN_PROBE_REQ 0x4

typedef struct _wmi_vendor_oui {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_vendor_oui */
    A_UINT32 oui_type_subtype; /** Vendor OUI type and subtype, lower 3 bytes is type and highest byte is subtype**/
}wmi_vendor_oui;

typedef struct {
    A_UINT32 tlv_header;
    /** oui to be used in probe request frame when  random mac addresss is
     * requested part of scan parameters. this is applied to both FW internal scans and
     * host initated scans. host can request for random mac address with
     * WMI_SCAN_ADD_SPOOFED_MAC_IN_PROBE_REQ flag.     */
    A_UINT32 prob_req_oui;
    A_UINT32 vdev_id;
    /** Control Flags **/
    A_UINT32 flags;
    /**  ie bitmap to use in probe req **/
    A_UINT32 ie_bitmap[WMI_IE_BITMAP_SIZE];
    /** Number of vendor OUIs. In the TLV vendor_oui[] **/
    A_UINT32 num_vendor_oui;
    /** pdev_id for identifying the MAC.  See macros starting with WMI_PDEV_ID_ for values. In non-DBDC case host should set it to 0 */
    A_UINT32 pdev_id;
    /* Following this tlv, there comes an array of structure of type wmi_vendor_oui
 wmi_vendor_oui vendor_oui[];*/
} wmi_scan_prob_req_oui_cmd_fixed_param;


enum wmi_scan_event_type {
    WMI_SCAN_EVENT_STARTED = 0x1,
    WMI_SCAN_EVENT_COMPLETED = 0x2,
    WMI_SCAN_EVENT_BSS_CHANNEL = 0x4,
    WMI_SCAN_EVENT_FOREIGN_CHANNEL = 0x8,
    WMI_SCAN_EVENT_DEQUEUED = 0x10, /* scan request got dequeued */
    WMI_SCAN_EVENT_PREEMPTED = 0x20, /* preempted by other high priority scan */
    WMI_SCAN_EVENT_START_FAILED = 0x40, /* scan start failed */
    WMI_SCAN_EVENT_RESTARTED = 0x80, /* scan restarted */
    WMI_SCAN_EVENT_FOREIGN_CHANNEL_EXIT = 0x100,
    WMI_SCAN_EVENT_SUSPENDED = 0x200, /* scan request is suspended */
    WMI_SCAN_EVENT_RESUMED = 0x400,   /* scan request is resumed */
    WMI_SCAN_EVENT_MAX = 0x8000
};

enum wmi_scan_completion_reason {
    /** scan related events */
    WMI_SCAN_REASON_NONE = 0xFF,
    WMI_SCAN_REASON_COMPLETED = 0,
    WMI_SCAN_REASON_CANCELLED = 1,
    WMI_SCAN_REASON_PREEMPTED = 2,
    WMI_SCAN_REASON_TIMEDOUT = 3,
    WMI_SCAN_REASON_INTERNAL_FAILURE = 4, /* This reason indication failures when performaing scan */
    WMI_SCAN_REASON_SUSPENDED = 5,
    WMI_SCAN_REASON_DFS_VIOLATION = 6, /* Failure when tried to SCAN channel in NOL list */
    WMI_SCAN_REASON_MAX,
};

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_scan_event_fixed_param */
    /** scan event (wmi_scan_event_type) */
    A_UINT32 event;
    /** status of the scan completion event */
    A_UINT32 reason;
    /** channel freq , only valid for FOREIGN channel event*/
    A_UINT32 channel_freq;
    /**id of the requestor whose scan is in progress */
    A_UINT32 requestor;
    /**id of the scan that is in progress */
    A_UINT32 scan_id;
    /**id of VDEV that requested the scan */
    A_UINT32 vdev_id;
    /** TSF Timestamp when the scan event (wmi_scan_event_type) is completed
     * In case of AP it is TSF of the AP vdev
     * In case of STA connected state this is the TSF of the AP
     * In case of STA not connected it will be the free running HW timer
     */
    A_UINT32 tsf_timestamp;
} wmi_scan_event_fixed_param;

/* WMI Diag event */
typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag is WMITLV_TAG_STRUC_wmi_diag_event_fixed_param */
    A_UINT32 time_stamp; /* Reference timestamp. diag frame contains diff value */
    A_UINT32 count;   /* Number of diag frames added to current event */
    A_UINT32 dropped;
    /* followed by WMITLV_TAG_ARRAY_BYTE */
} wmi_diag_event_fixed_param;

#define WMI_11K_OFFLOAD_BITMAP_NEIGHBOR_REPORT_REQ  0x1

typedef struct {
    A_UINT32 time_offset;                   /* positive offset in secs from the time 11k offload command has been received, 0xFFFFFFFF if offset is not valid */
    A_UINT32 low_rssi_offset;               /* positive offset in dB from current low RSSI roaming trigger to send neighbor req, 0xFFFFFFFF if offset is not valid */
    A_UINT32 bmiss_count_trigger;           /* value 1 is to send neighbor report at 1st BMISS, 0xFFFFFFFF if input is not valid */
    A_UINT32 per_threshold_offset;          /* percentage offset from the current per_threshold, 0xFFFFFFFF if input is not valid */
    A_UINT32 neighbor_report_cache_timeout; /* cache timeout in secs after which neighbor cache is not valid in FW, 0xFFFFFFFF if input is not valid */
    A_UINT32 max_neighbor_report_req_cap;   /* 0xFFFFFFFF if input is not valid, else positive number per every roam, these are the maximum number of
                                             * neighbor report requests that will be sent by FW after every roam */
    wmi_ssid ssid;                          /* ssid of current connected AP FW might choose to use this SSID in the neighbor report req frame if it is
                                             * interested in candidate of the same SSID */
} wmi_neighbor_report_offload;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_offload_11k_report_fixed_param */
    A_UINT32 vdev_id;
    A_UINT32 offload_11k; /* bitmask to indicate to FW what all 11k features are offloaded */
} wmi_11k_offload_report_fixed_param;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_neighbor_report_offload_tlv_param */
    wmi_neighbor_report_offload neighbor_rep_ofld_params;
} wmi_neighbor_report_11k_offload_tlv_param;

#define WMI_INVOKE_NEIGHBOR_REPORT_FLAGS_SEND_RESP_TO_HOST 0x1

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_invoke_neighbor_report_fixed_param */
    A_UINT32 vdev_id;
    A_UINT32 flags;
    wmi_ssid ssid; /* if ssid.len == 0, firmware doesn't include ssid sub-element.
                    * In that case AP gives all the candidates in ESS without SSID filter
                    * If host wants to insert ssid subelement in the neighbor report request frame, then it can specify the ssid here */
} wmi_11k_offload_invoke_neighbor_report_fixed_param;

#define WMI_MAX_PMKID_LEN   16
#define WMI_MAX_PMK_LEN     64

#define WMI_PMK_CACHE_CAT_FLAG_BSSID              0x1
#define WMI_PMK_CACHE_CAT_FLAG_SSID_CACHE_ID      0x2

#define WMI_PMK_CACHE_ACTION_FLAG_ADD_ENTRY       0x1
#define WMI_PMK_CACHE_ACTION_FLAG_DEL_ENTRY       0x2

typedef struct {
    A_UINT32       tlv_header;
    A_UINT32       pmk_len;
    A_UINT8        pmk[WMI_MAX_PMK_LEN];/* for big-endian hosts, manual endian conversion will be needed to keep the array values in their original order,
                                        in spite of the automatic byte-swap applied to WMI messages during download*/
    A_UINT32       pmkid_len;
    A_UINT8        pmkid[WMI_MAX_PMKID_LEN];
    wmi_mac_addr   bssid;
    wmi_ssid       ssid;
    A_UINT32       cache_id;
    A_UINT32       cat_flag;  // whether (bssid) or (ssid,cache_id) is valid
    A_UINT32       action_flag;  // add/delete the entry
} wmi_pmk_cache;

#define WMI_PMK_CACHE_OP_FLAG_FLUSH_ALL       0x1

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_pdev_update_pmk_cache_cmd_fixed_param */
    A_UINT32 op_flag;   //option to flush all the cache at once
    A_UINT32 vdev_id;
    A_UINT32 num_cache;
    /**
    * TLV (tag length value) parameters follow the update_pmk_cache cmd
    * structure. The TLV's are:
     * wmi_pmk_cache cache_list[];
    */
} wmi_pdev_update_pmk_cache_cmd_fixed_param;

#define WMI_FILS_MAX_USERNAME_LEN 16
#define WMI_FILS_MAX_REALM_LEN 256
#define WMI_FILS_MAX_RRK_LEN 64
#define WMI_FILS_MAX_RIK_LEN 64

/* for big-endian hosts, manual endian conversion will be needed to keep the array values in their original order,
in spite of the automatic byte-swap applied to WMI messages during download*/

typedef struct {
    A_UINT8     username[WMI_FILS_MAX_USERNAME_LEN];
    A_UINT32    username_length;
    A_UINT32    next_erp_seq_num;
    A_UINT8     rRk[WMI_FILS_MAX_RRK_LEN];
    A_UINT32    rRk_length;
    A_UINT8     rIk[WMI_FILS_MAX_RIK_LEN];
    A_UINT32    rIk_length;
    A_UINT8     realm[WMI_FILS_MAX_REALM_LEN];
    A_UINT32    realm_len;
} wmi_erp_info;

enum wmi_fils_hlp_pkt_type {
    WMI_FILS_HLP_PKT_TYPE_DHCP_DISCOVER = 1,
};

typedef struct {
    A_UINT32      tlv_header;  /** TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_roam_fils_offload_tlv_param */
    A_UINT32      flags;
    wmi_erp_info  vdev_erp_info;
} wmi_roam_fils_offload_tlv_param;

typedef struct {
    A_UINT32  tlv_header; /** tag WMITLV_TAG_STRUC_wmi_pdev_update_fils_hlp_pkt_cmd_fixed_param**/
    A_UINT32  flags;
    A_UINT32  vdev_id;
    A_UINT32  size;
    A_UINT32  pkt_type; // filled using enum wmi_fils_hlp_pkt_type
 // A_UINT8          fils_hlp_pkt[];
} wmi_pdev_update_fils_hlp_pkt_cmd_fixed_param;

#define WMI_MAX_KEK_LEN 64
#define GTK_OFFLOAD_KEK_EXTENDED_BYTES WMI_MAX_KEK_LEN /*KEK len has been increased to 64 to support FILS security.
                                          To not break backward compatibility, new GTK_OFFLOAD_KEK_EXTENDED_BYTES has been defined without modifying old GTK_OFFLOAD_KEK_BYTES */

typedef struct {
    A_UINT32   tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_roam_fils_synch_tlv_param */
    A_UINT32   update_erp_next_seq_num;// Boolean denoting whether next erp_seq_num changed or not.
    A_UINT32   next_erp_seq_num;
    A_UINT32   kek_len;
    A_UINT8    kek[WMI_MAX_KEK_LEN];
    A_UINT32   pmk_len;
    A_UINT8    pmk[WMI_MAX_PMK_LEN];
    A_UINT8    pmkid[WMI_MAX_PMKID_LEN];
    A_UINT8    realm[WMI_FILS_MAX_REALM_LEN];
    A_UINT32   realm_len;
} wmi_roam_fils_synch_tlv_param;

/*
 * FW sends PMK cache of roamed candidate to host to sync pmk cache with host
 */
typedef struct {
    A_UINT32  tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_roam_pmk_cache_synch_tlv_param */
    A_UINT32  pmk_len;
    A_UINT8  pmk[WMI_MAX_PMK_LEN];
    A_UINT8  pmkid[WMI_MAX_PMKID_LEN];
} wmi_roam_pmk_cache_synch_tlv_param;

/**
 * WMI_ROAM_LINK_FLAG_XXX definition:
 */
#define WMI_ROAM_LINK_FLAG_DISABLE    0x1   /* link is disabled, host can overwrite it later. */

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_roam_ml_setup_links_param */
    A_UINT32 vdev_id; /* when vdev_id is 0xffffffff, means the link_id hasn't attached to vdev */
    A_UINT32 link_id; /* link id defined as in 802.11 BE spec. */
    wmi_channel channel; /* link primary channel */
    /**
     * link flags: refer WMI_ROAM_LINK_FLAG_XXX.
     */
    A_UINT32 flags;
    wmi_mac_addr link_addr; /* link address */
    wmi_mac_addr self_link_addr; /* self-link address */
} wmi_roam_ml_setup_links_param;

/*
 * If FW has multiple active channels due to MCC(multi channel concurrency),
 * then these stats are combined stats for all the active channels.
 */
typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_update_whal_mib_stats_event_fixed_param */
    /** ack count, it is an incremental number, not accumulated number */
    A_UINT32 ackRcvBad;
    /** bad rts count, it is an incremental number, not accumulated number */
    A_UINT32 rtsBad;
    /** good rts, it is an incremental number, not accumulated number */
    A_UINT32 rtsGood;
    /** fcs count, it is an incremental number, not accumulated number */
    A_UINT32 fcsBad;
    /** beacon count, it is an incremental number, not accumulated number */
    A_UINT32 noBeacons;
} wmi_update_whal_mib_stats_event_fixed_param;

/*
 * This defines how much headroom is kept in the
 * receive frame between the descriptor and the
 * payload, in order for the WMI PHY error and
 * management handler to insert header contents.
 *
 * This is in bytes.
 */
#define WMI_MGMT_RX_HDR_HEADROOM (sizeof(wmi_comb_phyerr_rx_hdr) + WMI_TLV_HDR_SIZE + sizeof(wmi_single_phyerr_rx_hdr))

/** This event will be used for sending scan results
 * as well as rx mgmt frames to the host. The rx buffer
 * will be sent as part of this WMI event. It would be a
 * good idea to pass all the fields in the RX status
 * descriptor up to the host.
 */
/* ATH_MAX_ANTENNA value (4) can't be changed without breaking the compatibility */
#define ATH_MAX_ANTENNA 4 /* To support beelinear, which is up to 4 chains */

/** flag indicating that the mgmt frame (probe req/beacon) is received in the context of extscan performed by FW */
#define WMI_MGMT_RX_HDR_EXTSCAN     0x01
/** flag indicating that the mgmt frame (probe req/beacon) is received in the context of matched network by FW ENLO */
#define WMI_MGMT_RX_HDR_ENLO     0x02

#define MAX_ANTENNA_EIGHT 8


/** Helper macro for params GET/SET of MGMT_RX_FW_CONSUMED_EVENTID */
#define WMI_MGMT_RX_FW_CONSUMED_PARAM_PEER_ID_GET(peer_info_subtype) WMI_GET_BITS(peer_info_subtype, 0, 16)
#define WMI_MGMT_RX_FW_CONSUMED_PARAM_PEER_ID_SET(peer_info_subtype, value) WMI_SET_BITS(peer_info_subtype, 0, 16, value)

#define WMI_MGMT_RX_FW_CONSUMED_PARAM_IEEE_LINK_ID_GET(peer_info_subtype) WMI_GET_BITS(peer_info_subtype, 16, 3)
#define WMI_MGMT_RX_FW_CONSUMED_PARAM_IEEE_LINK_ID_SET(peer_info_subtype, value) WMI_SET_BITS(peer_info_subtype, 16, 3, value)

#define WMI_MGMT_RX_FW_CONSUMED_PARAM_SUBTYPE_GET(peer_info_subtype) WMI_GET_BITS(peer_info_subtype, 28, 4)
#define WMI_MGMT_RX_FW_CONSUMED_PARAM_SUBTYPE_SET(peer_info_subtype, value) WMI_SET_BITS(peer_info_subtype, 28, 4, value)

#define WMI_MGMT_RX_FW_CONSUMED_PARAM_MGMT_PKT_CTR_VALID_GET(mgmt_pkt_ctr_info) WMI_GET_BITS(mgmt_pkt_ctr_info, 15, 1)
#define WMI_MGMT_RX_FW_CONSUMED_PARAM_MGMT_PKT_CTR_VALID_SET(mgmt_pkt_ctr_info, value) WMI_SET_BITS(mgmt_pkt_ctr_info, 15, 1, value)

#define WMI_MGMT_RX_FW_CONSUMED_PARAM_MGMT_PKT_CTR_GET(mgmt_pkt_ctr_info) WMI_GET_BITS(mgmt_pkt_ctr_info, 16, 16)
#define WMI_MGMT_RX_FW_CONSUMED_PARAM_MGMT_PKT_CTR_SET(mgmt_pkt_ctr_info, value) WMI_SET_BITS(mgmt_pkt_ctr_info, 16, 16, value)

typedef struct {
    A_UINT32 tlv_header; /* WMITLV_TAG_STRUCT_wmi_mgmt_rx_fw_consumed_hdr */
    A_UINT32 rx_tsf_l32; /* h/w assigned timestamp of the rx frame in micro sec */
    A_UINT32 rx_tsf_u32 ;/* h/w assigned timestamp of the rx frame in micro sec */
    A_UINT32 pdev_id; /* pdev_id for identifying the MAC the rx mgmt frame was received by */
    /**
     * peer_info_subtype
     *
     * [15:0]:  ml_peer_id, ML peer_id unique across chips
     * [18:16]: ieee_link_id, protocol link id on which the rx frame is received
     * [27:19]: reserved
     * [31:28]: subtype, subtype of the received MGMT frame
     */
    A_UINT32 peer_info_subtype;
    A_UINT32 chan_freq; /* frequency in MHz of the channel on which this frame was received */
    /* Timestamp (in micro sec) of the last fw consumed/dropped mgmt. frame, same across chips */
    A_UINT32 global_timestamp;
    /**
     * mgmt_pkt_ctr_info
     *
     * [14:0]:  reserved
     * [15]:    mgmt_pkt_ctr_valid
     * [31:16]: mgmt_pkt_ctr, Sequence number of the last fw consumed mgmt frame
     */
    A_UINT32 mgmt_pkt_ctr_info;
    A_UINT32 rx_ppdu_duration_us; /* receive duration in us */
    A_UINT32 mpdu_end_timestamp; /* mpdu end timestamp in us (based on HWMLO timer) */
} wmi_mgmt_rx_fw_consumed_hdr;

/** Helper macro for param GET/SET of mgmt_rx_reo_params */
#define WMI_MGMT_RX_REO_PARAM_IEEE_LINK_ID_GET(mgmt_pkt_ctr_link_info) WMI_GET_BITS(mgmt_pkt_ctr_link_info, 12, 3)
#define WMI_MGMT_RX_REO_PARAM_IEEE_LINK_ID_SET(mgmt_pkt_ctr_link_info, value) WMI_SET_BITS(mgmt_pkt_ctr_link_info, 12, 3, value)

#define WMI_MGMT_RX_REO_PARAM_MGMT_PKT_CTR_VALID_GET(mgmt_pkt_ctr_link_info) WMI_GET_BITS(mgmt_pkt_ctr_link_info, 15, 1)
#define WMI_MGMT_RX_REO_PARAM_MGMT_PKT_CTR_VALID_SET(mgmt_pkt_ctr_link_info, value) WMI_SET_BITS(mgmt_pkt_ctr_link_info, 15, 1, value)

#define WMI_MGMT_RX_REO_PARAM_MGMT_PKT_CTR_GET(mgmt_pkt_ctr_link_info) WMI_GET_BITS(mgmt_pkt_ctr_link_info, 16, 16)
#define WMI_MGMT_RX_REO_PARAM_MGMT_PKT_CTR_SET(mgmt_pkt_ctr_link_info, value) WMI_SET_BITS(mgmt_pkt_ctr_link_info, 16, 16, value)

/** Data structure of the TLV to add in RX EVENTID for providing REO params
 *  like global_timestamp and mgmt_pkt_ctr
 */
typedef struct {
    A_UINT32 tlv_header; /*TLV WMITLV_TAG_STRUC_wmi_mgmt_rx_reo_params*/
    /* Timestamp (in micro sec) of the last fw forwarded mgmt. frame, same across chips */
    A_UINT32 global_timestamp;
    /**
     * mgmt_pkt_ctr_link_info
     *
     * [11:0]:  reserved
     * [14:12]: ieee_link_id, protocol link id on which the rx frame is received
     * [15]:    mgmt_pkt_ctr_valid
     * [31:16]: mgmt_pkt_ctr, Sequence number of the last fw forwarded mgmt frame
     */

    A_UINT32 mgmt_pkt_ctr_link_info;
    A_UINT32 rx_ppdu_duration_us; /* receive duration in us */
    A_UINT32 mpdu_end_timestamp; /* mpdu end timestamp in us (based on HWMLO timer) */
} wmi_mgmt_rx_reo_params;

/** Helper macro for param GET/SET */
#define WMI_RX_PARAM_EXT_META_ID_GET(mgmt_rx_params_ext_dword0) WMI_GET_BITS(mgmt_rx_params_ext_dword0, 0, 3)
#define WMI_RX_PARAM_EXT_META_ID_SET(mgmt_rx_params_ext_dword0, value) WMI_SET_BITS(mgmt_rx_params_ext_dword0, 0, 3, value)

#define WMI_RX_PARAM_EXT_BA_WIN_SIZE_GET(mgmt_rx_params_ext_dword1) WMI_GET_BITS(mgmt_rx_params_ext_dword1, 0, 16)
#define WMI_RX_PARAM_EXT_BA_WIN_SIZE_SET(mgmt_rx_params_ext_dword1, value) WMI_SET_BITS(mgmt_rx_params_ext_dword1, 0, 16, value)

#define WMI_RX_PARAM_EXT_REO_WIN_SIZE_GET(mgmt_rx_params_ext_dword1) WMI_GET_BITS(mgmt_rx_params_ext_dword1, 16, 16)
#define WMI_RX_PARAM_EXT_REO_WIN_SIZE_SET(mgmt_rx_params_ext_dword1, value) WMI_SET_BITS(mgmt_rx_params_ext_dword1, 16, 16, value)

typedef enum {
    WMI_RX_PARAMS_EXT_META_ADDBA = 0x0,
    WMI_RX_PARAMS_EXT_META_TWT = 0x1,
} wmi_mgmt_rx_params_ext_meta_t;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag (WMITLV_TAG_STRUC_wmi_mgmt_rx_params_ext) and len */
    union {
        struct {
            A_UINT32
                /* Describes the representation of the data in rx_param_ext_dword1
                 * Full set shown in wmi_mgmt_rx_params_ext_meta_t */
                meta_id     : 3,
                /* Dedicated for commonly used parameters only */
                reserved_0  : 29;
        };
        A_UINT32 mgmt_rx_params_ext_dword0;
    };
    union {
        struct {
            /* WMI_RX_PARAMS_EXT_META_ADDBA */
            A_UINT32
                ba_win_size :16,  /* negotiated BA window size */
                reo_win_size :16; /* 2x the negotiated BA window size to handle any latency across MLO */
        };
        A_UINT32 mgmt_rx_params_ext_dword1;
    };
    union {
        struct {
            /* WMI_RX_PARAMS_EXT_META_TWT */
            A_UINT32 twt_ie_buf_len; /* IE length */
            /* Following this structure is the TLV byte stream of IE data
             * of length twt_ie_buf_len:
             *     A_UINT8 ie_data[]; <-- length in bytes given by field
             *                            twt_ie_buf_len.
             *     This ie_data[] would contain only the TWT IE information
             *     when twt_ie_buf_len is non zero.
             */
        };
        A_UINT32 mgmt_rx_params_ext_dword2;
    };
} wmi_mgmt_rx_params_ext;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_mgmt_rx_hdr */
    /** channel on which this frame is received (channel number) */
    A_UINT32 channel;
    /** snr information used to cal RSSI */
    A_UINT32 snr;
    /** Rate kbps */
    A_UINT32 rate;
    /** rx phy mode WLAN_PHY_MODE */
    A_UINT32 phy_mode;
    /** length of the frame */
    A_UINT32 buf_len;
    /** rx status */
    A_UINT32 status; /* capture mode indication */
    /** RSSI of PRI 20MHz for each chain. */
    A_UINT32 rssi_ctl[ATH_MAX_ANTENNA];
    /** information about the management frame e.g. can give a scan source for a scan result mgmt frame */
    A_UINT32 flags;
    /** combined RSSI, i.e. the sum of the snr + noise floor (dBm units) */
    A_INT32 rssi;
    /** delta between local TSF(TSF timestamp when frame was RXd)
     *  and remote TSF(TSF timestamp in the IE for mgmt frame -
     *  beacon,proberesp for e.g). If remote TSF is not available,
     *  delta set to 0.
     *  Although tsf_delta is stored as A_UINT32, it can be negative,
     *  and thus would need to be sign-extended if added to a value
     *  larger than 32 bits.
     */
    A_UINT32 tsf_delta;

    /* The lower 32 bits of the TSF (rx_tsf_l32) is copied by FW from
     * TSF timestamp in the RX MAC descriptor provided by HW.
     */
    A_UINT32 rx_tsf_l32;

    /* The Upper 32 bits (rx_tsf_u32) is filled by reading the TSF register
     * after the packet is received.
     */
    A_UINT32 rx_tsf_u32;

    /** pdev_id for identifying the MAC the rx mgmt frame was received by
     * See macros starting with WMI_PDEV_ID_ for values.
     */
    A_UINT32 pdev_id;

    /** freq in MHz of the channel on which this frame was received */
    A_UINT32 chan_freq;

/* This TLV is followed by array of bytes:
 *   A_UINT8 bufp[]; <-- management frame buffer
 */
/* This TLV is optionally followed by array of struct:
 *  wmi_rssi_ctl_ext rssi_ctl_ext;
 */
/*
 * This TLV is followed by struct:
 * wmi_mgmt_rx_reo_params reo_params;// MGMT rx REO params
 */
/*
 * This TLV is optionally followed by struct:
 * wmi_mgmt_rx_params_ext mgmt_rx_params_ext[0 or 1];
 */
} wmi_mgmt_rx_hdr;

/* WMI CMD to receive the management filter criteria from the host */
typedef struct {
    A_UINT32 tlv_header; /* WMITLV_TAG_STRUC_wmi_mgmt_reo_filter_cmd_fixed_param */
    A_UINT32 pdev_id; /* pdev_id for identifying the MAC */
    /* filter:
     * Each bit represents the possible combination of frame type (2 bits)
     * and subtype (4 bits)
     * There would be 64 such combinations as per the 802.11 standard
     * For Exp : We have beacon frame, we will take the type and subtype
     *           of this frame and concatenate the bits, it will give 6 bits
     *           number. We need to go to that bit position in the below
     *           2 filter_low and filter_high bitmap and set the bit.
     */
    A_UINT32 filter_low;
    A_UINT32 filter_high;
} wmi_mgmt_rx_reo_filter_configuration_cmd_fixed_param;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag (WMITLV_TAG_STRUC_wmi_frame_pn_params) and len */
    A_UINT8 cur_pn[WMI_MAX_PN_LEN];
    A_UINT8 prev_pn[WMI_MAX_PN_LEN];
} wmi_frame_pn_params;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag (WMITLV_TAG_STRUC_wmi_is_my_frame) */
    A_UINT32 mgmt_frm_sub_type; /* to indicate which sub-type of MGMT frame */
    A_UINT32 is_my_frame; /* to indicate frame is sent to this BSSID */
} wmi_is_my_mgmt_frame;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_mgmt_ml_info */
    /**
     * CU vdev map to initmate about the on-going Critical update
     * per-link contains 16 VAPs at max.
     */

    /*
     *  bits    : 0-15 | 16-31
     *  link-id :  0   |   1
     */
    A_UINT32 cu_vdev_map_1;
    /*
     *  bits    : 0-15 | 16-31
     *  link-id :  2   |   3
     */
    A_UINT32 cu_vdev_map_2;
    /*
     *  bits    : 0-15 | 16-31
     *  link-id :  4   |   5
     */
    A_UINT32 cu_vdev_map_3;
    /*
     *  bits    : 0-15 | 16-31
     *  link-id :  6   |   7
     */
    A_UINT32 cu_vdev_map_4; /* bits 63:32 */
    /**
     * This is followed by byte array that contains BPCC value per MLO VAP.
     * There will be 16 byte entries for each link corresponding to VAP-ID.
     * So number of byte entries will be (num of max links supported by AP * 16)
     * Note: num of max links supported = 8
     */
} wmi_mgmt_ml_info;

#define WMI_MLO_BROADCAST_TID_TO_LINK_MAP_INFO_VDEV_ID_GET(_var)       WMI_GET_BITS(_var, 0, 8)
#define WMI_MLO_BROADCAST_TID_TO_LINK_MAP_INFO_VDEV_ID_SET(_var, _val) WMI_SET_BITS(_var, 0, 8, _val)

#define WMI_MLO_BROADCAST_TID_TO_LINK_MAP_INFO_EXP_DUR_GET(_var)       WMI_GET_BITS(_var, 8, 24)
#define WMI_MLO_BROADCAST_TID_TO_LINK_MAP_INFO_EXP_DUR_SET(_var, _val) WMI_SET_BITS(_var, 8, 24, _val)

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_mlo_bcast_t2lm_info */
    /*
     * Vdev_id for MLO vap
     * WMI_MLO_BROADCAST_TID_TO_LINK_MAP_INFO_VDEV_ID_GET /
     * WMI_MLO_BROADCAST_TID_TO_LINK_MAP_INFO_VDEV_ID_SET
     * vdev_id :8
     *
     * Duration time for MLO Vap
     * WMI_MLO_BROADCAST_TID_TO_LINK_MAP_INFO_VDEV_ID_GET /
     * WMI_MLO_BROADCAST_TID_TO_LINK_MAP_INFO_VDEV_ID_SET
     * expected_duration :24
     */
    A_UINT32 vdev_id_expec_dur;
} wmi_mlo_bcast_t2lm_info;

typedef enum {
    PKT_CAPTURE_MODE_DISABLE = 0,
    PKT_CAPTURE_MODE_MGMT_ONLY,
    PKT_CAPTURE_MODE_DATA_ONLY,
    PKT_CAPTURE_MODE_DATA_MGMT,
} WMI_PKT_CAPTURE_MODE_CONFIG;

/* This information sending to host during offloaded MGMT local TX and host TX */
typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_mgmt_hdr */
    /* channel frequency in MHz */
    A_UINT32 chan_freq;
    /** snr information used to cal RSSI in dB */
    A_UINT32 snr;
    /** Rate kbps */
    A_UINT32 rate_kbps;
    /** phy mode WLAN_PHY_MODE */
    A_UINT32 phy_mode;
    /** length of the frame in bytes */
    A_UINT32 buf_len;
    /** status:
     * 0x00: CRC ERR
     * 0x08: DECRYPT ERR
     * 0x10: MIC ERR
     * 0x20: KEY CACHE MISS
     */
    A_UINT32 status;
    /** flags:
     * Information about the management frame e.g. can give a scan source
     * for a scan result mgmt frame
     * Refer to WMI_MGMT_RX_HDR_ definitions.
     * ex: WMI_MGMT_RX_HDR_EXTSCAN,WMI_MGMT_RX_HDR_ENLO
     */
    A_UINT32 flags;
    /** combined RSSI, i.e. the sum of the snr + noise floor (dBm units) */
    A_INT32 rssi;
    /** delta between local TSF (TSF timestamp when frame was RXd)
     *  and remote TSF (TSF timestamp in the IE for mgmt frame -
     *  beacon, proberesp for example). If remote TSF is not available,
     *  delta is set to 0.
     *  Although tsf_delta is stored as A_UINT32, it can be negative,
     *  and thus would need to be sign-extended if added to a value
     *  larger than 32 bits.
     */
    A_UINT32 tsf_delta;

    /* The lower 32 bits of the TSF (tsf_l32) is copied by FW from
     * TSF timestamp in the TX MAC descriptor provided by HW.
     */
    A_UINT32 tsf_l32;

    /* The upper 32 bits of the TSF (tsf_u32) is copied by FW from
     * TSF timestamp in the TX MAC descriptor provided by HW.
     */
    A_UINT32 tsf_u32;

    /** pdev_id for identifying the MAC the tx mgmt frame transmitted.
     * See macros starting with WMI_PDEV_ID_ for values.
     */
    A_UINT32 pdev_id;

    A_UINT32 direction; /* tx:0,rx:1*/

    /** tx_status:
     * 0: xmit ok
     * 1: excessive retries
     * 2: blocked by tx filtering
     * 4: fifo underrun
     * 8: swabort
     */
    A_UINT32 tx_status;

    A_UINT32
        /* tx_retry_cnt:
         * Indicates retry count of offloaded/local & host mgmt tx frames.
         * The WMI_MGMT_HDR_TX_RETRY_[SET,GET] macros can be used to access
         * this bitfield in a portable manner.
         */
        tx_retry_cnt:6, /* [5:0] */
        reserved_1:26;  /* [31:6] */

/* This TLV may be followed by array of bytes:
 *   A_UINT8 bufp[]; <-- management frame buffer
 */
} wmi_mgmt_hdr;

/* Tx retry cnt set & get bits*/
#define WMI_MGMT_HDR_TX_RETRY_CNT_SET(tx_retry_cnt, value) \
    WMI_SET_BITS(tx_retry_cnt, 0, 6, value)
#define WMI_MGMT_HDR_TX_RETRY_CNT_GET(tx_retry_cnt) \
    WMI_GET_BITS(tx_retry_cnt, 0, 6)

/*
 * Instead of universally increasing the RX_HDR_HEADROOM size which may cause problems for older targets,
 * this new ext_hdr can be used for extending the header and will be only applicable for new targets.
 */
typedef struct
{
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_rssi_ctl_ext */
    /** RSSI of PRI 20MHz for each chain, in dB w.r.t. noise floor */
    A_UINT32 rssi_ctl_ext[MAX_ANTENNA_EIGHT - ATH_MAX_ANTENNA];
} wmi_rssi_ctl_ext;

typedef struct {
    /** TSF timestamp */
    A_UINT32 tsf_timestamp;

    /**
     * Current freq1, freq2
     *
     * [7:0]:    freq1[lo]
     * [15:8] :   freq1[hi]
     * [23:16]:   freq2[lo]
     * [31:24]:   freq2[hi]
     */
    A_UINT32 freq_info_1;

    /**
     * Combined RSSI over all chains and channel width for this PHY error
     *
     * [7:0]: RSSI combined
     * [15:8]: Channel width (MHz)
     * [23:16]: PHY error code
     * [24:16]: reserved (future use)
     */
    A_UINT32 freq_info_2;

    /**
     * RSSI on chain 0 through 3
     *
     * This is formatted the same as the PPDU_START RX descriptor
     * field:
     *
     * [7:0]:   pri20
     * [15:8]:  sec20
     * [23:16]: sec40
     * [31:24]: sec80
     */
    A_UINT32 rssi_chain0;
    A_UINT32 rssi_chain1;
    A_UINT32 rssi_chain2;
    A_UINT32 rssi_chain3;

    /**
     * Last calibrated NF value for chain 0 through 3
     *
     * nf_list_1:
     *
     * + [15:0] - chain 0
     * + [31:16] - chain 1
     *
     * nf_list_2:
     *
     * + [15:0] - chain 2
     * + [31:16] - chain 3
     */
    A_UINT32 nf_list_1;
    A_UINT32 nf_list_2;

    /** Length of the frame */
    A_UINT32 buf_len;
} wmi_single_phyerr_rx_hdr;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_single_phyerr_ext_rx_hdr */
    /**
     * RSSI on chain 4 through 7 in dB w.r.t noise floor.
     *
     * This is formatted the same as the PPDU_START RX descriptor
     * field:
     *
     * [7:0]:   pri20
     * [15:8]:  sec20
     * [23:16]: sec40
     * [31:24]: sec80
     */
    A_UINT32 rssi_chain4;
    A_UINT32 rssi_chain5;
    A_UINT32 rssi_chain6;
    A_UINT32 rssi_chain7;
    /**
     * Last calibrated NF value for chain 4 through 7 in dBm
     *
     * nf_list_3:
     * + [15:0] - chain 4
     * + [31:16] - chain 5
     *
     * nf_list_4:
     * + [15:0] - chain 6
     * + [31:16] - chain 7
     *
     * Each chain's noise floor is stored as a sign-extended (negative)
     * value in dBm units.
     */
    A_UINT32 nf_list_3;
    A_UINT32 nf_list_4;
} wmi_single_phyerr_ext_rx_hdr;

#define WMI_UNIFIED_FREQINFO_1_LO   0x000000ff
#define WMI_UNIFIED_FREQINFO_1_LO_S 0
#define WMI_UNIFIED_FREQINFO_1_HI   0x0000ff00
#define WMI_UNIFIED_FREQINFO_1_HI_S 8
#define WMI_UNIFIED_FREQINFO_2_LO   0x00ff0000
#define WMI_UNIFIED_FREQINFO_2_LO_S 16
#define WMI_UNIFIED_FREQINFO_2_HI   0xff000000
#define WMI_UNIFIED_FREQINFO_2_HI_S 24

/*
 * Please keep in mind that these _SET macros break macro side effect
 * assumptions; don't be clever with them.
 */
#define WMI_UNIFIED_FREQ_INFO_GET(hdr, f)                                   \
            (WMI_F_MS((hdr)->freq_info_1,                                   \
              WMI_UNIFIED_FREQINFO_##f##_LO)                                \
              | (WMI_F_MS((hdr)->freq_info_1,                               \
                 WMI_UNIFIED_FREQINFO_##f##_HI) << 8))

#define WMI_UNIFIED_FREQ_INFO_SET(hdr, f, v)                                \
        do {                                                                \
            WMI_F_RMW((hdr)->freq_info_1, (v) & 0xff,                       \
                WMI_UNIFIED_FREQINFO_##f##_LO);                             \
            WMI_F_RMW((hdr)->freq_info_1, ((v) >> 8) & 0xff,                \
                WMI_UNIFIED_FREQINFO_##f##_HI);                             \
        } while (0)

#define WMI_UNIFIED_FREQINFO_2_RSSI_COMB    0x000000ff
#define WMI_UNIFIED_FREQINFO_2_RSSI_COMB_S  0
#define WMI_UNIFIED_FREQINFO_2_CHWIDTH      0x0000ff00
#define WMI_UNIFIED_FREQINFO_2_CHWIDTH_S    8
#define WMI_UNIFIED_FREQINFO_2_PHYERRCODE   0x00ff0000
#define WMI_UNIFIED_FREQINFO_2_PHYERRCODE_S 16

#define WMI_UNIFIED_RSSI_COMB_GET(hdr)                                      \
            ((int8_t) (WMI_F_MS((hdr)->freq_info_2,                         \
                WMI_UNIFIED_FREQINFO_2_RSSI_COMB)))

#define WMI_UNIFIED_RSSI_COMB_SET(hdr, v)                                   \
            WMI_F_RMW((hdr)->freq_info_2, (v) & 0xff,                       \
              WMI_UNIFIED_FREQINFO_2_RSSI_COMB);

#define WMI_UNIFIED_CHWIDTH_GET(hdr)                                        \
            WMI_F_MS((hdr)->freq_info_2, WMI_UNIFIED_FREQINFO_2_CHWIDTH)

#define WMI_UNIFIED_CHWIDTH_SET(hdr, v)                                     \
            WMI_F_RMW((hdr)->freq_info_2, (v) & 0xff,                       \
              WMI_UNIFIED_FREQINFO_2_CHWIDTH);

#define WMI_UNIFIED_PHYERRCODE_GET(hdr)                                     \
            WMI_F_MS((hdr)->freq_info_2, WMI_UNIFIED_FREQINFO_2_PHYERRCODE)

#define WMI_UNIFIED_PHYERRCODE_SET(hdr, v)                                  \
            WMI_F_RMW((hdr)->freq_info_2, (v) & 0xff,                       \
              WMI_UNIFIED_FREQINFO_2_PHYERRCODE);

#define WMI_UNIFIED_CHAIN_0     0x0000ffff
#define WMI_UNIFIED_CHAIN_0_S   0
#define WMI_UNIFIED_CHAIN_1     0xffff0000
#define WMI_UNIFIED_CHAIN_1_S   16
#define WMI_UNIFIED_CHAIN_2     0x0000ffff
#define WMI_UNIFIED_CHAIN_2_S   0
#define WMI_UNIFIED_CHAIN_3     0xffff0000
#define WMI_UNIFIED_CHAIN_3_S   16

#define WMI_UNIFIED_CHAIN_4     0x0000ffff
#define WMI_UNIFIED_CHAIN_4_S   0
#define WMI_UNIFIED_CHAIN_5     0xffff0000
#define WMI_UNIFIED_CHAIN_5_S   16
#define WMI_UNIFIED_CHAIN_6     0x0000ffff
#define WMI_UNIFIED_CHAIN_6_S   0
#define WMI_UNIFIED_CHAIN_7     0xffff0000
#define WMI_UNIFIED_CHAIN_7_S   16

#define WMI_UNIFIED_CHAIN_0_FIELD   nf_list_1
#define WMI_UNIFIED_CHAIN_1_FIELD   nf_list_1
#define WMI_UNIFIED_CHAIN_2_FIELD   nf_list_2
#define WMI_UNIFIED_CHAIN_3_FIELD   nf_list_2
#define WMI_UNIFIED_CHAIN_4_FIELD   nf_list_3
#define WMI_UNIFIED_CHAIN_5_FIELD   nf_list_3
#define WMI_UNIFIED_CHAIN_6_FIELD   nf_list_4
#define WMI_UNIFIED_CHAIN_7_FIELD   nf_list_4

#define WMI_UNIFIED_NF_CHAIN_GET(hdr, c)                                    \
            ((int16_t) (WMI_F_MS((hdr)->WMI_UNIFIED_CHAIN_##c##_FIELD,      \
              WMI_UNIFIED_CHAIN_##c)))

#define WMI_UNIFIED_NF_CHAIN_SET(hdr, c, nf)                                \
            WMI_F_RMW((hdr)->WMI_UNIFIED_CHAIN_##c##_FIELD, (nf) & 0xffff,  \
              WMI_UNIFIED_CHAIN_##c);

/*
 * For now, this matches what the underlying hardware is doing.
 * Update ar6000ProcRxDesc() to use these macros when populating
 * the rx descriptor and then we can just copy the field over
 * to the WMI PHY notification without worrying about breaking
 * things.
 */
#define WMI_UNIFIED_RSSI_CHAN_PRI20     0x000000ff
#define WMI_UNIFIED_RSSI_CHAN_PRI20_S   0
#define WMI_UNIFIED_RSSI_CHAN_SEC20     0x0000ff00
#define WMI_UNIFIED_RSSI_CHAN_SEC20_S   8
#define WMI_UNIFIED_RSSI_CHAN_SEC40     0x00ff0000
#define WMI_UNIFIED_RSSI_CHAN_SEC40_S   16
#define WMI_UNIFIED_RSSI_CHAN_SEC80     0xff000000
#define WMI_UNIFIED_RSSI_CHAN_SEC80_S   24

#define WMI_UNIFIED_RSSI_CHAN_SET(hdr, c, ch, rssi)                         \
            WMI_F_RMW((hdr)->rssi_chain##c, (rssi) & 0xff,                  \
              WMI_UNIFIED_RSSI_CHAN_##ch);

#define WMI_UNIFIED_RSSI_CHAN_GET(hdr, c, ch)                               \
            ((int8_t) (WMI_F_MS((hdr)->rssi_chain##c,                       \
              WMI_UNIFIED_RSSI_CHAN_##ch)))

#define WMI_UNIFIED_CHAIN_RSSI_GET(tlv, chain_idx, band) \
    ((A_INT8) WMI_F_MS((tlv)->chain_rssi[chain_idx], WMI_UNIFIED_RSSI_CHAN_ ## band))

typedef struct {
    /** Phy error event header */
    wmi_single_phyerr_rx_hdr hdr;
    /** frame buffer */
    A_UINT8 bufp[1];
} wmi_single_phyerr_rx_event;

/* PHY ERROR MASK 0 */
/* bits 1:0 defined but not published */
#define WMI_PHY_ERROR_MASK0_RADAR                           (1 <<  2)
/* bits 23:3 defined but not published */
#define WMI_PHY_ERROR_MASK0_FALSE_RADAR_EXT                 (1 << 24)
/* bits 25:24 defined but not published */
#define WMI_PHY_ERROR_MASK0_SPECTRAL_SCAN                   (1 << 26)
/* bits 31:27 defined but not published */

/* PHY ERROR MASK 1 */
/* bits 13:0 defined but not published */
/* bits 31:14 reserved */

/* PHY ERROR MASK 2 */
/* bits 31:0 reserved */

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_comb_phyerr_rx_hdr */
    /** Phy error phy error count */
    A_UINT32 num_phyerr_events;
    A_UINT32 tsf_l32;
    A_UINT32 tsf_u32;
    A_UINT32 buf_len;
    union {
        A_UINT32 pmac_id; /* OBSOLETE - will be removed once all refs are gone */
        /** pdev_id for identifying the MAC
         * See macros starting with WMI_PDEV_ID_ for values.
         */
        A_UINT32 pdev_id;
    };
    A_UINT32 rsPhyErrMask0; /* see WMI_PHY_ERROR_MASK0 */
    A_UINT32 rsPhyErrMask1; /* see WMI_PHY_ERROR_MASK1 */
    A_UINT32 rsPhyErrMask2; /* see WMI_PHY_ERROR_MASK2 */
/* This TLV is followed by array of bytes:
 *     frame buffer - contains multiple payloads in the order:
 *         header - payload, header - payload...
 *     (The header is of type: wmi_single_phyerr_rx_hdr)
 *   A_UINT8 bufp[];
 *     The extension hdr will repeat num_phyerr_events of times
 *     and will have 1:1 mapping with above header. i.e the 1st
 *     ext_rx_hdr will belong to 1st phyerr_rx_hdr and so on.
 *   wmi_single_phyerr_ext_rx_hdr single_phyerr_ext;
 */
} wmi_comb_phyerr_rx_hdr;

/* WMI MGMT TX  */
typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_mgmt_tx_hdr */
    /** unique id identifying the VDEV, generated by the caller */
    A_UINT32 vdev_id;
    /** peer MAC address */
    wmi_mac_addr peer_macaddr;
    /** xmit rate */
    A_UINT32 tx_rate;
    /** xmit power */
    A_UINT32 tx_power;
    /** Buffer length in bytes */
    A_UINT32 buf_len;
/* This TLV is followed by array of bytes:
 *   A_UINT8 bufp[]; <-- management frame buffer
 */
} wmi_mgmt_tx_hdr;

#define WMI_TX_SEND_PARAM_EXT_META_ID_GET(tx_param_ext_dword0) WMI_GET_BITS(tx_param_dword0, 0, 3)
#define WMI_TX_SEND_PARAM_EXT_META_ID_SET(tx_param_ext_dword0, value) WMI_SET_BITS(tx_param_dword0, 0, 3, value)

#define WMI_TX_SEND_PARAM_EXT_WIN_SIZE_GET(tx_param_ext_dword1) WMI_GET_BITS(tx_param_dword1, 0, 16)
#define WMI_TX_SEND_PARAM_EXT_WIN_SIZE_SET(tx_param_ext_dword1, value) WMI_SET_BITS(tx_param_dword1, 0, 16, value)

typedef enum {
    WMI_TX_SEND_PARAMS_EXT_META_ADDBA = 0x0,
    WMI_TX_SEND_PARAMS_EXT_META_DELBA = 0x1,
} wmi_tx_send_params_ext_meta_t;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag (WMITLV_TAG_STRUC_wmi_tx_send_params_ext) and len */
    union {
        struct {
            A_UINT32 meta_id     : 3,  /* Describes the representation of the data in tx_param_ext_dword1 Full set shown in wmi_tx_send_params_ext_meta_t */
                     reserved_0  : 29; /* Dedicated for commonly used parameters only */
        };
        A_UINT32 tx_param_ext_dword0;
    };
    union {
        struct {
        /* WMI_TX_SEND_PARAMS_EXT_META_ADDBA */
            A_UINT32 win_size    : 16,
                     reserved_1  : 16;
        };
        A_UINT32 tx_param_ext_dword1;
    };
} wmi_tx_send_params_ext;

#define WMI_TX_SEND_PARAM_PWR_GET(tx_param_dword0) WMI_GET_BITS(tx_param_dword0, 0, 8)
#define WMI_TX_SEND_PARAM_PWR_SET(tx_param_dword0, value) WMI_SET_BITS(tx_param_dword0, 0, 8, value)

#define WMI_TX_SEND_PARAM_MCS_MASK_GET(tx_param_dword0) WMI_GET_BITS(tx_param_dword0, 8, 12)
#define WMI_TX_SEND_PARAM_MCS_MASK_SET(tx_param_dword0, value) WMI_SET_BITS(tx_param_dword0, 8, 12, value)

#define WMI_TX_SEND_PARAM_NSS_MASK_GET(tx_param_dword0) WMI_GET_BITS(tx_param_dword0, 20, 8)
#define WMI_TX_SEND_PARAM_NSS_MASK_SET(tx_param_dword0, value) WMI_SET_BITS(tx_param_dword0, 20, 8, value)

#define WMI_TX_SEND_PARAM_RETRY_LIMIT_GET(tx_param_dword0) WMI_GET_BITS(tx_param_dword0, 28, 4)
#define WMI_TX_SEND_PARAM_RETRY_LIMIT_SET(tx_param_dword0, value) WMI_SET_BITS(tx_param_dword0, 28, 4, value)

#define WMI_TX_SEND_PARAM_CHAIN_MASK_GET(tx_param_dword1) WMI_GET_BITS(tx_param_dword1, 0, 8)
#define WMI_TX_SEND_PARAM_CHAIN_MASK_SET(tx_param_dword1, value) WMI_SET_BITS(tx_param_dword1, 0, 8, value)

#define WMI_TX_SEND_PARAM_BW_MASK_GET(tx_param_dword1) WMI_GET_BITS(tx_param_dword1, 8, 7)
#define WMI_TX_SEND_PARAM_BW_MASK_SET(tx_param_dword1, value) WMI_SET_BITS(tx_param_dword1, 8, 7, value)

#define WMI_TX_SEND_PARAM_PREAMBLE_GET(tx_param_dword1) WMI_GET_BITS(tx_param_dword1, 15, 5)
#define WMI_TX_SEND_PARAM_PREAMBLE_SET(tx_param_dword1, value) WMI_SET_BITS(tx_param_dword1, 15, 5, value)

#define WMI_TX_SEND_PARAM_FRAME_TYPE_GET(tx_param_dword1) WMI_GET_BITS(tx_param_dword1, 20, 1)
#define WMI_TX_SEND_PARAM_FRAME_TYPE_SET(tx_param_dword1, value) WMI_SET_BITS(tx_param_dword1, 20, 1, value)

#define WMI_TX_SEND_PARAM_CFR_CAPTURE_GET(tx_param_dword1) WMI_GET_BITS(tx_param_dword1, 21, 1)
#define WMI_TX_SEND_PARAM_CFR_CAPTURE_SET(tx_param_dword1, value) WMI_SET_BITS(tx_param_dword1, 21, 1, value)

#define WMI_TX_SEND_PARAM_BEAMFORM_GET(tx_param_dword1) WMI_GET_BITS(tx_param_dword1, 22, 1)
#define WMI_TX_SEND_PARAM_BEAMFORM_SET(tx_param_dword1, value) WMI_SET_BITS(tx_param_dword1, 22, 1, value)

#define WMI_TX_SEND_PARAM_RETRY_LIMIT_EXT_GET(tx_param_dword1) WMI_GET_BITS(tx_param_dword1, 23, 3)
#define WMI_TX_SEND_PARAM_RETRY_LIMIT_EXT_SET(tx_param_dword1, value) WMI_SET_BITS(tx_param_dword1, 23, 3, value)


/* TX_SEND flags:
 * Bit 0: set wrong txkey
 *     There is one special WFA test case in STA or AP, setting wrong txkey
 *     in disassoc or deauth with PMF enabled to verify if peer disconnected
 */
#define WMI_TX_SEND_FLAG_SET_WRONG_KEY    0x00000001
#define WMI_TX_SEND_FLAG_SET_WRONG_KEY_GET(tx_flags) WMI_GET_BITS(tx_flags, 0, 1)
#define WMI_TX_SEND_FLAG_SET_WRONG_KEY_SET(tx_flags, value) WMI_SET_BITS(tx_flags, 0, 1, value)

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_tx_send_params */

    union {
        struct {
            /* DWORD 0: tx power, tx rate, retry_limit */
            A_UINT32
                /* pwr -
                 * Specify what power the tx frame needs to be transmitted at.
                 * The power a signed (two's complement) value is in units of 0.5 dBm.
                 * The value needs to be appropriately sign-extended when extracting
                 * the value from the message and storing it in a variable that is
                 * larger than A_INT8.  (fw automatically handles this sign-extension.)
                 * If the transmission uses multiple tx chains, this power spec is
                 * the total transmit power, assuming incoherent combination of
                 * per-chain power to produce the total power.
                 */
                 pwr: 8,

                /* mcs_mask -
                 * Specify the allowable values for MCS index (modulation and coding)
                 * to use for transmitting the frame.
                 *
                 * For HT / VHT preamble types, this mask directly corresponds to
                 * the HT or VHT MCS indices that are allowed.  For each bit N set
                 * within the mask, MCS index N is allowed for transmitting the frame.
                 * For legacy CCK and OFDM rates, separate bits are provided for CCK
                 * rates versus OFDM rates, so the host has the option of specifying
                 * that the target must transmit the frame with CCK or OFDM rates
                 * (not HT or VHT), but leaving the decision to the target whether
                 * to use CCK or OFDM.
                 *
                 * For CCK and OFDM, the bits within this mask are interpreted as
                 * follows:
                 *     bit  0 -> CCK 1 Mbps rate is allowed
                 *     bit  1 -> CCK 2 Mbps rate is allowed
                 *     bit  2 -> CCK 5.5 Mbps rate is allowed
                 *     bit  3 -> CCK 11 Mbps rate is allowed
                 *     bit  4 -> OFDM BPSK modulation, 1/2 coding rate is allowed
                 *     bit  5 -> OFDM BPSK modulation, 3/4 coding rate is allowed
                 *     bit  6 -> OFDM QPSK modulation, 1/2 coding rate is allowed
                 *     bit  7 -> OFDM QPSK modulation, 3/4 coding rate is allowed
                 *     bit  8 -> OFDM 16-QAM modulation, 1/2 coding rate is allowed
                 *     bit  9 -> OFDM 16-QAM modulation, 3/4 coding rate is allowed
                 *     bit 10 -> OFDM 64-QAM modulation, 2/3 coding rate is allowed
                 *     bit 11 -> OFDM 64-QAM modulation, 3/4 coding rate is allowed
                 *
                 * The MCS index specification needs to be compatible with the
                 * bandwidth mask specification.  For example, a MCS index == 9
                 * specification is inconsistent with a preamble type == VHT,
                 * Nss == 1, and channel bandwidth == 20 MHz.
                 *
                 * Furthermore, the host has only a limited ability to specify to
                 * the target to select from HT + legacy rates, or VHT + legacy rates,
                 * since this mcs_mask can specify either HT/VHT rates or legacy rates.
                 * If no bits are set, target can choose what MCS type to use.
                 */
                 mcs_mask: 12,

                /* nss_mask -
                 * Specify which numbers of spatial streams (MIMO factor) are permitted.
                 * Each bit in this mask corresponds to a Nss value:
                 *     bit 0: if set, Nss = 1 (non-MIMO) is permitted
                 *     bit 1: if set, Nss = 2 (2x2 MIMO) is permitted
                 *     bit 2: if set, Nss = 3 (3x3 MIMO) is permitted
                 *     bit 3: if set, Nss = 4 (4x4 MIMO) is permitted
                 *     bit 4: if set, Nss = 5 (5x5 MIMO) is permitted
                 *     bit 5: if set, Nss = 6 (6x6 MIMO) is permitted
                 *     bit 6: if set, Nss = 7 (7x7 MIMO) is permitted
                 *     bit 7: if set, Nss = 8 (8x8 MIMO) is permitted
                 * The values in the Nss mask must be suitable for the recipient, e.g.
                 * a value of 0x4 (Nss = 3) cannot be specified for a tx frame to a
                 * recipient which only supports 2x2 MIMO.
                 * If no bits are set, target can choose what NSS type to use.
                 */
                 nss_mask: 8,

                /* retry_limit -
                 * Specify the maximum number of transmissions, including the
                 * initial transmission, to attempt before giving up if no ack
                 * is received.
                 * If the tx rate is specified, then all retries shall use the
                 * same rate as the initial transmission.
                 * If no tx rate is specified, the target can choose whether to
                 * retain the original rate during the retransmissions, or to
                 * fall back to a more robust rate.
                 */
                 retry_limit: 4;

       };
       A_UINT32 tx_param_dword0;
    };

    union {
        struct {
            /* DWORD 1: tx chain mask, preamble_type, tx BW */
            A_UINT32
                /* chain_mask - specify which chains to transmit from
                 * If not set, target will choose what chain_mask to use.
                 */
                chain_mask: 8,

                /* The bits in this mask correspond to the values as below
                 *     bit  0 -> 5MHz
                 *     bit  1 -> 10MHz
                 *     bit  2 -> 20MHz
                 *     bit  3 -> 40MHz
                 *     bit  4 -> 80MHz
                 *     bit  5 -> 160MHz
                 *     bit  6 -> 80_80MHz
                 * If no bits are set, target can choose what BW to use.
                 */
                bw_mask: 7,

                /* preamble_type_mask -
                 * Specify which preamble types (CCK, OFDM, HT, VHT) the target
                 * may choose from for transmitting this frame.
                 * Each bit in this mask corresponds to a preamble_type value:
                 *     bit 0: if set, OFDM
                 *     bit 1: if set, CCK
                 *     bit 2: if set, HT
                 *     bit 3: if set, VHT
                 *     bit 4: if set, HE
                 * If no bits are set, target can choose what preamble type to use.
                 */
                preamble_type: 5,

                /* Data:1 Mgmt:0 */
                frame_type: 1,

                /* Capture CFR when bit is set */
                cfr_capture: 1,

                /* Enables Beamforming when bit is set */
                en_beamforming: 1,

                /*
                 * Extra 3 bits of retry limit defined in tx_param_dword0,
                 * to allow maximum 127 retries for specific frames.
                 */
                retry_limit_ext: 3,

                reserved1_31_26: 6;
        };
        A_UINT32 tx_param_dword1;
    };
} wmi_tx_send_params;

#define WMI_MLO_MGMT_TID 0xFFFFFFFF

typedef struct {
    A_UINT32 tlv_header; /* TLV tag (WMITLV_TAG_STRUC_wmi_mlo_tx_send_params) and len */
    A_UINT32 hw_link_id; /** Unique link id across SOCs, provided by QMI handshake.
                           * If WMI_MLO_MGMT_TID then the frame will be queued in the MLO queue
                           * If valid hw_link_id
                           */
} wmi_mlo_tx_send_params;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_mgmt_tx_send_cmd_fixed_param */
    A_UINT32 vdev_id;
    A_UINT32 desc_id;  /* echoed in tx_compl_event */
    A_UINT32 chanfreq; /* MHz units */
    /* WMI_MGMT_TX_SEND_CMDID is used for both pass by value and
     * pass by reference WMI management frames.
     *
     * a) If the command is for pass by reference,
     *    paddr_lo and padd_hi will hold the address of remote/host buffer
     * b) If the command is for pass by value,
     *    paddr_lo and paddr_hi will be NULL.
     */
    A_UINT32 paddr_lo;
    A_UINT32 paddr_hi;
    A_UINT32 frame_len;
    A_UINT32 buf_len;  /** Buffer length in bytes */
    /*
     * The frame which will have tx_params_valid set will be always be RAW
     * frame, as it will be tx'ed on non-pause tid
     */
    A_UINT32 tx_params_valid;
    /* tx_flags:
     * Extra flags when tx_params_valid is 0.
     * Refer to WMI_TX_SEND_FLAG_xxx defs regarding the meaning of the
     * bits within this field.
     */
    A_UINT32 tx_flags;
    /* peer_rssi:
     * If non-zero, indicates saved peer beacon/probe resp RSSI (dBm units)
     * ONLY for init connection auth/assoc pkt.
     */
    A_INT32 peer_rssi;


/* This TLV is followed by array of bytes: First 64 bytes of management frame
 *   A_UINT8 bufp[];
 */
/* This TLV is followed by wmi_tx_send_params
 * wmi_tx_send_params tx_send_params;
 * wmi_mlo_tx_send_params mlo_tx_send_params[];
 *     Note: WMI_MLO_MGMT_TID path validated for specific scenario
 *     (BTM Usecase). Full support is not available.
 * wmi_tx_send_params_ext tx_send_params_ext[0 or 1];
 */
} wmi_mgmt_tx_send_cmd_fixed_param;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_offchan_data_tx_send_cmd_fixed_param */
    A_UINT32 vdev_id;
    A_UINT32 desc_id;  /* echoed in tx_compl_event */
    A_UINT32 chanfreq; /* MHz units */
    A_UINT32 paddr_lo;
    A_UINT32 paddr_hi;
    A_UINT32 frame_len;
    A_UINT32 buf_len;  /** Buffer length in bytes */
    /* The frame which will have tx_params_valid set will be always be RAW
     * frame, as it will be tx'ed on non-pause tid
     */
    A_UINT32 tx_params_valid;

/* This TLV is followed by array of bytes: First 64 bytes of frame
 *   A_UINT8 bufp[];
 */
/* This TLV is followed by wmi_tx_send_params
 * wmi_tx_send_params tx_send_params;
 */
} wmi_offchan_data_tx_send_cmd_fixed_param;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_qos_null_frame_tx_send_cmd_fixed_param */
    A_UINT32 vdev_id;
    A_UINT32 desc_id;  /* echoed in tx_compl_event */
    A_UINT32 paddr_lo; /* paddr_lo and padd_hi will hold the address of remote/host buffer, which is physical address of frame */
    A_UINT32 paddr_hi;
    A_UINT32 frame_len; /* Actual length of frame in bytes*/
    A_UINT32 buf_len;  /** Buffer length in bytes, length of data DMA'ed to FW from host */

/* This fixed_param TLV is followed by the TLVs listed below:
 * 1.  ARRAY_BYTE TLV: First buf_len (expected to be 64) bytes of frame
 *     A_UINT8 bufp[];
 * 2.  wmi_tx_send_params tx_send_params;
 */
} wmi_qos_null_frame_tx_send_cmd_fixed_param;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_echo_event_fixed_param */
    A_UINT32 value;
} wmi_echo_event_fixed_param;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_echo_cmd_fixed_param */
    A_UINT32 value;
} wmi_echo_cmd_fixed_param;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag would be equivalent to WMITLV_TAG_STRUC_wmi_mlo_link_disable_request_event_fixed_param  */
    /** AP MLD address request to be disabled some set of link */
    wmi_mac_addr mld_addr;
    /** Request link id set to disable */
    A_UINT32 linkid_bitmap;
} wmi_mlo_link_disable_request_event_fixed_param;

typedef enum {
    /**
     * Projects support to offload regulatory database by default.
     * If don`t offload regulatory database, host can set this bit.
     */
    WMI_REGDOMAIN_DATABASE_NO_OFFLOAD_BITMASK = 0x00000001,
} WMI_REGDOMAIN_BITMASK;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_pdev_set_regdomain_cmd_fixed_param */

    /** pdev_id for identifying the MAC
     * See macros starting with WMI_PDEV_ID_ for values.
     */
    A_UINT32 pdev_id;
    /** reg domain code */
    A_UINT32 reg_domain;
    A_UINT32 reg_domain_2G; /* fulfil 2G domain ID */
    A_UINT32 reg_domain_5G; /* fulfil 5G domain ID */
    A_UINT32 conformance_test_limit_2G; /* 2G whole band CTL index */
    A_UINT32 conformance_test_limit_5G; /* 5G whole band CTL index */
    A_UINT32 dfs_domain;

    /**
     * The below conformance_test_limit index fields are for supporting the
     * 5G sub-band CTL feature.
     * Conformance test limits (CTLs) are the product-specific
     * regulatory-compliant powers stored in the board data file (BDF).
     * These CTLs within the BDF are identified by CTL index values.
     * For example, the BDF file is expected to contain CTL data for
     * FCC (CTL index = 0x10), ETSI (CTL index = 0x30),
     * Japan/MKK (CTL index = 0x40), Korea (CTL index = 0x50),
     * and China (CTL index = 0x60) CTL regions.
     * The target FW will use the CTL indices specified in this message to
     * find a BDF CTL entry with a matching CTL index value, and then use
     * that CTL as one of the inputs into the tx power limit computation.
     * A CTL index value of 0x0 is invalid, and will be ignored by the FW.
     */
    A_UINT32 conformance_test_limit_5G_subband_UNII1;
    A_UINT32 conformance_test_limit_5G_subband_UNII2a;
    A_UINT32 conformance_test_limit_5G_subband_UNII2c;
    A_UINT32 conformance_test_limit_5G_subband_UNII3;
    A_UINT32 conformance_test_limit_5G_subband_UNII4;
    /**
     * The below conformance_test_limit index fields are like the above,
     * but are for supporting the 6G sub-band CTL feature.
     */
    A_UINT32 conformance_test_limit_6G_subband_UNII5;
    A_UINT32 conformance_test_limit_6G_subband_UNII6;
    A_UINT32 conformance_test_limit_6G_subband_UNII7;
    A_UINT32 conformance_test_limit_6G_subband_UNII8;

    /**
     * In 6G sub-band CTL, fulfil 6G domain id and whole band CTL index firstly.
     * Unlike 5G sub-band CTL index fields, role ap and role client have
     * different indices.
     * Each role has 3 sub-band indices due to different power_mode type.
     * Below 3 represent for power_mode types: 0-LPI, 1-SP, 2-VLP
     * Below 2 represent for client_max: 0-default, 1-subordinate
     */

    A_UINT32 reg_domain_6G;  /* fulfil 6G domain id */
    A_UINT32 conformance_test_limit_6G; /* 6G whole band CTL index */

    A_UINT32 conformance_test_limit_6G_subband_UNII5_ap[3];
    A_UINT32 conformance_test_limit_6G_subband_UNII6_ap[3];
    A_UINT32 conformance_test_limit_6G_subband_UNII7_ap[3];
    A_UINT32 conformance_test_limit_6G_subband_UNII8_ap[3];

    A_UINT32 conformance_test_limit_6G_subband_UNII5_client[3][2];
    A_UINT32 conformance_test_limit_6G_subband_UNII6_client[3][2];
    A_UINT32 conformance_test_limit_6G_subband_UNII7_client[3][2];
    A_UINT32 conformance_test_limit_6G_subband_UNII8_client[3][2];

    /** reg domain bitmap */
    A_UINT32 regdomain_bitmap;
} wmi_pdev_set_regdomain_cmd_fixed_param;

typedef struct {
    /** TRUE for scan start and flase for scan end */
    A_UINT32 scan_start;
} wmi_pdev_scan_cmd;

/* WMI support for setting ratemask in target */

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_vdev_config_ratemask_fixed_param */
    A_UINT32 vdev_id;
    /*
     * 0 - cck/ofdm
     * 1 - HT
     * 2 - VHT
     * 3 - HE
     * 4 - EHT
     *
     * Rate Bit mask format:
     *     <MCS in NSS MAX> ...
     *     <MCS MAX, ..., 2, 1, 0 : NSS2>
     *     <MCS MAX, ..., 2, 1, 0 : NSS1>
     * EHT Rate Bit Mask format:
     *     <MCS in NSS MAX> ...
     *     <MCS MAX, ... 2, 1, 0, -1, -2 : NSS2>
     *     <MCS MAX, ..., 2, 1, 0, -1(DCM), -2(EHT Dup) : NSS1>
     */
    A_UINT32 type;

    A_UINT32 mask_lower32;
    A_UINT32 mask_higher32;
    A_UINT32 mask_lower32_2;
    A_UINT32 mask_higher32_2;
} wmi_vdev_config_ratemask_cmd_fixed_param;

/* nrp action - Filter Neighbor Rx Packets  - add/remove filter */
enum {
    WMI_FILTER_NRP_ACTION_ADD        = 0x1,
    WMI_FILTER_NRP_ACTION_REMOVE     = 0x2,
    WMI_FILTER_NRP_ACTION_GET_LIST   = 0x3,
}; /* nrp - Neighbor Rx Packets */

/* nrp type - Filter Neighbor Rx Packets  - ap/client addr */
enum {
    WMI_FILTER_NRP_TYPE_AP_BSSID     = 0x1,
    WMI_FILTER_NRP_TYPE_STA_MACADDR  = 0x2,
};

/* nrp flag - Filter Neighbor Rx Packets
 * (capture flag, 2 & 3 not initially supported)
 */
enum {
    WMI_FILTER_NRP_CAPTURE_ONLY_RX_PACKETS      = 0x1,
    WMI_FILTER_NRP_CAPTURE_ONLY_TX_PACKETS      = 0x2,
    WMI_FILTER_NRP_CAPTURE_BOTH_TXRX_PACKETS    = 0x3,
};

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_vdev_filter_nrp_config_cmd_fixed_param */
    A_UINT32 vdev_id;
    /* AP Bssid or Client Mac-addr */
    wmi_mac_addr addr;
    /* Add/Remove NRF Filter */
    A_UINT32 action; /* WMI_FILTER_NRP_ACTION enum */
    /* client/ap filter */
    A_UINT32 type; /* WMI_FILTER_NRP_TYPE enum */
    /* optional - tx/rx capture */
    A_UINT32 flag; /* WMI_FILTER_NRP_CAPTURE enum */
    /* BSSID index - index of the BSSID register */
    A_UINT32 bssid_idx;
} wmi_vdev_filter_nrp_config_cmd_fixed_param; /* Filter for Neighbor Rx Packets */

/* tx peer filter action - Filter Tx Packets  - add/remove filter */
enum {
    WMI_PEER_TX_FILTER_ACTION_ADD                           = 1,
    WMI_PEER_TX_FILTER_ACTION_REMOVE                        = 2,
    WMI_PEER_TX_FILTER_ACTION_ADD_AND_ENABLE_FILTERING      = 3,
    WMI_PEER_TX_FILTER_ACTION_REMOVE_AND_CLEAR_FILTERING    = 4,
};

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_peer_tx_filter_cmd_fixed_param */
    A_UINT32 vdev_id;
    /* Client  MAC addr */
    wmi_mac_addr addr;
    /* Add/Remove monitor_sta Filter */
    A_UINT32 action; /* WMI_PEER_TX_FILTER_ACTION enum */
} wmi_peer_tx_filter_cmd_fixed_param; /* Filter for TX Packets */

/* Command to set/unset chip in quiet mode */
typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_pdev_set_quiet_cmd_fixed_param */
    A_UINT32 pdev_id; /** pdev_id for identifying the MAC, See macros starting with WMI_PDEV_ID_ for values. */
    A_UINT32 period; /*period in TUs*/
    A_UINT32 duration; /*duration in TUs*/
    A_UINT32 next_start; /*offset in TUs*/
    A_UINT32 enabled; /*enable/disable*/
} wmi_pdev_set_quiet_cmd_fixed_param;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_vdev_set_quiet_cmd_fixed_param */
    A_UINT32 vdev_id;    /* Virtual interface ID */
    A_UINT32 period;     /* period in TUs */
    A_UINT32 duration;   /* duration in TUs */
    A_UINT32 next_start; /* offset in TUs */
    A_UINT32 enabled;    /* enable/disable */
} wmi_vdev_set_quiet_cmd_fixed_param;

/*
 * START_STOP flag value: 1 - Start, 0 - Stop
 */
#define WMI_OFFLOAD_QUIET_FLAG_START_STOP              0x00000001
/*
 * ONE_SHOT flag value: 1 - One shot, 0 - Repeat
 * This flag is only relevant if the START_STOP flag == 1 (start).
 */
#define WMI_OFFLOAD_QUIET_FLAG_ONE_SHOT                0x00000002
/*
 * Enable/Disable sending Quiet IE info in SWBA event from the target
 * 0 - Don't include Quiet IE in WMI SWBA Event
 * 1 - Include Quiet IE in WMI SWBA Event
 */
#define WMI_OFFLOAD_QUIET_FLAG_INFO_IN_SWBA_START_STOP 0x00000004

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_vdev_bcn_offload_ml_quiet_config_params */
    A_UINT32 vdev_id;    /* partner vdev_id */
    A_UINT32 hw_link_id; /* hw_link_id: Unique link id across SOCs, got as part of QMI handshake */
    A_UINT32 beacon_interval; /* beacon interval in TU from received beacon of the partner link */
    A_UINT32 period;     /* period in TUs */
    A_UINT32 duration;   /* duration in TUs */
    A_UINT32 next_start; /* offset in TUs from beacon */
    A_UINT32 flags;      /* STOP or START (and single vs. repeated) Quiet IE
                          * See WMI_OFFLOAD_QUIET_FLAG_xxx defs.
                          */
} wmi_vdev_bcn_offload_ml_quiet_config_params;

typedef struct {
    A_UINT32 tlv_header; /* TLV tag and len; tag equals WMITLV_TAG_STRUC_wmi_vdev_bcn_offload_quiet_config_cmd_fixed_param */
    A_UINT32 vdev_id;    /* Virtual interface ID */
    A_UINT32 period;     /* period in TUs */
    A_UINT32 duration;   /* duration in TUs */
    A_UINT32 next_start; /* offset in TUs from beacon */
    A_UINT32 flags;      /* STOP or START (and single vs. repeated) Quiet IE
                          * See WMI_OFFLOAD_QUIET_FLAG_xxx defs.
                          */
/*
 * This TLV is optionally followed by array of wmi_vdev_bcn_offload_ml_quiet_config_params struct
 * wmi_vdev