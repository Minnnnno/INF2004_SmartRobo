/** @file memory_probe.c
 * @brief ARM ABI size probes; compile only, never link into firmware.
 */
#include "motion.h"
#include "encoder_capture.h"

const uint8_t g_size_motion[sizeof(motion_t)] = {0u};
const uint8_t g_size_command[sizeof(motion_command_t)] = {0u};
const uint8_t g_size_status[sizeof(motion_status_t)] = {0u};
const uint8_t g_size_event[sizeof(motion_event_t)] = {0u};
const uint8_t g_size_encoder[sizeof(encoder_capture_t)] = {0u};
const uint8_t g_size_pid[sizeof(pid_t)] = {0u};
const uint8_t g_size_odometry[sizeof(odometry_t)] = {0u};
const uint8_t g_size_config[sizeof(motion_config_t)] = {0u};
const uint8_t g_size_state_enum[sizeof(motion_state_t)] = {0u};
const uint8_t g_size_mode_enum[sizeof(motion_mode_t)] = {0u};
const uint8_t g_size_fault_enum[sizeof(motion_fault_t)] = {0u};
#ifdef MOTION_STORAGE_H
const uint8_t g_size_stored_command[sizeof(motion_storage_command_t)] = {0u};
const uint8_t g_size_stored_events[sizeof(motion_storage_events_t)] = {0u};
#endif
