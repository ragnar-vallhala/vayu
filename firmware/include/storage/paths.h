/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/**
 * @file storage/paths.h
 * @brief Where the firmware keeps things on the card.
 *
 * All 8.3 names: FF_USE_LFN is 0, so a longer one fails vfs_open with
 * FR_INVALID_NAME rather than being truncated.
 *
 * The two stores are a pair -- both are the result of a procedure nobody wants
 * to repeat, and fs_query refuses to delete either (see the delete policy in
 * comm/xfer/fs_query.c).
 *
 * PID_FILE_PATH and PID_FILE_SIZE used to sit beside these, a second spelling
 * of "0:pid.bin" with no users at all. Two names for one file is how two
 * writers of one file start.
 */
#ifndef VAYU_STORAGE_PATHS_H
#define VAYU_STORAGE_PATHS_H

/** Persisted PID tune, restored by pid_config_init() before the controllers. */
#define PID_CONFIG_FILE_PATH "0:pid.bin"

/** Persisted sensor calibration. */
#define CALIBRATION_FILE_PATH "0:cal.bin"

/* The blackbox. Everything recorded in flight lives here: the sampled streams
 * (imu/act/vrt/ctl) plus the byte streams (NavLink RX, vayu_log text). It used
 * to share the card with three 10 MB circular log files that nothing ever
 * wrote to; those are gone and their one live use, the text log, is a stream
 * in here instead. 8.3 name -- FF_USE_LFN is 0 and a longer name fails
 * vfs_open with FR_INVALID_NAME. */
#define HSL_FILENAME "0:blackbox.bin"
#ifdef VAYU_SIM
/* 64 sectors -> a 63-slot ring: small enough that test_hslog can drive it all
 * the way round, big enough to hold two multi-stream sessions first. Bounded by
 * the host VFS's per-file cap (sim/host/src/host_vfs.c HOST_VFS_FILE_CAP). */
#define HSL_FILE_SIZE (32 * 1024)
#else
/* Sized by how much armed history is worth keeping, not by one flight: arms
 * ACCUMULATE into a circular ring (see storage/imu_hs_log.h) and the ring
 * resumes across power cycles, so the file holds the last HSL_FILE_SIZE of
 * armed time however many arms and boots that spans.
 * 32 MB = ~22 min at 2 kHz x 12 B -- a whole props-on bench session, not just
 * one pack. Size is close to free here: imu_hs_log_boot_init() creates the
 * file by seeking past EOF and writing one byte, which allocates the cluster
 * chain in ~8 FAT sector writes and never touches the data sectors. It does
 * NOT use vfs_preallocate, which would zero-fill all 65536 of them -- see the
 * note in imu_hs_log.c. The clusters therefore come back holding whatever the
 * card had before, which is what HSL_RING_SENTINEL is for.
 *
 * NB raising this later still works -- imu_hs_log_boot_init() extends an
 * existing short file -- but LOWERING it does not shrink one. */
#define HSL_FILE_SIZE (32 * 1024 * 1024)
#endif

#endif // VAYU_STORAGE_PATHS_H
