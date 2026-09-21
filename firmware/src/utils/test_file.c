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
#include "comm/comm.h"
#include "storage/fs_owner.h"
#include "navhal.h"
#include "vaios.h"
#include "variables.h"
#include "vfs.h"
int write_pos = 0;
/** @noreq scratch logger test task */
void test_task(void *args) {
  while (1) {
    // send_packet(&g_telemetry_channel, PACKET_TYPE_LOG, (byte *)"Hello", 5);
    fs_owner_enqueue_log(GENERAL_LOGGER, (byte *)"Hello How are you?", 18);
    v_delay(500);
  }
}
