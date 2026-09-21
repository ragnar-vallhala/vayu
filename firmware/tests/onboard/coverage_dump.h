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
#ifndef VAYU_HWTEST_COVERAGE_DUMP_H
#define VAYU_HWTEST_COVERAGE_DUMP_H
/* On-target gcov dump (C3). Walks the .gcov_info linker section and streams the
 * gcfn+gcda merge-stream to 0:cov.gcd on the SD card (8.3 name: FatFS LFN is off,
 * so a 4-char ".gcda" extension is rejected). Download it, rename to cov.gcda, and
 * reconstruct with `arm-none-eabi-gcov-tool merge-stream`. No-op unless VAYU_HW_TEST_COV.
 * See firmware/docs/plans/on-hardware-test-and-coverage.md (tier 2 / C3). */
void coverage_dump(void);
#endif
