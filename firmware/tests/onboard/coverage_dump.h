#ifndef VAYU_HWTEST_COVERAGE_DUMP_H
#define VAYU_HWTEST_COVERAGE_DUMP_H
/* On-target gcov dump (C3). Walks the .gcov_info linker section and streams the
 * gcfn+gcda merge-stream to 0:cov.gcd on the SD card (8.3 name: FatFS LFN is off,
 * so a 4-char ".gcda" extension is rejected). Download it, rename to cov.gcda, and
 * reconstruct with `arm-none-eabi-gcov-tool merge-stream`. No-op unless VAYU_HW_TEST_COV.
 * See firmware/docs/plans/on-hardware-test-and-coverage.md (tier 2 / C3). */
void coverage_dump(void);
#endif
