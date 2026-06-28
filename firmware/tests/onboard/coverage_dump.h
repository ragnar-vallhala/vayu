#ifndef VAYU_HWTEST_COVERAGE_DUMP_H
#define VAYU_HWTEST_COVERAGE_DUMP_H
/* On-target gcov dump (C3). Walks the .gcov_info linker section and streams the
 * gcfn+gcda merge-stream to 0:cov.gcda on the SD card; the host reconstructs it
 * with `arm-none-eabi-gcov-tool merge-stream`. No-op unless VAYU_HW_TEST_COV.
 * See docs/plans/on-hardware-test-and-coverage.md (tier 2 / C3). */
void coverage_dump(void);
#endif
