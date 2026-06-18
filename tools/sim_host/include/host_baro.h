#ifndef VAYU_HOST_BARO_H
#define VAYU_HOST_BARO_H

/* Launch the SITL barometer feeder thread: reads modelled pressure frames from
 * vsim_d's /tmp/vsim_baro FIFO and injects them into the REAL firmware bme280
 * path (bme280_publish), so the FC's own altitude derivation + BARO telemetry
 * run in SITL exactly as on hardware. Mirrors host_imu_feeder_start(). */
void host_baro_start(void);

#endif /* VAYU_HOST_BARO_H */
