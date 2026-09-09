/* Host SITL stub for NavHAL family/rcc_reg.h. The host build never
 * touches RCC registers — we just need the type names that
 * clock_types.h embeds in hal_clock_config_t. */
#ifndef VAYU_SIM_RCC_REG_H
#define VAYU_SIM_RCC_REG_H

typedef enum {
  RCC_CFGR_HPRE_DIV1 = 0,
} rcc_cfgr_hpre_div_t;

typedef enum {
  RCC_CFGR_PPRE_DIV1 = 0,
} rcc_cfgr_ppre_div_t;

#endif
