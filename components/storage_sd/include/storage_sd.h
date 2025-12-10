#pragma once
#include "esp_err.h"
#include "sdmmc_cmd.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool mounted;
    sdmmc_card_t *card;
} sd_state_t;

esp_err_t storage_sd_init(sd_state_t *state);
esp_err_t storage_sd_deinit(sd_state_t *state);

// Simple helpers
esp_err_t storage_sd_write_text(const char *path, const char *text);
esp_err_t storage_sd_read_text(const char *path, char *buf, size_t buf_sz, size_t *out_len);
esp_err_t storage_sd_list_dir(const char *path);

#ifdef __cplusplus
}
#endif
