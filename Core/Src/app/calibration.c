#include "app/calibration.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "app/app_config.h"
#include "stm32f4xx_hal.h"
#include "utils/crc.h"

#define CALIBRATION_MAGIC 0x43414C31u /* "CAL1" */
#define CALIBRATION_VERSION 1u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t sequence;
    int16_t pan_zero;
    int16_t tilt_zero;
    int16_t pan_min;
    int16_t pan_max;
    int16_t tilt_min;
    int16_t tilt_max;
    uint32_t reserved;
    uint32_t crc32;
} CalibrationRecord;

_Static_assert(sizeof(CalibrationRecord) == 32u, "calibration record must stay 32 bytes");

typedef enum {
    CALIBRATION_BANK_NONE = 0,
    CALIBRATION_BANK_A,
    CALIBRATION_BANK_B
} calibration_bank_t;

static CalibrationData s_data;
static calibration_bank_t s_active_bank;
static volatile bool s_commit_requested;
static SemaphoreHandle_t s_mutex;
static StaticSemaphore_t s_mutex_control;

static void Calibration_Lock(void) {
    (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
}

static void Calibration_Unlock(void) {
    (void)xSemaphoreGive(s_mutex);
}

static uint32_t BankAddress(calibration_bank_t bank) {
    return (bank == CALIBRATION_BANK_B) ? APP_CALIBRATION_ADDRESS_B : APP_CALIBRATION_ADDRESS_A;
}

static uint32_t BankSector(calibration_bank_t bank) {
    return (bank == CALIBRATION_BANK_B) ? APP_CALIBRATION_SECTOR_B : APP_CALIBRATION_SECTOR_A;
}

static bool RecordIsValid(const CalibrationRecord* record) {
    if (record->magic != CALIBRATION_MAGIC || record->version != CALIBRATION_VERSION ||
        record->length != sizeof(CalibrationRecord)) {
        return false;
    }

    uint32_t crc = Crc32((const uint8_t*)record, offsetof(CalibrationRecord, crc32));
    return crc == record->crc32;
}

static bool ReadBank(calibration_bank_t bank, CalibrationRecord* record) {
    memcpy(record, (const void*)BankAddress(bank), sizeof(*record));
    return RecordIsValid(record);
}

static void LoadRecord(const CalibrationRecord* record) {
    s_data.valid = true;
    s_data.sequence = record->sequence;
    s_data.pan_zero = record->pan_zero;
    s_data.tilt_zero = record->tilt_zero;
    s_data.pan_min = record->pan_min;
    s_data.pan_max = record->pan_max;
    s_data.tilt_min = record->tilt_min;
    s_data.tilt_max = record->tilt_max;
}

void Calibration_Init(void) {
    s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_control);
    configASSERT(s_mutex != NULL);

    CalibrationRecord record_a;
    CalibrationRecord record_b;
    bool valid_a = ReadBank(CALIBRATION_BANK_A, &record_a);
    bool valid_b = ReadBank(CALIBRATION_BANK_B, &record_b);

    s_commit_requested = false;
    s_active_bank = CALIBRATION_BANK_NONE;

    s_data = (CalibrationData){
        .valid = false,
        .sequence = 0u,
        .pan_zero = 0,
        .tilt_zero = 0,
        .pan_min = APP_GIMBAL_PAN_MIN,
        .pan_max = APP_GIMBAL_PAN_MAX,
        .tilt_min = APP_GIMBAL_TILT_MIN,
        .tilt_max = APP_GIMBAL_TILT_MAX,
    };

    if (valid_a && (!valid_b || record_a.sequence >= record_b.sequence)) {
        LoadRecord(&record_a);
        s_active_bank = CALIBRATION_BANK_A;
    } else if (valid_b) {
        LoadRecord(&record_b);
        s_active_bank = CALIBRATION_BANK_B;
    }
}

bool Calibration_IsValid(void) {
    Calibration_Lock();
    bool valid = s_data.valid;
    Calibration_Unlock();
    return valid;
}

void Calibration_GetSnapshot(CalibrationData* data) {
    if (data == NULL) {
        return;
    }

    Calibration_Lock();
    *data = s_data;
    Calibration_Unlock();
}

void Calibration_SetPanZero(int16_t value) {
    Calibration_Lock();
    s_data.pan_zero = value;
    Calibration_Unlock();
}

void Calibration_SetTiltZero(int16_t value) {
    Calibration_Lock();
    s_data.tilt_zero = value;
    Calibration_Unlock();
}

void Calibration_SetLimits(int16_t pan_min, int16_t pan_max, int16_t tilt_min, int16_t tilt_max) {
    Calibration_Lock();
    s_data.pan_min = pan_min;
    s_data.pan_max = pan_max;
    s_data.tilt_min = tilt_min;
    s_data.tilt_max = tilt_max;
    Calibration_Unlock();
}

void Calibration_RequestCommit(void) {
    s_commit_requested = true;
}

static void FillRecord(CalibrationRecord* record, const CalibrationData* data) {
    memset(record, 0, sizeof(*record));
    record->magic = CALIBRATION_MAGIC;
    record->version = CALIBRATION_VERSION;
    record->length = sizeof(*record);
    record->sequence = data->sequence + 1u;
    record->pan_zero = data->pan_zero;
    record->tilt_zero = data->tilt_zero;
    record->pan_min = data->pan_min;
    record->pan_max = data->pan_max;
    record->tilt_min = data->tilt_min;
    record->tilt_max = data->tilt_max;
    record->crc32 = Crc32((const uint8_t*)record, offsetof(CalibrationRecord, crc32));
}

static bool ProgramBank(calibration_bank_t bank, const CalibrationRecord* record) {
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = BankSector(bank),
        .NbSectors = 1u,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t sector_error = 0u;
    const uint32_t address = BankAddress(bank);
    const uint32_t* words = (const uint32_t*)record;
    bool ok = true;

    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        ok = false;
    }

    for (uint32_t i = 0u; ok && i < (sizeof(*record) / 4u); i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i * 4u, words[i]) != HAL_OK) {
            ok = false;
        }
    }
    HAL_FLASH_Lock();

    if (!ok) {
        return false;
    }

    return memcmp((const void*)address, record, sizeof(*record)) == 0;
}

calibration_commit_t Calibration_ProcessCommit(void) {
    if (!s_commit_requested) {
        return CALIBRATION_COMMIT_NONE;
    }
    s_commit_requested = false;

    /* Take a consistent snapshot, then release the lock for the flash
     * operation. The erase/program can take seconds; holding the mutex here
     * would stall every command and telemetry update. */
    Calibration_Lock();
    calibration_bank_t target =
        (s_active_bank == CALIBRATION_BANK_A) ? CALIBRATION_BANK_B : CALIBRATION_BANK_A;
    CalibrationData snapshot = s_data;
    Calibration_Unlock();

    CalibrationRecord record;
    FillRecord(&record, &snapshot);

    if (!ProgramBank(target, &record)) {
        return CALIBRATION_COMMIT_FAILED;
    }

    Calibration_Lock();
    s_active_bank = target;
    s_data.valid = true;
    s_data.sequence = record.sequence;
    Calibration_Unlock();
    return CALIBRATION_COMMIT_OK;
}
