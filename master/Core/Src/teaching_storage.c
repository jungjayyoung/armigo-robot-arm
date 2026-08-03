#include "teaching_storage.h"
#include "stm32f4xx_hal_flash_ex.h"
#include <stddef.h>
#include <string.h>

#define TEACHING_FLASH_START  0x08060000UL
#define TEACHING_FLASH_END    0x08080000UL
#define TEACHING_FLASH_SECTOR FLASH_SECTOR_7
/* Changed with the 30-step record layout. Older 10-step records are safely
 * ignored and the sector is erased on the first save. */
#define TEACHING_MAGIC        0x5445434AUL

typedef struct
{
  uint32_t magic;
  uint32_t sequence;
  TeachingSequence_t presets[TEACHING_PRESET_COUNT + 1U];
  uint32_t checksum;
} TeachingRecord_t;

static uint32_t TeachingStorage_Checksum(const TeachingRecord_t *record)
{
  const uint8_t *bytes = (const uint8_t *)record;
  const uint32_t length = (uint32_t)offsetof(TeachingRecord_t, checksum);
  uint32_t hash = 2166136261UL;

  for (uint32_t i = 0U; i < length; ++i)
  {
    hash ^= bytes[i];
    hash *= 16777619UL;
  }
  return hash;
}

static bool TeachingStorage_IsValid(const TeachingRecord_t *record)
{
  return (record->magic == TEACHING_MAGIC) &&
         (record->checksum == TeachingStorage_Checksum(record));
}

static const TeachingRecord_t *TeachingStorage_FindLatest(uint32_t *next_address)
{
  const TeachingRecord_t *latest = NULL;
  uint32_t address = TEACHING_FLASH_START;

  while ((address + sizeof(TeachingRecord_t)) <= TEACHING_FLASH_END)
  {
    const TeachingRecord_t *record = (const TeachingRecord_t *)address;

    if (record->magic == 0xFFFFFFFFUL)
    {
      break;
    }
    if (TeachingStorage_IsValid(record) &&
        ((latest == NULL) || (record->sequence > latest->sequence)))
    {
      latest = record;
    }
    address += sizeof(TeachingRecord_t);
  }

  if (next_address != NULL)
  {
    /* No valid record means a previous firmware layout may still be in this
     * sector. Start at the sector head so Save erases it before writing. */
    *next_address = (latest == NULL) ? TEACHING_FLASH_START : address;
  }
  return latest;
}

bool TeachingStorage_Load(TeachingSequence_t presets[TEACHING_PRESET_COUNT + 1U])
{
  const TeachingRecord_t *latest;

  if (presets == NULL)
  {
    return false;
  }

  latest = TeachingStorage_FindLatest(NULL);
  if (latest == NULL)
  {
    return false;
  }

  memcpy(presets, latest->presets, sizeof(latest->presets));
  return true;
}

bool TeachingStorage_Save(const TeachingSequence_t presets[TEACHING_PRESET_COUNT + 1U])
{
  /* 30-step record is ~2.9 KB.  Keep it out of the 2 KB keypad command
   * task stack; BTN12 invokes this function from that task. */
  static TeachingRecord_t record;
  const TeachingRecord_t *latest;
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0U;
  uint32_t address;
  const uint32_t *words;
  bool success = true;

  if (presets == NULL)
  {
    return false;
  }

  latest = TeachingStorage_FindLatest(&address);
  memset(&record, 0, sizeof(record));
  record.magic = TEACHING_MAGIC;
  record.sequence = (latest == NULL) ? 1U : (latest->sequence + 1U);
  memcpy(record.presets, presets, sizeof(record.presets));
  record.checksum = TeachingStorage_Checksum(&record);

  if ((address + sizeof(record)) > TEACHING_FLASH_END)
  {
    address = TEACHING_FLASH_START;
  }

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return false;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                         FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                         FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  if (address == TEACHING_FLASH_START)
  {
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector = TEACHING_FLASH_SECTOR;
    erase.NbSectors = 1U;
    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
    {
      success = false;
    }
  }

  words = (const uint32_t *)&record;
  for (uint32_t offset = 0U;
       success && (offset < sizeof(record));
       offset += sizeof(uint32_t))
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          address + offset,
                          words[offset / sizeof(uint32_t)]) != HAL_OK)
    {
      success = false;
    }
  }

  (void)HAL_FLASH_Lock();

  if (!success)
  {
    return false;
  }
  return TeachingStorage_IsValid((const TeachingRecord_t *)address);
}
