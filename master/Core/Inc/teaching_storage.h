#ifndef TEACHING_STORAGE_H
#define TEACHING_STORAGE_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define TEACHING_PRESET_COUNT 10U
#define TEACHING_AXIS_COUNT    4U
#define TEACHING_SEQUENCE_STEPS 30U /* Up to 30 sequential robot actions. */
#define TEACHING_EMPTY_AXIS_VALUE 0xFFFFU /* Unused step: NULL-equivalent. */
/* Optional settling time after a confirmed AUTO step (milliseconds).
 * The next target is never sent before the current target is reached. */
#define AUTO_SEQUENCE_STEP_DELAY_MS 10U
#define TEACHING_SAVE_DEADBAND 5U /* Ignore normal ±5 AX-12 position jitter. */

typedef struct
{
  uint16_t axis[TEACHING_AXIS_COUNT];
} TeachingPoint_t;

typedef struct
{
  TeachingPoint_t step[TEACHING_SEQUENCE_STEPS];
  uint32_t saved_mask; /* bit0..29: saved; remaining steps hold EMPTY value. */
} TeachingSequence_t;

bool TeachingStorage_Load(TeachingSequence_t presets[TEACHING_PRESET_COUNT + 1U]);
bool TeachingStorage_Save(const TeachingSequence_t presets[TEACHING_PRESET_COUNT + 1U]);

#endif
