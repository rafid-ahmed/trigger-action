#pragma once

#include <stdint.h>

/***********************************************************************/
/* Tunes */
/***********************************************************************/
typedef struct {
    uint32_t tuneId;
    const uint16_t *notes;
    uint16_t noteCount;
    uint16_t bpm;
} BuzzerTune;

#define C4  (262)
#define C4s (273)
#define D4b (C4s)
#define D4  (294)
#define D4s (311)
#define E4b (D4s)
#define E4  (330)
#define F4  (349)
#define F4s (370)
#define G4b (F4s)
#define G4  (392)
#define G4s (415)
#define A4b (415)
#define A4  (440)
#define A4s (466)
#define B4b (A4s)
#define B4  (494)
#define C5  (523)
#define C5s (554)
#define D5b (C5s)
#define D5  (587)
#define D5s (622)
#define E5b (D5s)
#define E5  (659)
#define F5  (698)
#define F5s (740)
#define G5b (F5s)
#define G5  (784)
#define G5s (831)
#define A5b (G5s)
#define A5  (880)
#define A5s (932)
#define B5b (A5s)
#define B5  (988)
#define C6  (1047)

// #define BUZZER_TUNE_BWV147_LENGTH   (63)
// const uint16_t BUZZER_TUNE_BWV147[ BUZZER_TUNE_BWV147_LENGTH ] = {
//     G4, A4, B4, D5, C5, C5, E5, D5,
//     D5, G5, F5s, G5, D5, B4, G4, A4, B4,
//     C5, D5, E5, D5, C5, B4, A4, B4, G4,
//     F4s, G4, A4, D4, F4s, A4, C5, B4, A4,
//     B4, G4, A4, B4, D5, C5, C5, E5, D5,
//     D5, G5, F5s, G5, D5, B4, G4, A4, B4,
//     E4, D5, C5, B4, A4, G4, D4, G4, F4s, G4   };

#define BUZZER_TUNE_BWV147_LENGTH   (63)
const uint16_t BUZZER_TUNE_BWV147[ 2 * BUZZER_TUNE_BWV147_LENGTH ] = {
    G4, 1, A4, 1, B4, 1, D5, 1, C5, 1, C5, 1, E5, 1, D5, 1,
    D5, 1, G5, 1, F5s, 1, G5, 1, D5, 1, B4, 1, G4, 1, A4, 1, B4, 1,
    C5, 1, D5, 1, E5, 1, D5, 1, C5, 1, B4, 1, A4, 1, B4, 1, G4, 1,
    F4s, 1, G4, 1, A4, 1, D4, 1, F4s, 1, A4, 1, C5, 1, B4, 1, A4, 1,
    B4, 1, G4, 1, A4, 1, B4, 1, D5, 1, C5, 1, C5, 1, E5, 1, D5, 1,
    D5, 1, G5, 1, F5s, 1, G5, 1, D5, 1, B4, 1, G4, 1, A4, 1, B4, 1,
    E4, 1, D5, 1, C5, 1, B4, 1, A4, 1, G4, 1, D4, 1, G4, 1, F4s, 1, G4, 1   };

#define BUZZER_TUNE_SANDSTORM_LENGTH   (96)
const uint16_t BUZZER_TUNE_SANDSTORM[ 2 * BUZZER_TUNE_SANDSTORM_LENGTH ] = {
    B4, 1, B4, 1, B4, 1, B4, 1, B4, 2,                      // 5
    B4, 1, B4, 1, B4, 1, B4, 1, B4, 1, B4, 1, B4, 2,        // 7
    E5, 1, E5, 1, E5, 1, E5, 1, E5, 1, E5, 1, E5, 2,        // 7
    D5, 1, D5, 1, D5, 1, D5, 1, D5, 1, D5, 1, D5, 2,        // 7
    A4, 1, A4, 1,                                           // 2
    B4, 1, B4, 1, B4, 1, B4, 1, B4, 2,                      // 5
    B4, 1, B4, 1, B4, 1, B4, 1, B4, 1, B4, 1, B4, 2,        // 7
    E5, 1, E5, 1,                                           // 2
    B4, 1, B4, 1, B4, 1, B4, 1, B4, 2,                      // 5
    B4, 1, B4, 1, B4, 1, B4, 1, B4, 1, B4, 1, B4, 2,        // 7
    E5, 1, E5, 1,                                           // 2
    B4, 1, B4, 1, B4, 1, B4, 1, B4, 2,                      // 5
    B4, 1, B4, 1, B4, 1, B4, 1, B4, 1, B4, 1, B4, 2,        // 7
    E5, 1, E5, 1, E5, 1, E5, 1, E5, 1, E5, 1, E5, 2,        // 7
    D5, 1, D5, 1, D5, 1, D5, 1, D5, 1, D5, 1, D5, 2,        // 7
    A4, 1, A4, 1,                                           // 2
    B4, 1, B4, 1, B4, 1, B4, 1, B4, 2,                      // 5
    B4, 1, B4, 1, B4, 1, B4, 1, B4, 1, B4, 1, B4, 2         // 7
};

#define BUZZER_TUNE_COUNT (2)
const BuzzerTune BUZZER_TUNES[ BUZZER_TUNE_COUNT ] = {
    { .tuneId = 1, .notes = BUZZER_TUNE_BWV147, .noteCount = BUZZER_TUNE_BWV147_LENGTH, .bpm = 240 },
    { .tuneId = 2, .notes = BUZZER_TUNE_SANDSTORM, .noteCount = BUZZER_TUNE_SANDSTORM_LENGTH, .bpm = 600 },
};
/***********************************************************************/