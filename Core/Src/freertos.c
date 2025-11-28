/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "semphr.h"
#include "tim.h"
#include "lcd_i2c.h"
#include <stdio.h>
#include <stdlib.h>   // rand()
#include <string.h>   // strlen, memcpy
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRUIT_TIMEOUT_MS 2000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t gameTaskHandle;
const osThreadAttr_t gameTask_attributes = {
  .name       = "gameTask",
  .stack_size = 256 * 4,
  .priority   = (osPriority_t) osPriorityNormal,
};

/* --- SOUND TASK STUFF --- */
typedef enum {
    SFX_NONE = 0,
    SFX_START,
    SFX_SLICE,
    SFX_MISS,          // NEW: missed fruit sound
    SFX_GAMEOVER_TUNE
} SfxType_t;


typedef struct {
    SfxType_t type;
} SoundEvent_t;

QueueHandle_t xSoundQueue;

osThreadId_t soundTaskHandle;
const osThreadAttr_t soundTask_attributes = {
  .name       = "soundTask",
  .stack_size = 128 * 4,
  .priority   = (osPriority_t) osPriorityBelowNormal,   // lower than game
};
/* USER CODE END Variables */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
void StartDefaultTask(void *argument);

/* USER CODE BEGIN FunctionPrototypes */
static void Buzzer_PlayTone(uint32_t freq_hz, uint32_t dur_ms);
static void LCD_DrawTimer(uint32_t seconds);
static void UpdateTimer(TickType_t endTick, uint32_t *pLastShownSec);
static void Sound_Send(SfxType_t type);
static void PlayGameOverTune(void);
void vGameTask(void *argument);
void vSoundTask(void *argument);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  xSoundQueue = xQueueCreate(8, sizeof(SoundEvent_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  gameTaskHandle  = osThreadNew(vGameTask,  NULL, &gameTask_attributes);
  soundTaskHandle = osThreadNew(vSoundTask, NULL, &soundTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// --- SOUND TASK ------------------------------------------------------------

// Send a sound effect request to the sound task
static void Sound_Send(SfxType_t type)
{
    if (!xSoundQueue) return;

    SoundEvent_t ev;
    ev.type = type;
    xQueueSend(xSoundQueue, &ev, 0);   // no block if full
}

// Sound task: waits for events and plays tones/tunes
void vSoundTask(void *argument)
{
    SoundEvent_t ev;

    for (;;)
    {
        if (xQueueReceive(xSoundQueue, &ev, portMAX_DELAY) == pdTRUE)
        {
            switch (ev.type)
            {
            case SFX_START:
                // start jingle
                Buzzer_PlayTone(600, 120);
                Buzzer_PlayTone(800, 120);
                Buzzer_PlayTone(1000, 150);
                break;

            case SFX_SLICE:
                // "shhhk" style fast slice: quick rising notes
                Buzzer_PlayTone(900,  40);
                Buzzer_PlayTone(1300, 40);
                Buzzer_PlayTone(1700, 50);
                break;

            case SFX_MISS:
                // sad descending "you failed" blip
                Buzzer_PlayTone(600,  80);
                Buzzer_PlayTone(400, 100);
                Buzzer_PlayTone(0,    60);
                break;

            case SFX_GAMEOVER_TUNE:
                // your longer heroic tune
                PlayGameOverTune();
                break;

            default:
                break;
            }
        }
    }
}


// --- SCORING HELPERS ------------------------------------------------------

// Compute per-slice score + bonus for fast slices
static uint32_t ComputeSliceScore(uint32_t diff_ms)
{
    if (diff_ms == 0) return 0;

    // base score: faster = larger (simple 1000/dt)
    uint32_t base = 1000 / diff_ms;
    uint32_t bonus = 0;

    // same thresholds as MakeFeedback()
    if (diff_ms <= 80) {
        bonus = 20;        // LIGHTNING SLAY
    } else if (diff_ms <= 150) {
        bonus = 10;        // SUPER FAST!
    } else {
        bonus = 0;         // others: no bonus
    }

    return base + bonus;
}

// Longer game-over tune so user really feels it's done
static void PlayGameOverTune(void)
{
    // Small heroic-ish melody, about ~1.5s total
    Buzzer_PlayTone(523, 180);  // C5
    Buzzer_PlayTone(587, 180);  // D5
    Buzzer_PlayTone(659, 180);  // E5
    Buzzer_PlayTone(698, 200);  // F5
    Buzzer_PlayTone(784, 220);  // G5
    Buzzer_PlayTone(659, 180);  // E5
    Buzzer_PlayTone(587, 200);  // D5
    Buzzer_PlayTone(523, 260);  // C5 hold
    Buzzer_PlayTone(0,   150);  // little pause
}

// Flash "TIME'S" / "UP!" on the LCD a few times
static void FlashTimesUp(void)
{
    for (int i = 0; i < 6; i++)     // 6 flashes
    {
        LCD_Clear();
        if ((i % 2) == 0) {
            LCD_SetCursor(0, 0);
            LCD_Print("    TIME'S     ");
            LCD_SetCursor(1, 0);
            LCD_Print("      UP!      ");
        }
        vTaskDelay(pdMS_TO_TICKS(200));   // 200 ms on/off
    }
}


// Center a short message on line 1 (bottom line)
static void LCD_PrintCenteredLine1(const char *msg)
{
    char line[17];
    int len = (int)strlen(msg);
    if (len > 16) len = 16;

    // fill with spaces
    for (int i = 0; i < 16; i++) line[i] = ' ';
    line[16] = '\0';

    int start = (16 - len) / 2;
    if (start < 0) start = 0;
    if (start + len > 16) start = 16 - len;

    memcpy(&line[start], msg, len);

    LCD_SetCursor(1, 0);
    LCD_Print(line);
}

// Pick a fun feedback message based on slice speed (diff_ms)
static void MakeFeedback(uint32_t diff_ms, char *outBuf, size_t bufLen)
{
    const char *msg;

    if (diff_ms <= 80) {
        msg = "LIGHTNING SLAY";
    } else if (diff_ms <= 150) {
        msg = "SUPER FAST!";
    } else if (diff_ms <= 250) {
        msg = "Nice & clean";
    } else if (diff_ms <= 400) {
        msg = "Getting there";
    } else {
        msg = "SLOW SLICE :(";
    }

    // copy into buffer (max 16 chars)
    snprintf(outBuf, bufLen, "%s", msg);
}


/* --- BUZZER HELPER -------------------------------------------------------- */
static void Buzzer_PlayTone(uint32_t freq_hz, uint32_t dur_ms)
{
    if (freq_hz == 0) {
        __HAL_TIM_SET_COMPARE(BUZZER_TIM, BUZZER_CHANNEL, 0);
        vTaskDelay(pdMS_TO_TICKS(dur_ms));
        return;
    }

    uint32_t timer_clk = 1000000; // 1 MHz after prescaler
    uint32_t period    = timer_clk / freq_hz;
    if (period == 0) period = 1;

    __HAL_TIM_SET_AUTORELOAD(BUZZER_TIM, period - 1);

    // ~8% duty cycle so it's not super loud
    uint32_t duty = period / 12;
    if (duty < 1) duty = 1;
    __HAL_TIM_SET_COMPARE(BUZZER_TIM, BUZZER_CHANNEL, duty);

    vTaskDelay(pdMS_TO_TICKS(dur_ms));

    __HAL_TIM_SET_COMPARE(BUZZER_TIM, BUZZER_CHANNEL, 0);
}

/* --- TIMER DISPLAY HELPERS ------------------------------------------------ */

/**
 * Draws "Xs" centered on line 0 (top line).
 * Example for 30:
 *   "      30s       "
 */
static void LCD_DrawTimer(uint32_t seconds)
{
    char timerStr[8];
    char line[17];
    int  len, start;

    snprintf(timerStr, sizeof(timerStr), "%lus", (unsigned long)seconds);
    len   = (int)strlen(timerStr);
    if (len > 16) len = 16;

    // fill line with spaces
    for (int i = 0; i < 16; i++) line[i] = ' ';
    line[16] = '\0';

    // center the timer string
    start = (16 - len) / 2;
    if (start < 0) start = 0;
    if (start + len > 16) start = 16 - len;

    memcpy(&line[start], timerStr, len);

    LCD_SetCursor(0, 0);
    LCD_Print(line);
}

/**
 * Recomputes remaining time from endTick and updates the timer line
 * only when the value changes.
 */
static void UpdateTimer(TickType_t endTick, uint32_t *pLastShownSec)
{
    TickType_t nowTicks = xTaskGetTickCount();
    if (nowTicks >= endTick) return;

    uint32_t msLeft  = (endTick - nowTicks) * portTICK_PERIOD_MS;
    uint32_t secLeft = msLeft / 1000;

    if (secLeft != *pLastShownSec) {
        *pLastShownSec = secLeft;
        LCD_DrawTimer(secLeft);
    }
}

/**
  * Game:
  * - Wait for button press
  * - 30 s round
  * - Random fruit lights up, wait for both IR beams to be cut
  * - Timer ("Xs") is always on line 0, centered, updating continuously
  * - Line 1 is used for messages (fruit, score, etc.)
  */
void vGameTask(void *argument)
{
    // seed RNG
    srand((unsigned int)HAL_GetTick());

    // init LCD here; only this task uses it
    LCD_Init();
    LCD_Clear();

    // Arrays to handle fruits more easily
    GPIO_TypeDef* ledPorts[4]  = {
        FRUIT1_LED_GPIO_Port, FRUIT2_LED_GPIO_Port,
        FRUIT3_LED_GPIO_Port, FRUIT4_LED_GPIO_Port
    };
    uint16_t ledPins[4] = {
        FRUIT1_LED_Pin, FRUIT2_LED_Pin,
        FRUIT3_LED_Pin, FRUIT4_LED_Pin
    };

    GPIO_TypeDef* ir1Ports[4] = {
        FRUIT1_IR1_GPIO_Port, FRUIT2_IR1_GPIO_Port,
        FRUIT3_IR1_GPIO_Port, FRUIT4_IR1_GPIO_Port
    };
    uint16_t ir1Pins[4] = {
        FRUIT1_IR1_Pin, FRUIT2_IR1_Pin,
        FRUIT3_IR1_Pin, FRUIT4_IR1_Pin
    };

    GPIO_TypeDef* ir2Ports[4] = {
        FRUIT1_IR2_GPIO_Port, FRUIT2_IR2_GPIO_Port,
        FRUIT3_IR2_GPIO_Port, FRUIT4_IR2_GPIO_Port
    };
    uint16_t ir2Pins[4] = {
        FRUIT1_IR2_Pin, FRUIT2_IR2_Pin,
        FRUIT3_IR2_Pin, FRUIT4_IR2_Pin
    };

    char feedback[17];       // message about previous slice

    for (;;)
    {
        /* ----------- IDLE SCREEN: WAIT FOR START BUTTON ----------- */
        LCD_Clear();
        LCD_SetCursor(0, 0);
        LCD_Print("Press button to");
        LCD_SetCursor(1, 0);
        LCD_Print("start slaying ;)");

        // wait until PB0 is pressed (pull-up, active LOW)
        while (HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin) == GPIO_PIN_SET)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        // debounce + wait for release
        vTaskDelay(pdMS_TO_TICKS(50));
        while (HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin) == GPIO_PIN_RESET)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // start jingle
        Sound_Send(SFX_START);


        // game lasts 30 seconds
        TickType_t now     = xTaskGetTickCount();
        TickType_t endTick = now + pdMS_TO_TICKS(30000);
        uint32_t   lastSecShown = 0xFFFFFFFF;

        // scoring state
        uint32_t totalScore    = 0;
        uint32_t fruitsSlayed  = 0;

        // initial feedback
        snprintf(feedback, sizeof(feedback), "Ready to slay!");
        UpdateTimer(endTick, &lastSecShown);
        LCD_PrintCenteredLine1(feedback);

        /* ----------- MAIN GAME LOOP (30 s) ----------- */
        while (xTaskGetTickCount() < endTick)
        {
            // choose random fruit 0..3
            int fruit = rand() % 4;

            // turn OFF all LEDs
            for (int i = 0; i < 4; i++)
            {
                HAL_GPIO_WritePin(ledPorts[i], ledPins[i], GPIO_PIN_RESET);
            }

            // turn ON chosen fruit LED
            HAL_GPIO_WritePin(ledPorts[fruit], ledPins[fruit], GPIO_PIN_SET);

            // show feedback from PREVIOUS slice on line 1
            UpdateTimer(endTick, &lastSecShown);
            LCD_PrintCenteredLine1(feedback);

            // wait for slice: both IR beams
            TickType_t t1 = 0, t2 = 0;

            // per-fruit deadline (e.g. 2 seconds max)
            TickType_t fruitDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(FRUIT_TIMEOUT_MS);

            while (xTaskGetTickCount() < endTick &&
                   xTaskGetTickCount() < fruitDeadline)
            {
                TickType_t tnow = xTaskGetTickCount();

                // continuously update timer while waiting
                UpdateTimer(endTick, &lastSecShown);

                GPIO_PinState s1 = HAL_GPIO_ReadPin(ir1Ports[fruit], ir1Pins[fruit]);
                GPIO_PinState s2 = HAL_GPIO_ReadPin(ir2Ports[fruit], ir2Pins[fruit]);

                // IR modules assumed active LOW
                if (s1 == GPIO_PIN_RESET && t1 == 0) t1 = tnow;
                if (s2 == GPIO_PIN_RESET && t2 == 0) t2 = tnow;

                if (t1 && t2) break;

                vTaskDelay(pdMS_TO_TICKS(1));
            }

            // If we exit the loop without both beams hit:
            if (!(t1 && t2))
            {
                // Case 1: whole game time is over -> end game
                if (xTaskGetTickCount() >= endTick)
                {
                    break;   // break out of the main game loop
                }

                // Case 2: fruit timeout only -> MISSED FRUIT, move to next fruit
                Sound_Send(SFX_MISS);

                snprintf(feedback, sizeof(feedback), "MISSED FRUIT!");
                UpdateTimer(endTick, &lastSecShown);
                LCD_PrintCenteredLine1(feedback);

                // short pause so player can see they messed up,
                // but still keep timer updating
                TickType_t missEnd = xTaskGetTickCount() + pdMS_TO_TICKS(500);
                while (xTaskGetTickCount() < missEnd &&
                       xTaskGetTickCount() < endTick)
                {
                    UpdateTimer(endTick, &lastSecShown);
                    vTaskDelay(pdMS_TO_TICKS(40));
                }

                // go straight to next fruit (no scoring)
                continue;
            }


            // turn off LEDs (all 4 fruits)
            for (int i = 0; i < 4; i++)
            {
                HAL_GPIO_WritePin(ledPorts[i], ledPins[i], GPIO_PIN_RESET);
            }

            // compute slice speed
            TickType_t diffTicks = (t1 > t2) ? (t1 - t2) : (t2 - t1);
            uint32_t diff_ms = diffTicks * portTICK_PERIOD_MS;

            // update scoring
            uint32_t sliceScore = ComputeSliceScore(diff_ms);
            totalScore   += sliceScore;
            fruitsSlayed += 1;

            // slice SFX
            Sound_Send(SFX_SLICE);


            // create NEW feedback based on this slice & display it
            MakeFeedback(diff_ms, feedback, sizeof(feedback));
            UpdateTimer(endTick, &lastSecShown);
            LCD_PrintCenteredLine1(feedback);

            // pause ~600 ms but keep updating timer
            TickType_t pauseEnd = xTaskGetTickCount() + pdMS_TO_TICKS(600);
            while (xTaskGetTickCount() < pauseEnd && xTaskGetTickCount() < endTick)
            {
                UpdateTimer(endTick, &lastSecShown);
                vTaskDelay(pdMS_TO_TICKS(40));
            }
        }

        /* ----------- GAME OVER SEQUENCE ----------- */

        // play tune in sound task
        Sound_Send(SFX_GAMEOVER_TUNE);

        // while tune is playing, we can flash TIME'S / UP! on LCD
        FlashTimesUp();


        // final score screen
        LCD_Clear();
        char line[17];

        // line 0: total score
        snprintf(line, sizeof(line), "Score: %6lu", (unsigned long)totalScore);
        LCD_SetCursor(0, 0);
        LCD_Print(line);

        // line 1: fruits slayed
        snprintf(line, sizeof(line), "Fruits: %3lu", (unsigned long)fruitsSlayed);
        LCD_SetCursor(1, 0);
        LCD_Print(line);

        // make sure all LEDs are off
        for (int i = 0; i < 4; i++)
        {
            HAL_GPIO_WritePin(ledPorts[i], ledPins[i], GPIO_PIN_RESET);
        }

        // let player stare at their glory for a bit
        vTaskDelay(pdMS_TO_TICKS(3000));
        // then loop back and wait for next button press (idle screen)
    }

}
/* USER CODE END Application */
