/*
STM32 + FreeRTOS Implementation
Project: Hybrid Solar / Battery / Grid / Generator Dewatering System
Target: STM32 (recommended: STM32F7 / STM32H7 family for Ethernet + CAN + performance)
Build: STM32CubeIDE (CubeMX generated HAL + FreeRTOS + lwIP + FatFS if needed)

This file is a design + code-snippet collection to implement the same functionality
as the Siemens S7-1500 PLC design but on an STM32 using FreeRTOS.

Contents:
 - System assumptions & recommended hardware
 - Peripheral & IO mapping suggestions
 - RTOS architecture (tasks, priorities, synchronization primitives)
 - Data structures / messaging / Event Groups
 - Task pseudocode and full sample C functions
 - Communication patterns (CAN for BMS, Modbus TCP for VFD, lwIP, MQTT optional)
 - Safety & watchdog
 - Commissioning & tuning tips

---

ASSUMPTIONS & HARDWARE RECOMMENDATIONS
 - MCU: STM32F746 / STM32F767 / STM32H743 (Ethernet MAC + 2x CAN + plenty of RAM/Flash)
 - Peripherals used: ADCs (PV, battery, pump current), SPI/I2C sensors (dust), GPIOs (contactor outputs, genset start), PWM (tracker motor), UART (RS485 for Modbus RTU if needed), Ethernet (Modbus TCP / MQTT), CAN (BMS)
 - RTOS: FreeRTOS (use CubeMX to scaffold tasks)
 - Middleware: lwIP (Ethernet), FatFS+SD (logging), LwMQTT or MQTT over TLS optional
 - VFD: Modbus TCP or Modbus RTU on RS485. If VFD supports Profinet, use appropriate stack.

IO MAPPING (example GPIO/ADC names, adapt per your board)
 - DI_PV_ok (digital)           : GPIO_IN (PV panel OK threshold via comparator or from ADC logic)
 - DI_Grid_Present              : GPIO_IN (grid relay/voltage detector)
 - DI_Gen_RunFeedback           : GPIO_IN
 - DI_EStop                     : GPIO_IN (emergency stop)
 - DI_Solar_Dust                : GPIO_IN (dust sensor digital output)
 - AI_PV_Voltage                : ADC1_IN0
 - AI_PV_Current                : ADC1_IN1
 - AI_Batt_Voltage              : ADC1_IN2
 - AI_Batt_SOC                  : from CAN (preferred) or ADC
 - AI_Grid_Voltage              : ADC1_IN3
 - AI_Grid_Frequency            : measured using input capture / freq sensor
 - AI_Pump_Current              : ADC1_IN4
 - DO_Contactor_Solar           : GPIO_OUT
 - DO_Contactor_Battery         : GPIO_OUT
 - DO_Contactor_Grid            : GPIO_OUT
 - DO_Genset_Start              : GPIO_OUT (pulse)
 - DO_Genset_Stop               : GPIO_OUT
 - DO_Spray_Cleaner             : GPIO_OUT
 - DO_Tracker_CW/CCW           : GPIO_OUT / PWM for stepper or DC motor
 - DO_VFD_RunCmd               : (via Modbus write) or digital output depending on VFD

RTOS ARCHITECTURE
 - Use multiple periodic tasks and dedicated comm tasks. Protect shared state with mutexes.
 - Primary tasks (suggested priorities & periods):
   * TASK_SensorAcq (priority high, 100-200 ms) - read ADCs, CAN BMS, update shared state
   * TASK_PowerManager (priority high, 500 ms - 1 s) - implements the energy source priority logic
   * TASK_GeneratorController (priority above normal, event-driven) - handle start sequence
   * TASK_VFDController (priority normal, 200-500 ms) - communicate setpoints to VFD
   * TASK_SolarCleaner (priority low, event-driven) - dust detection & spray
   * TASK_AlarmManager (priority high, event-driven) - manage alarms & HMI
   * TASK_Communication (priority normal, 200-500 ms) - CAN/Ethernet/Cloud comms
   * TASK_HMI (priority low, 500 ms) - update local display

 - Synchronization: mutex for SystemState, queues for commands (e.g., generator start requests), EventGroup for global flags (EStop, Fault)
 - Watchdog: HW independent IWDG + feed in a high-priority task or monitor.

DATA STRUCTURES
 Use a single shared state structure protected by a mutex. Also define command queues.

typedef enum { SRC_NONE=0, SRC_SOLAR, SRC_BATTERY, SRC_GRID, SRC_GENERATOR } supply_source_t;

typedef struct {
  float pv_voltage;
  float pv_current;
  float pv_power_kw;
  float batt_voltage;
  float batt_soc;
  float grid_voltage;
  float grid_freq;
  float pump_current;
  bool grid_present;
  bool genset_running;
  bool dust_detected;
  bool estop;
  supply_source_t selected_source;
  char reason[64];
} system_state_t;

// Command queue item for generator control
typedef struct {
  bool auto_start_request; // true = auto start request
  bool manual_start;      // true = manual start
} gen_cmd_t;

// Shared handles
SemaphoreHandle_t xStateMutex;
QueueHandle_t xGenCmdQueue;
EventGroupHandle_t xGlobalEvents; // bits: ESTOP_BIT, FAULT_BIT etc.

// Event bits
#define ESTOP_BIT (1<<0)
#define FAULT_BIT (1<<1)

// Initialize shared objects in system init

/* ========== Example Code Snippets ========== */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "cmsis_os.h" // optional wrapper
#include "stm32f7xx_hal.h"
#include <string.h>

system_state_t gState;

void SystemState_Init(void) {
  memset(&gState,0,sizeof(gState));
  gState.selected_source = SRC_NONE;
}

void SharedObjects_Create(void) {
  xStateMutex = xSemaphoreCreateMutex();
  xGenCmdQueue = xQueueCreate(4,sizeof(gen_cmd_t));
  xGlobalEvents = xEventGroupCreate();
}

/* -------- Sensor Acquisition Task -------- */
void TASK_SensorAcq(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(200);
  for(;;) {
    // Read ADCs via HAL_ADC, read CAN for BMS, read digital inputs
    float pv_v = Read_ADC_PV_Voltage();
    float pv_i = Read_ADC_PV_Current();
    float batt_v = Read_ADC_Batt_Voltage();
    float grid_v = Read_ADC_Grid_Voltage();
    float pump_i = Read_ADC_Pump_Current();
    bool dust = Read_GPIO_DUST();
    bool grid_present = Read_GPIO_GridPresent();
    bool gen_run = Read_GPIO_GenRun();
    bool estop = Read_GPIO_ESTOP();

    xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(50));
    gState.pv_voltage = pv_v;
    gState.pv_current = pv_i;
    gState.pv_power_kw = (pv_v * pv_i) / 1000.0f;
    gState.batt_voltage = batt_v;
    // BMS SOC typically via CAN - read and set batt_soc
    gState.grid_voltage = grid_v;
    gState.pump_current = pump_i;
    gState.dust_detected = dust;
    gState.grid_present = grid_present;
    gState.genset_running = gen_run;
    gState.estop = estop;
    xSemaphoreGive(xStateMutex);

    // set global event for estop
    if (estop) xEventGroupSetBits(xGlobalEvents, ESTOP_BIT);
    else xEventGroupClearBits(xGlobalEvents, ESTOP_BIT);

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/* -------- Power Manager Task (priority & logic) -------- */
void TASK_PowerManager(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);

  for(;;) {
    // read shared state snapshot
    system_state_t snapshot;
    xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(100));
    memcpy(&snapshot, &gState, sizeof(snapshot));
    xSemaphoreGive(xStateMutex);

    // decision thresholds (tune in config or via HMI)
    const float PV_MIN_POWER = 0.5f; // kW
    const float BATT_SOC_MIN = 30.0f; // percent
    bool pv_ok = (snapshot.pv_power_kw >= PV_MIN_POWER) && (snapshot.pv_voltage > 40.0f);
    bool batt_ok = (snapshot.batt_soc >= BATT_SOC_MIN);
    bool grid_ok = snapshot.grid_present && (snapshot.grid_voltage > 200.0f);

    supply_source_t chosen = SRC_NONE;
    char reason[64] = {0};

    if (pv_ok) {
      chosen = SRC_SOLAR;
      strcpy(reason, "Solar available");
    } else if (batt_ok) {
      chosen = SRC_BATTERY;
      strcpy(reason, "Battery SOC sufficient");
    } else if (grid_ok) {
      chosen = SRC_GRID;
      strcpy(reason, "Grid available");
    } else if (snapshot.genset_running) {
      chosen = SRC_GENERATOR;
      strcpy(reason, "Generator running");
    } else {
      chosen = SRC_NONE;
      strcpy(reason, "No source");
    }

    // If no source & auto-gen enabled -> request generator start
    bool auto_gen_allowed = true; // from config/HMI
    if ((chosen == SRC_NONE) && auto_gen_allowed && !snapshot.genset_running) {
      gen_cmd_t cmd = {.auto_start_request = true, .manual_start = false};
      xQueueSend(xGenCmdQueue, &cmd, 0);
      strcpy(reason, "Requesting generator start");
    }

    // Apply contactor commands via safe switching (open old, wait, close new)
    // For simplicity: use atomic change with interlock delays handled by output function
    ApplyPowerSource(chosen);

    // update shared state
    xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(50));
    gState.selected_source = chosen;
    strncpy(gState.reason, reason, sizeof(gState.reason)-1);
    xSemaphoreGive(xStateMutex);

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/* -------- Generator Controller Task -------- */
void TASK_GeneratorController(void *pvParameters) {
  gen_cmd_t cmd;
  for(;;) {
    if (xQueueReceive(xGenCmdQueue, &cmd, pdMS_TO_TICKS(1000)) == pdPASS) {
      if (cmd.auto_start_request || cmd.manual_start) {
        // safety checks
        EventBits_t bits = xEventGroupGetBits(xGlobalEvents);
        if (bits & ESTOP_BIT) continue; // do not start if E-Stop

        // start sequence: pulse start relay, wait for feedback
        HAL_GPIO_WritePin(GPIO_GEN_START_PORT, GPIO_GEN_START_PIN, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(3000)); // pulse length
        HAL_GPIO_WritePin(GPIO_GEN_START_PORT, GPIO_GEN_START_PIN, GPIO_PIN_RESET);

        // wait & check run feedback for up to X seconds
        const TickType_t timeout = pdMS_TO_TICKS(10000);
        TickType_t start = xTaskGetTickCount();
        bool started = false;
        while ((xTaskGetTickCount() - start) < timeout) {
          if (Read_GPIO_GenRun()) { started = true; break; }
          vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (!started) {
          // retry logic or send alarm
          // send to alarm task or set fault event
          xEventGroupSetBits(xGlobalEvents, FAULT_BIT);
        }
      }
    }
  }
}

/* -------- VFD Controller Task (example Modbus TCP) -------- */
void TASK_VFDController(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(500);
  for(;;) {
    // read desired speed from HMI or PID loop
    float desired_pct = Read_HMI_SpeedDemand(); // 0..100

    // example: only enable VFD if selected source is valid and not estop
    xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(50));
    supply_source_t sel = gState.selected_source;
    bool estop = gState.estop;
    xSemaphoreGive(xStateMutex);

    if (!estop && (sel != SRC_NONE)) {
      // compute setpoint and write to VFD via Modbus TCP/RTU
      VFD_WriteSpeed(desired_pct);
      VFD_SetRun(true);
    } else {
      VFD_SetRun(false);
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/* -------- Solar Cleaner Task -------- */
void TASK_SolarCleaner(void *pvParameters) {
  const TickType_t checkPeriod = pdMS_TO_TICKS(10000);
  for(;;) {
    xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(50));
    bool dust = gState.dust_detected;
    bool waterAvailable = true; // read flag or sensor
    bool estop = gState.estop;
    xSemaphoreGive(xStateMutex);

    if (dust && waterAvailable && !estop) {
      // trigger spray for defined pulse
      HAL_GPIO_WritePin(GPIO_SPRAY_PORT, GPIO_SPRAY_PIN, GPIO_PIN_SET);
      vTaskDelay(pdMS_TO_TICKS(2000)); // spray 2s (tune)
      HAL_GPIO_WritePin(GPIO_SPRAY_PORT, GPIO_SPRAY_PIN, GPIO_PIN_RESET);
    }
    vTaskDelay(checkPeriod);
  }
}

/* -------- Alarm Manager Task -------- */
void TASK_AlarmManager(void *pvParameters) {
  for(;;) {
    EventBits_t bits = xEventGroupWaitBits(xGlobalEvents, FAULT_BIT|ESTOP_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(500));
    if (bits & ESTOP_BIT) {
      // activate buzzer and HMI
      HAL_GPIO_WritePin(GPIO_BUZZER_PORT, GPIO_BUZZER_PIN, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(GPIO_BUZZER_PORT, GPIO_BUZZER_PIN, GPIO_PIN_RESET);
    }
    if (bits & FAULT_BIT) {
      // post to HMI logs and alert
      HMI_PostAlarm("System Fault - check logs");
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

/* -------- Communication Task (CAN + Ethernet) -------- */
void TASK_Communication(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(500);
  for(;;) {
    // Handle CAN messages from BMS
    CAN_HandleMessages();
    // Publish telemetry via MQTT/HTTP or ModbusTCP registers
    PublishTelemetry(&gState);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/* ========== Helper functions (simplified) ========== */
void ApplyPowerSource(supply_source_t src) {
  // DOs must be toggled with interlock and safe delay
  static supply_source_t last = SRC_NONE;
  if (src == last) return;

  // Open all contactors first
  HAL_GPIO_WritePin(GPIO_SOLAR_CONTACTOR_PORT, GPIO_SOLAR_CONTACTOR_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIO_BATT_CONTACTOR_PORT, GPIO_BATT_CONTACTOR_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIO_GRID_CONTACTOR_PORT, GPIO_GRID_CONTACTOR_PIN, GPIO_PIN_RESET);
  vTaskDelay(pdMS_TO_TICKS(500));

  // Close desired contactor
  switch(src) {
    case SRC_SOLAR:
      HAL_GPIO_WritePin(GPIO_SOLAR_CONTACTOR_PORT, GPIO_SOLAR_CONTACTOR_PIN, GPIO_PIN_SET);
      break;
    case SRC_BATTERY:
      HAL_GPIO_WritePin(GPIO_BATT_CONTACTOR_PORT, GPIO_BATT_CONTACTOR_PIN, GPIO_PIN_SET);
      break;
    case SRC_GRID:
      HAL_GPIO_WritePin(GPIO_GRID_CONTACTOR_PORT, GPIO_GRID_CONTACTOR_PIN, GPIO_PIN_SET);
      break;
    case SRC_GENERATOR:
    case SRC_NONE:
    default:
      // leave all open (generator provides power outside MCU control)
      break;
  }
  last = src;
}

/* Stub functions to integrate with actual hardware */
float Read_ADC_PV_Voltage(void) { return 600.0f; } // replace with real ADC read & scaling
float Read_ADC_PV_Current(void) { return 2.0f; }
float Read_ADC_Batt_Voltage(void) { return 48.0f; }
bool Read_GPIO_DUST(void) { return false; }
bool Read_GPIO_GridPresent(void) { return true; }
bool Read_GPIO_GenRun(void) { return false; }
bool Read_GPIO_ESTOP(void) { return false; }

void VFD_WriteSpeed(float pct) { /* implement Modbus write */ }
void VFD_SetRun(bool run) { /* implement run/stop command */ }

void HMI_PostAlarm(const char* msg) { /* send to local HMI */ }
void PublishTelemetry(system_state_t* s) { /* send via MQTT or Modbus TCP registers */ }
void CAN_HandleMessages(void) { /* read CAN and update gState.batt_soc etc */ }

/* ========== main.c (initialization & task creation) ========== */
int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_ETH_Init(); // lwIP
  MX_USARTx_UART_Init(); // RS485 if needed

  SystemState_Init();
  SharedObjects_Create();

  // Create tasks (stack sizes & priorities must be tuned)
  xTaskCreate(TASK_SensorAcq, "SensorAcq", 512, NULL, 6, NULL);
  xTaskCreate(TASK_PowerManager, "PowerMgr", 512, NULL, 7, NULL);
  xTaskCreate(TASK_GeneratorController, "GenCtrl", 256, NULL, 5, NULL);
  xTaskCreate(TASK_VFDController, "VFD", 512, NULL, 4, NULL);
  xTaskCreate(TASK_SolarCleaner, "Cleaner", 256, NULL, 2, NULL);
  xTaskCreate(TASK_AlarmManager, "Alarm", 256, NULL, 8, NULL);
  xTaskCreate(TASK_Communication, "Comm", 768, NULL, 3, NULL);
  xTaskCreate(TASK_HMI, "HMI", 512, NULL, 1, NULL);

  // Start scheduler
  vTaskStartScheduler();

  // Should never reach here
  while(1) {}
}

/* ========== Safety & Best Practices ========== */
// - Use hardware interlocks for critical contactor logic; software should complement hardware.
// - Implement CRC and sequence checks for commands to VFD and genset.
// - Use IWDG to reset MCU on hangs; feed IWDG from a monitored high-priority task.
// - Store configuration (thresholds, timers) in EEPROM/flash and provide HMI knobs.
// - Ensure EMI and isolation for ADC measurements around high-voltage/AC circuits.
// - Test in simulation before connecting real actuators; use smoke tests for contactor control.

/* ========== Commissioning Checklist ========== */
// - Verify ADC calibration & scaling for each sensor (voltage dividers, shunt amplifiers)
// - Validate CAN communication with BMS (message IDs, rates)
// - Validate Modbus comms with VFD: setpoint register & run/stop
// - Validate generator start/stop pulse timing with vendor's spec
// - Test contactor interlocks: ensure never more than one main contactor closed
// - Verify solar cleaner doesn't trigger during rain/watering failure
// - Enable verbose logging & remote telemetry for at least initial commissioning

/* End of document */
