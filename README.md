# Hybrid Dewatering System (Solar - Battery - Grid - Generator)

**Repository:** Hybrid Dewatering System for Copper Mining — STM32 (FreeRTOS) + Optional PLC (Siemens S7-1500)


---

## Quick summary
A resilient hybrid power-control system for dewatering pumps in remote mining sites. The design prioritizes renewable energy (solar + battery), falls back to grid power at night, and uses a generator as a tertiary backup. The system integrates intelligent switching logic, VFD-driven pump control (soft-start/soft-stop & RPM control), solar-panel dust detection with automated cleaning, and full telemetry via CAN / Ethernet.

This repository contains firmware examples for an STM32-based controller using FreeRTOS and supporting documentation / PLC templates for Siemens S7-1500.


---

## Table of contents
1. [Highlights & Invention](#highlights--invention)
2. [System architecture](#system-architecture)
3. [Hardware — recommended BOM & wiring](#hardware--recommended-bom--wiring)
4. [Software architecture & repo layout](#software-architecture--repo-layout)
5. [STM32 (FreeRTOS) — Build & Flash Quickstart](#stm32-freertos---build--flash-quickstart)
6. [Siemens S7-1500 PLC — Overview & import](#siemens-s7-1500-plc---overview--import)
7. [HMI / SCADA integration & telemetry](#hmi--scada-integration--telemetry)
8. [Commissioning & test procedures](#commissioning--test-procedures)
9. [Safety & best practices](#safety--best-practices)
10. [Troubleshooting & FAQs](#troubleshooting--faqs)
11. [Contributing & roadmap](#contributing--roadmap)
12. [License & credits](#license--credits)


---

## Highlights & Invention
- **Smart hybrid power manager**: priority switching logic that selects the greenest/most economical source (Solar → Battery → Grid → Generator) with anti-chatter and safe contactor sequencing.
- **Self-maintaining solar PV**: dust detection sensor + automated spray cleaning subsystem to reduce soiling losses — essential in dusty mining environments.
- **Adaptive pump control**: VFD-driven RPM control based on water demand, power availability and protective soft-start/stop sequences to reduce mechanical stress and energy consumption.
- **Edge telemetry**: STM32 collects PV, BMS, VFD and generator metrics via ADC, CAN and Ethernet — local HMI + optional cloud telemetry via MQTT / Modbus TCP.


---

## System architecture
(See `/assets/block_diagram.png` for the visual block diagram.)

High-level components:
- **Solar PV array** with MPPT and per-string sensing
- **Battery bank** with BMS (CAN/RS485) for SOC and protection
- **Inverter / DC-DC** to provide AC to the AC bus or VFD depending on VFD type
- **Grid & Generator** as AC sources connected to AC Main Bus via contactors
- **VFD + Dewatering Pump** with RPM & current sensing
- **Control Unit**: STM32 (main) running FreeRTOS, CAN + Ethernet + ADC inputs
- **Optional PLC**: Siemens S7-1500 — templates included for sites that require PLC-class hardware
- **HMI**: Local touchscreen (Profinet / Ethernet) + remote SCADA (MQTT / OPC UA optional)


---

## Hardware — recommended BOM & wiring
> **Note:** This is a guideline. Validate ratings (voltages, currents, safety standards) for your specific site.

### Suggested components (example)
- Solar PV panels (array configuration to meet site kW) + MPPT controller(s)
- Battery bank (Li-ion or lead acid) + Battery Management System (BMS) with CAN/RS485
- Inverter or bi-directional DC-DC (match battery & pump/VFD requirements)
- VFD rated for pump motor (specify motor rating + safety margins)
- STM32 development board (STM32F7 / STM32H7 recommended) with: ADC channels, 2x CAN, Ethernet PHY, SPI/I2C, UART
- Contactors & interlocking relays for Solar/Batt/Grid/Gen (rated to system current)
- Generator with remote start capability (starter relay input) and run feedback
- Dust sensor (optical / capacitive) for solar soiling detection
- Spray solenoid + plumbing for panel cleaning
- HMI touchscreen (Ethernet) or Pi-based HMI
- Current shunts / Hall-effect sensors for measuring pump and PV currents
- Isolation transformers / surge arrestors and proper grounding

### Example minimal wiring map (simplified)
- ADC inputs: PV voltage, PV current (shunt), Battery voltage, Pump current
- Digital inputs: Grid detect, Genset run feedback, Dust detect, E-STOP
- Digital outputs: Contactor solar, contactor battery, contactor grid, genset start, spray valve, tracker motors
- CAN: BMS (SOC, cell voltages)
- Ethernet: Optional Modbus TCP to VFD & HMI / MQTT gateway

Refer to `/docs/io_mapping.md` for a ready-to-copy I/O mapping table (example tags available for STM32 & S7-1500).


---

## Software architecture & repo layout
```
/ (root)
├─ /stm32/                    # STM32CubeIDE project & FreeRTOS firmware
│   ├─ /src/
│   ├─ /inc/
│   ├─ /docs/                  # STM32-specific notes and memory map
│   └─ README.md
├─ /plc/                      # Siemens S7-1500 Structured Text (SCL) templates
│   ├─ FB_PowerManager.scl
│   ├─ FB_GeneratorController.scl
│   └─ DB_Config.xlsx
├─ /assets/                   # diagrams, PNG/SVG for slides
├─ /docs/                     # commissioning, IO mapping, tests and checklists
├─ LICENSE
└─ README.md
```

### Important modules (STM32 firmware)
- `TASK_SensorAcq` — ADC, digital inputs, CAN BMS reads
- `TASK_PowerManager` — source selection logic & contactor sequencing
- `TASK_GeneratorController` — generator start/stop sequence with retries
- `TASK_VFDController` — setpoint translation & VFD comms (Modbus TCP/RTU)
- `TASK_SolarCleaner` — dust detection & spray logic
- `TASK_AlarmManager` — alarms, HMI notifications
- `TASK_Communication` — telemetry publishing (MQTT / Modbus TCP registers)


---

## STM32 (FreeRTOS) — Build & Flash Quickstart
**Prerequisites**: STM32CubeIDE (or Make toolchain), ST-Link, power-supply and hardware wired.

1. Clone repository: `git clone <repo-url>`
2. Open `stm32` folder in STM32CubeIDE: `File -> Open Projects from File System` → select `/stm32` project.
3. Check `MX_*.c` peripheral config (ADC channels, CAN, Ethernet). Adjust `CubeMX` .ioc if using a different board.
4. Build: `Project -> Build All`.
5. Connect ST-Link; flash: `Run -> Debug` (or `Run -> Run` to run without debugger).
6. Configure runtime thresholds via HMI or modify `config.h` and rebuild.

**Notes:**
- The code uses FreeRTOS tasks with mutexes and queues — tune stack sizes & priorities for your CPU.
- Provide a hardware IWDG or use the HAL watchdog; ensure the watchdog is fed by a monitored task.


---

## Siemens S7-1500 PLC — Overview & import
This repo includes SCL templates and DB suggestions intended for TIA Portal users.

1. Import the `.scl` files into your TIA Portal project (create a CPU 1516-3 PN/DP or similar).
2. Create DBs: `DB_SystemStatus`, `DB_PowerStats`, `DB_Config` and map the I/O to your hardware.
3. Compile and download to the CPU. Use HMI screens for operator control and monitoring.

The S7-1500 templates mirror the STM32 FreeRTOS logic and are intended for teams who must use PLC-class hardware in production.


---

## HMI / SCADA integration & telemetry
- **Local HMI**: Create HMI screens showing: Selected Source, PV kW, Battery SOC, Grid status, Generator status, VFD RUN % and Alarms. HMI should allow manual override for generator start/stop and power forcing.
- **Remote telemetry**: Support for Modbus TCP registers or MQTT topics. Suggested telemetry payloads: `pv_power_kW`, `batt_soc`, `grid_voltage`, `pump_current`, `selected_source`, `alarms[]`.
- **Data logging**: Store rolling logs on SD card (FatFS) for incident investigations.


---

## Commissioning & test procedures
Follow the commissioning checklist in `/docs/commissioning_checklist.md`. Key scenarios to test:
1. **Daytime Solar**: Full solar available — system should select solar, battery charging, pump running from solar.
2. **Evening / Low PV**: PV falls below threshold — battery supplies pump; SOC monitored & alarm if critical.
3. **Grid fallback**: At night, if configured, grid takes over when battery low.
4. **Generator auto-start**: Force grid & battery fail — system should request auto-generator start (verify safety interlocks before allowing auto-start).
5. **VFD control**: Start/stop & speed setpoints while measuring motor current and verifying soft-start/stop responses.
6. **Solar cleaning**: Trigger dust sensor & validate spray cycle; ensure cleaning suppressed during safety conditions.

Run end-to-end tests with telemetry enabled and record logs.


---

## Safety & best practices
- Use **hardware interlocks** for contactors — never rely only on software to prevent paralleling sources.
- Implement **redundant sensing** for critical measurements (grid detect, generator run feedback).
- Ensure **earthing / grounding** and surge protection are implemented per local regulations.
- Have an **emergency-stop** circuit that disables all automatic switching and stops the pump.
- Adhere to local electrical codes for battery enclosures and genset installations.


---

## Troubleshooting & FAQs
- **Generator won’t start**: verify starter relay wiring and battery/charger on genset — check `TASK_GeneratorController` log and retries.
- **Frequent source switching**: tune the hysteresis & anti-chatter timers in `DB_Config` (PV min power, SOC hysteresis, switch delay).
- **VFD faults on start**: check rated voltage/frequency, motor wiring, and VFD parameter settings (ramp times, protective thresholds).


---

## Contribution, roadmap & governance
Contributions welcome! Suggested way to contribute:
1. Fork the repo
2. Create a feature branch `feature/<name>`
3. Submit a pull request with description and testing steps

Planned roadmap (next iterations):
- Add automated data analytics for energy optimization
- Improve VFD control loops to include PID-based water-level control
- Add redundant PLC path and functional safety module


---

## License & credits
This project is released under the **MIT License**. See `LICENSE` in the repository.

If you use this project in academic work, please cite the repo and list this team as authors.


---

## Where to find files in this repo
- `/stm32` — STM32CubeIDE project
- `/plc` — SCL/DB templates for Siemens S7-1500
- `/assets` — block diagram and slide images
- `/docs` — commissioning checklist, I/O mapping, and detailed testing scenarios


---

If you want, I can also:
- generate a ready-to-use `LICENSE` file (MIT) in the repo,
- export this README as `README.md` in the canvas so you can copy it to your GitHub repo,
- create a `release` ZIP containing a minimal STM32CubeIDE skeleton and the S7-1500 SCL files.

