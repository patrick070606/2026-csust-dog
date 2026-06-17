# Track Blue Boost Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an independent STM32 test firmware that starts like the main firmware, follows line tracking like the main firmware, and switches to higher step height and longer stride when blue is detected.

**Architecture:** Keep the main firmware and stair test firmware untouched at runtime by adding a separate CMake target. Reuse `image_command`, `dog_gait`, and servo initialization paths, but keep the behavior focused on line tracking plus blue-triggered gait boost.

**Tech Stack:** STM32 HAL C firmware, CMake/Ninja, existing dog gait and image command modules.

---

### Task 1: Add Test Program

**Files:**
- Create: `User/track_blue_boost_test_program.h`
- Create: `User/track_blue_boost_test_program.c`

- [x] **Step 1: Define the test interface**

Expose `TrackBlueBoostTest_Init()` and `TrackBlueBoostTest_Run()`.

- [x] **Step 2: Implement startup matching main firmware**

Use the same center, wait, stand, payload load mode, and vision init flow as `DogTask_Init()`.

- [x] **Step 3: Implement main tracking logic**

Use the same deadband, recover window, steer gain, max steer, gait period, move time, speed frequency, and left/right track stride values as the main firmware.

- [x] **Step 4: Implement blue-triggered boost**

When `IMAGE_COMMAND_PLATFORM` is received, set a persistent boost flag. Tracking continues with step height `30.0f`, left stride `30.0f`, and right stride `25.0f`.

### Task 2: Add Independent Firmware Entry

**Files:**
- Create: `Core/Src/main_track_blue_boost_test.c`
- Modify: `CMakeLists.txt`

- [x] **Step 1: Add a standalone main**

Initialize HAL, GPIO, TIM1, USART1, and USART2, then run `TrackBlueBoostTest`.

- [x] **Step 2: Add CMake target**

Create `DogRobotTrackBlueBoostTest` with the shared HAL, servo, gait, image command, and new test program sources.

### Task 3: Add Verification

**Files:**
- Create: `tests/check_track_blue_boost_test_program.py`

- [x] **Step 1: Add static checks**

Check startup constants, tracking constants, boost constants, blue command handling, and CMake target wiring.

- [x] **Step 2: Build the target**

Run `cmake --build build\codex-debug --target DogRobotTrackBlueBoostTest`.
