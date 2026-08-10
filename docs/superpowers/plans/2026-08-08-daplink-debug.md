# DAPLink Debugging Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add a VS Code build and Cortex-Debug launch workflow that debugs the STM32H723ZGTx through a DAPLink CMSIS-DAP probe over SWD.

**Architecture:** Keep the existing STM32CubeIDE for VS Code CMake project and generated firmware unchanged. Add two CMake tasks (configure and build) and one Cortex-Debug launch profile that starts OpenOCD with CMSIS-DAP/SWD and loads the Debug ELF. Tool executable names are configurable through project-local VS Code settings while defaulting to PATH resolution.

**Tech Stack:** VS Code tasks, Cortex-Debug, OpenOCD CMSIS-DAP, Arm GNU GDB, CMake Presets, Ninja.

---

### Task 1: Add configurable tool defaults

**Files:**
- Modify: `.vscode/settings.json`

- [ ] **Step 1: Preserve existing STM32CubeIDE settings and add debugger tool keys**

Append these settings to the existing JSON object:

```json
    "stm32cubeide.openocdPath": "openocd",
    "stm32cubeide.gdbPath": "arm-none-eabi-gdb"
```

The values intentionally use executable names so installations that put OpenOCD and Arm GDB on `PATH` work without edits. Users with non-standard installs can replace either value in this workspace setting.

- [ ] **Step 2: Validate JSON syntax**

Run:

```powershell
Get-Content -Raw .vscode/settings.json | ConvertFrom-Json | Out-Null
```

Expected: command exits successfully with no parser error.

### Task 2: Add reproducible Debug build tasks

**Files:**
- Create: `.vscode/tasks.json`

- [ ] **Step 1: Create the configure task**

Use a shell task rooted at `${workspaceFolder}`:

```json
{
    "label": "CMake: Configure Debug",
    "type": "shell",
    "command": "cmake",
    "args": ["--preset", "Debug"],
    "options": {"cwd": "${workspaceFolder}"},
    "problemMatcher": []
}
```

- [ ] **Step 2: Create the build task depending on configure**

Add a second task:

```json
{
    "label": "CMake: Build Debug",
    "type": "shell",
    "command": "cmake",
    "args": ["--build", "--preset", "Debug"],
    "options": {"cwd": "${workspaceFolder}"},
    "dependsOn": "CMake: Configure Debug",
    "dependsOrder": "sequence",
    "problemMatcher": ["$gcc"],
    "group": {"kind": "build", "isDefault": true}
}
```

- [ ] **Step 3: Validate task JSON**

Run:

```powershell
Get-Content -Raw .vscode/tasks.json | ConvertFrom-Json | Out-Null
```

Expected: command exits successfully with no parser error.

### Task 3: Add the DAPLink Cortex-Debug profile

**Files:**
- Create: `.vscode/launch.json`

- [ ] **Step 1: Create the OpenOCD CMSIS-DAP launch configuration**

Add this configuration:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "DAPLink: STM32H723 (SWD)",
            "type": "cortex-debug",
            "request": "launch",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/build/Debug/26rcf.elf",
            "servertype": "openocd",
            "serverpath": "${config:stm32cubeide.openocdPath}",
            "gdbPath": "${config:stm32cubeide.gdbPath}",
            "configFiles": [
                "interface/cmsis-dap.cfg",
                "target/stm32h7x.cfg"
            ],
            "openOCDLaunchCommands": [
                "transport select swd",
                "adapter speed 4000"
            ],
            "device": "STM32H723ZGTx",
            "runToEntryPoint": "main",
            "preLaunchTask": "CMake: Build Debug",
            "showDevDebugOutput": "raw"
        }
    ]
}
```

`configFiles` selects the standard OpenOCD CMSIS-DAP and STM32H7 scripts. The launch command explicitly selects SWD and uses 4 MHz as a stable default for DAPLink.

- [ ] **Step 2: Validate launch JSON**

Run:

```powershell
Get-Content -Raw .vscode/launch.json | ConvertFrom-Json | Out-Null
```

Expected: command exits successfully with no parser error.

### Task 4: Verify build and OpenOCD script resolution

**Files:**
- Verify: `build/Debug/26rcf.elf`

- [ ] **Step 1: Run the Debug CMake build**

Run:

```powershell
cmake --build --preset Debug
```

Expected: build completes successfully and `build/Debug/26rcf.elf` exists.

- [ ] **Step 2: Check OpenOCD can resolve both probe and target scripts**

Run:

```powershell
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg -c "transport select swd" -c "adapter speed 4000" -c "init; shutdown"
```

Expected with no probe connected: OpenOCD parses both scripts and then reports a CMSIS-DAP connection failure during `init`; an error about missing `interface/cmsis-dap.cfg` or `target/stm32h7x.cfg` indicates an installation/path problem that must be fixed through `stm32cubeide.openocdPath` or the OpenOCD installation.

- [ ] **Step 3: Confirm the VS Code profile contract**

Open Run and Debug, select `DAPLink: STM32H723 (SWD)`, and start it with a connected DAPLink probe. Expected: the pre-launch build runs, OpenOCD starts, Cortex-Debug connects over SWD, the ELF is loaded, and execution stops at `main`.
