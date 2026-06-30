# DAPLink 使用说明

本文说明第一次在新电脑上使用当前工程时，如何通过 DAPLink 编译、下载、运行和调试 STM32F103。

当前工程目标芯片为 STM32F103C8Tx，主要产物为：

```text
build/Debug/DogRobot.elf
```

## 1. 需要准备

硬件：

- STM32F103C8Tx 目标板
- DAPLink，普通有线 DAPLink 或装甲版无线 DAPLink
- USB 数据线，必须是可传数据的线
- SWD 连接线

软件：

- VSCode
- VSCode 插件：C/C++、Cortex-Debug、CMake Tools
- CMake、Ninja
- ARM GCC 工具链
- OpenOCD

确认工程能编译：

```powershell
cd C:\Coding\STM32\Myproject_vscode\2026-csust-dog
cmake --preset Debug
cmake --build --preset Debug --target DogRobot
```

如果输出类似：

```text
ninja: no work to do.
```

说明当前目标已经是最新的，这不是错误。

## 2. 安装和检查 OpenOCD

可以使用 winget 安装 xPack OpenOCD：

```powershell
winget install --id xpack-dev-tools.openocd-xpack --source winget
```

安装后重新打开 PowerShell，检查：

```powershell
openocd --version
```

如果找不到 `openocd`，检查安装目录是否已加入 Path。常见路径类似：

```text
C:\Users\12403\AppData\Local\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe\xpack-openocd-0.12.0-7\bin
```

当前工程的 `.vscode/launch.json` 和 `.vscode/tasks.json` 使用通用命令 `openocd`，不会写死某一台电脑的安装路径。因此新电脑必须先确认下面命令可用：

```powershell
openocd --version
```

如果 VSCode 中找不到 OpenOCD，通常是因为 OpenOCD 的 `bin` 目录没有加入 Path，或 VSCode 还没有重启刷新环境变量。

## 3. 有线 DAPLink 接线

有线调试时，只需要一个 DAPLink：

```text
电脑 USB -> DAPLink -> STM32
```

SWD 接线：

```text
DAPLink GND   -> STM32 GND
DAPLink SWDIO -> STM32 PA13 / SWDIO
DAPLink SWCLK -> STM32 PA14 / SWCLK
DAPLink 3V3   -> STM32 3.3V / VTref
DAPLink NRST  -> STM32 NRST，可选但建议接
```

注意：

- GND 必须共地。
- 目标板必须有 3.3V。
- 如果目标板已经单独供电，不要让多个电源互相冲突。

测试连接：

```powershell
openocd -f interface/cmsis-dap.cfg -f target/stm32f1x.cfg
```

成功时会看到：

```text
SWD DPIDR 0x1ba01477
Cortex-M3 r1p1 processor detected
Examination succeed
Listening on port 3333 for gdb connections
```

OpenOCD 停在这里是正常的，它在等待 GDB 连接。按 `Ctrl + C` 可以退出。

## 4. 装甲版无线 DAPLink 模式

装甲版无线 DAPLink 有三种主要模式：

```text
红色 = 有线 USB 模式
蓝色 = 无线 Host，插电脑
绿色 = 无线 Slave，接 STM32
黄色 = 设置/配对模式
```

设置模式：

```text
长按 A -> 灯变黄色，进入设置模式
短按 B -> 切换模式
长按 A -> 黄色闪烁，保存到 Flash
短按 A -> 退出
```

建议先把两个 DAPLink 分别贴标签：

```text
Host：蓝色，插电脑
Slave：绿色，接 STM32
```

无线连接结构：

```text
电脑 USB -> 蓝色 Host
          无线
绿色 Slave -> STM32
```

Slave 接 STM32：

```text
Slave GND   -> STM32 GND
Slave SWDIO -> STM32 PA13 / SWDIO
Slave SWCLK -> STM32 PA14 / SWCLK
Slave 3V3   -> STM32 3.3V / VTref
Slave NRST  -> STM32 NRST，可选
```

Slave 也必须供电。可以由 STM32 板子的 3.3V 供电，也可以单独供电，但必须和 STM32 共地。

无线链路初次测试建议用低速：

```powershell
openocd -f interface/cmsis-dap.cfg -c "adapter speed 100" -f target/stm32f1x.cfg
```

成功后可以继续试：

```powershell
openocd -f interface/cmsis-dap.cfg -c "adapter speed 500" -f target/stm32f1x.cfg
openocd -f interface/cmsis-dap.cfg -c "adapter speed 1000" -f target/stm32f1x.cfg
```

当前测试中 100、500、1000 kHz 均稳定，因此 VSCode 配置默认使用 `adapter speed 1000`。

## 5. 编译、下载并直接运行

这是日常最常用的“烧录运行”方式，不进入调试器。

在 VSCode 菜单中选择：

```text
Terminal -> Run Task... -> Flash and Run DogRobot
```

中文界面通常是：

```text
终端 -> 运行任务... -> Flash and Run DogRobot
```

该任务会执行：

```text
编译 DogRobot
下载 build/Debug/DogRobot.elf
Verify 校验
Reset 复位运行
退出 OpenOCD
```

成功输出示例：

```text
** Programming Started **
** Programming Finished **
** Verify Started **
** Verified OK **
** Resetting Target **
shutdown command invoked
```

出现下面输出也正常：

```text
ninja: no work to do.
```

它表示没有源码变化，不需要重新编译。

## 6. 编译并进入调试

在 VSCode 左侧运行和调试面板中选择：

```text
DAPLink OpenOCD STM32F103
```

然后按 `F5`。

该配置会先执行：

```text
Build DogRobot
```

然后启动 OpenOCD 和 GDB 调试会话。

成功时常见输出：

```text
Cortex-M3 r1p1 processor detected
Examination succeed
accepting 'gdb' connection
flash size = 128 KiB
halted due to debug-request
```

此时可以在 VSCode 中打断点、单步、继续运行、暂停和重启。

## 7. 两个入口的区别

```text
F5 / DAPLink OpenOCD STM32F103
= 编译 + 下载 + 进入调试

Terminal -> Run Task... -> Flash and Run DogRobot
= 编译 + 下载 + 校验 + 复位运行 + 退出
```

如果只是把程序烧进去运行，用 `Flash and Run DogRobot`。

如果需要断点、单步、看变量，用 `F5` 调试。

## 8. 常见问题

### openocd 找不到

先检查：

```powershell
openocd --version
```

如果找不到，确认 OpenOCD 的 `bin` 目录已经加入 Path。当前 VSCode 配置依赖 `openocd` 命令，因此新电脑必须让 `openocd --version` 能在 PowerShell 中正常输出版本。

### unable to find a matching CMSIS-DAP device

说明电脑没有识别到 DAPLink。检查：

```text
DAPLink 是否插电脑
USB 线是否是数据线
设备管理器是否出现 CMSIS-DAP / DAPLink / HID 设备
是否有其他软件占用 DAPLink
```

### CMSIS-DAP command CMD_INFO failed

OpenOCD 找到了设备，但和 DAPLink 通信失败。

可以尝试强制 HID 后端：

```powershell
openocd -f interface/cmsis-dap.cfg -c "cmsis_dap_backend hid" -f target/stm32f1x.cfg
```

当前测试中，一个 DAPLink 默认后端可用，另一个需要强制 HID 后端才能正常识别。

无线模式下，只有插电脑的 Host 会直接被 OpenOCD 识别，因此这个差异只影响 Host 端命令。

### Error connecting DP: cannot read IDR

DAPLink 本体已识别，但没有连上 STM32。检查：

```text
SWDIO/SWCLK 是否接反
GND 是否共地
目标板是否有 3.3V
Slave 是否供电
BOOT0 是否为 0
PA13/PA14 是否被电路强占或短路
```

### Programming 时报 couldn't open 路径

OpenOCD 使用 Tcl 解析命令，Windows 反斜杠路径可能被当作转义字符。

当前 `.vscode/tasks.json` 已使用工程工作目录和相对路径：

```text
program build/Debug/DogRobot.elf verify reset exit
```

不要在 OpenOCD 的 `program` 命令里直接写未处理的 `C:\...` 反斜杠路径。

### Warn: Adding extra erase range

这是 OpenOCD 为了按 Flash 页对齐擦除范围自动扩展擦除区间，通常正常。

### flash size = 128 KiB

很多 STM32F103C8T6 板子会被识别出 128 KiB Flash。当前工程实际测试中 OpenOCD 显示：

```text
flash size = 128 KiB
```

这是正常现象。
