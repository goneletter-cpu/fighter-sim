# Fighter Sim

C++ 战斗机飞行动力学仿真，3D线框渲染。

## 架构

```
src/
├── main.cpp              # 主循环：输入→PID→物理→渲染
├── pid.cpp               # 单轴PID控制器（含anti-windup）
├── physics.cpp           # 六自由度动力学，RK4积分
├── flight_controller.cpp # 三轴PID飞控（俯仰/滚转/偏航）
└── renderer.cpp          # OpenGL线框渲染 + 战斗机模型顶点

include/
├── pid.h
├── physics.h
├── flight_controller.h
└── renderer.h
```

## 编译（macOS，一条命令）

```bash
chmod +x build.sh
./build.sh
./build/fighter_sim
```

`build.sh` 会自动用 Homebrew 安装 glfw 和 glm，然后用 clang++ 直接编译。不需要 CMake。

### 自动重编译（推荐开发时使用）

保存文件就自动重新编译运行，反馈最快：

```bash
brew install entr
find src include -name '*.cpp' -o -name '*.h' | entr -r ./build.sh
```

## 操控

| 按键 | 功能 |
|------|------|
| W / S | 俯仰（抬头/低头） |
| A / D | 滚转（左/右） |
| Q / E | 偏航（左/右） |
| Shift / Ctrl | 油门加/减 |
| ESC | 退出 |

## 开发顺序建议

1. 先跑起来，看到线框飞机 ✓
2. 调整 `make_fighter_mesh()` 里的顶点，让外形更好看
3. 在 `physics.cpp` 里调整气动参数，改变飞行手感
4. 在 `flight_controller.cpp` 里调PID参数（kp/ki/kd）
5. 加地面/地平线渲染
6. 加HUD文字（考虑引入 stb_truetype）

## 调PID技巧

当前默认参数较为保守。如果飞机响应太慢，增大 `kp`；
如果飞机振荡（来回摆），增大 `kd` 或减小 `kp`；
如果稳态有误差，增大 `ki`（但要小心积分饱和）。
