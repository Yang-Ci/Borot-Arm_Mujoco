# reBot Arm B601-DM Web Simulator

Web simulator for the ROS2 reBot Arm B601-DM model. In this repository it is
part of a monorepo and reads the URDF and arm meshes from the sibling ROS2
workspace; this directory is not a self-contained standalone package.

The UI models the arm as 6 URDF joints plus the configured gripper actuator
from `gripper.yaml` (`motor_id: 0x07`). The current ROS2 URDF ends at
`end_link`, so the web simulator adds a lightweight visual gripper at the tool
end and drives it from 0-90 mm, matching the ROS2 demo values:

```text
close: 0.00 m
open:  0.09 m
```

## Run

```bash
npm start
```

Then open:

```text
http://localhost:3001
```

## HTTPS and PWA status

> **Note:** The current build does **not** register a Service Worker
> (`index.html` no longer loads `pwa.js`, and no
> `navigator.serviceWorker.register` call exists). The files
> `manifest.webmanifest`, `service-worker.js`, and `pwa.js` are still present in
> the repository but are orphaned and unused. As a result the panel is **not
> installable as a full PWA** and has no offline caching. HTTPS mode itself
> still works and is mainly useful to satisfy the secure-context requirement for
> `wss://` rosbridge connections.

Browsers block `ws://` WebSocket connections from HTTPS pages (mixed content).
If you serve the panel over HTTPS, keep the ROS connection scheme aligned:

```text
HTTPS page -> use wss:// for rosbridge, or proxy rosbridge through the same HTTPS origin
HTTP page  -> ws:// works for LAN testing
```

### Local HTTPS on Windows

Generate a local development certificate:

```powershell
npm run cert:dev
```

The certificate is bound to the LAN IP printed by the script. If your phone uses
a different computer IP, regenerate it explicitly:

```powershell
pwsh -ExecutionPolicy Bypass -File scripts/create-dev-cert.ps1 -HostIp 192.168.x.x
```

Start the HTTPS server:

```powershell
npm run start:https
```

Then open the LAN URL printed by the server, for example:

```text
https://192.168.x.x:3443
```

If Android says the connection is not secure, the phone does not trust the local
certificate yet. Copy `.certs/rebotarm-local-root-ca.cer` to the phone and
install it as a trusted CA certificate in Android settings, then reopen the HTTPS
URL.

Treat that root certificate like a development key: install only the one you
generated yourself, use it only on your own LAN, and remove it from the phone
when you no longer need it.

## ROS2 bridge to Ubuntu VM

Default WebSocket target:

```text
ws://<Ubuntu 主机实际 IP>:9090
```

>`start_rebot_mujoco_all.sh` 启动时打印的 `web URL` 即为当前主机实际 IP。

On Ubuntu 24.04 + ROS2 Jazzy, start the ROS side:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/reBotArmController_ROS2-main
colcon build --symlink-install
source install/setup.bash
ros2 launch rebotarm_bringup fake_bringup.launch.py
```

In another Ubuntu terminal, start rosbridge:

```bash
source /opt/ros/jazzy/setup.bash
source ~/reBotArmController_ROS2-main/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 address:=0.0.0.0
```

From Windows, open the simulator and click `连接 ROS`. Keep `允许网页向真实机械臂发控制` off until the fake driver mirrors correctly.

The simulator reads the ROS2 model from:

```text
../reBotArmController_ROS2-main/src/rebotarm_bringup/description/urdf/reBot-DevArm_fixend.urdf
../reBotArmController_ROS2-main/src/rebotarm_bringup/description/meshes
```

The web-only gripper meshes are kept in:

```text
./split_meshes/grouped_gripper
```

Do not add a second `urdf/` or `meshes/` copy here for normal monorepo use. A
standalone export must copy both directories from the same ROS2 model version
and update `server.js` to read those local paths. The historical
`split_meshes/end_link/` directory contains mesh-processing intermediates; the
runtime uses only the four grouped gripper STL files.

```powershell
Set-Location e:\reBot-DevArm-main\reBotArm_simulator-DM
node server.js
```
