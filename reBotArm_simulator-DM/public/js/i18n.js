/**
 * reBot Arm — 国际化引擎（中 / 英）
 * 暴露 window.rebotI18n = { t, setLang, getLang, applyI18n, onLangChange }
 * 静态文案：data-i18n 文本；data-i18n-ph 占位符；data-i18n-title 标题；data-i18n-html 富文本
 * 动态文案：rebotI18n.t('key', { param: value })，占位符用 {name}
 */
(function () {
  const STORAGE_KEY = 'rebotarm.lang';
  const SUPPORTED = ['zh', 'en'];
  const DICT = {
    'app.eyebrow': { zh: '欢迎来到reBot Arm B601-DM的世界', en: 'Welcome to reBot Arm B601-DM' },
    'app.title': { zh: 'ROS2 ReBot Arm B601-DM机械臂仿真器', en: 'ROS2 ReBot Arm B601-DM Arm Simulator' },
    'app.loading': { zh: '加载中', en: 'Loading' },
    'app.tcp': { zh: 'TCP', en: 'TCP' },
    'app.tcpDefault': { zh: 'X -- / Y -- / Z --', en: 'X -- / Y -- / Z --' },
    'app.reach': { zh: '可达参考', en: 'Reach' },
    'app.reachDefault': { zh: '--', en: '--' },
    'app.dragTarget': { zh: '拖拽目标', en: 'Drag target' },
    'app.dragDisabled': { zh: '未启用', en: 'Disabled' },
    'app.dragTcp': { zh: '拖拽 TCP', en: 'Drag TCP' },
    'app.axisX': { zh: 'ROS +X 前方', en: 'ROS +X Forward' },
    'app.axisY': { zh: 'ROS +Y 左侧', en: 'ROS +Y Left' },
    'app.axisZ': { zh: 'ROS +Z 向上', en: 'ROS +Z Up' },
    'panel.eyebrow': { zh: '控制台', en: 'Console' },
    'panel.title': { zh: '仿真控制台', en: 'Sim Console' },
    'panel.help': { zh: '功能帮助', en: 'Help' },
    'panel.collapse': { zh: '折叠侧边栏', en: 'Collapse sidebar' },
    'panel.reset': { zh: '重置视角', en: 'Reset view' },
    'panel.langTitle': { zh: '语言 / Language', en: '语言 / Language' },
    'ros.eyebrow': { zh: 'ROS2 桥接', en: 'ROS2 Bridge' },
    'ros.title': { zh: '真实机械臂连接', en: 'Real Arm Connection' },
    'ros.ws': { zh: 'WebSocket', en: 'WebSocket' },
    'ros.connect': { zh: '连接 ROS', en: 'Connect ROS' },
    'ros.disconnect': { zh: '断开', en: 'Disconnect' },
    'ros.safeDisconnect': { zh: '断开时自动安全回零并失能', en: 'Auto safe-home & disable on disconnect' },
    'ros.mirror': { zh: '镜像真实关节状态到网页', en: 'Mirror real joint state to web' },
    'ros.controlEnable': { zh: '允许网页向真实机械臂发控制', en: 'Allow web to control the real arm' },
    'ros.enable': { zh: '使能', en: 'Enable' },
    'ros.disable': { zh: '失能', en: 'Disable' },
    'ros.safeHome': { zh: '安全回零', en: 'Safe Home' },
    'ros.openGripper': { zh: '夹爪打开', en: 'Open Gripper' },
    'ros.closeGripper': { zh: '夹爪闭合', en: 'Close Gripper' },
    'ros.gravity': { zh: '重力补偿', en: 'Gravity Comp.' },
    'ros.gravityNotQueried': { zh: '未查询', en: 'Not queried' },
    'ros.gravityStart': { zh: '启动重补', en: 'Start Comp.' },
    'ros.gravityStop': { zh: '停止重补', en: 'Stop Comp.' },
    'ros.gravityQuery': { zh: '查询状态', en: 'Query Status' },
    'ros.feedbackError': { zh: '反馈差值', en: 'Feedback Error' },
    'ros.feedbackWaiting': { zh: '等待 MuJoCo 物理关节反馈', en: 'Waiting for MuJoCo physics joint feedback' },
    'ros.message': { zh: '先在 Ubuntu 虚拟机启动 rosbridge，再输入地址连接。', en: 'Start rosbridge in the Ubuntu VM first, then enter the address to connect.' },
    'log.title': { zh: 'ROS 实时日志', en: 'ROS Live Log' },
    'log.clear': { zh: '清空日志', en: 'Clear Log' },
    'llm.eyebrow': { zh: 'AI 助手', en: 'AI Assistant' },
    'llm.title': { zh: 'LLM 文本控制', en: 'LLM Text Control' },
    'llm.notStarted': { zh: '未启动', en: 'Not started' },
    'llm.placeholder': { zh: '输入指令，如：打开夹爪、移动到...', en: 'Enter a command, e.g. open gripper, move to...' },
    'llm.send': { zh: '发送', en: 'Send' },
    'llm.start': { zh: '启动 AI 助手', en: 'Start AI Assistant' },
    'llm.stop': { zh: '停止', en: 'Stop' },
    'llm.message': { zh: '点击启动按钮，系统将自动连接 MCP 并启用 AI 控制。', en: 'Click start to auto-connect MCP and enable AI control.' },
    'camera.eyebrow': { zh: 'RGB Camera', en: 'RGB Camera' },
    'camera.title': { zh: '桌面俯视相机', en: 'Overhead Camera' },
    'camera.topic': { zh: '话题', en: 'Topic' },
    'camera.vision': { zh: '颜色识别', en: 'Color Detection' },
    'camera.visionWait': { zh: '等待检测', en: 'Awaiting detection' },
    'camera.targetPose': { zh: '目标 Pose', en: 'Target Pose' },
    'camera.targetColor': { zh: '目标颜色', en: 'Target Color' },
    'camera.colorAuto': { zh: '自动目标', en: 'Auto target' },
    'camera.colorRed': { zh: '红色', en: 'Red' },
    'camera.colorYellow': { zh: '黄色', en: 'Yellow' },
    'camera.colorBlue': { zh: '蓝色', en: 'Blue' },
    'camera.approachZ': { zh: '上方 Z m', en: 'Approach Z m' },
    'camera.graspZ': { zh: '下探 Z m', en: 'Descent Z m' },
    'camera.fillPose': { zh: '填入 Pose', en: 'Fill Pose' },
    'camera.moveAbove': { zh: '移动到目标上方', en: 'Move Above Target' },
    'camera.pick': { zh: '视觉抓取', en: 'Vision Pick' },
    'camera.place': { zh: '放下物体', en: 'Place Object' },
    'safety.title': { zh: '安全限制', en: 'Safety Limits' },
    'safety.vlim': { zh: '关节速度上限 rad/s', en: 'Joint speed limit rad/s' },
    'safety.trajDuration': { zh: '轨迹总时长 s', en: 'Trajectory duration s' },
    'action.title': { zh: 'Pose 运动', en: 'Pose Motion' },
    'action.x': { zh: 'X m', en: 'X m' },
    'action.y': { zh: 'Y m', en: 'Y m' },
    'action.z': { zh: 'Z m', en: 'Z m' },
    'action.duration': { zh: '时长 s', en: 'Duration s' },
    'action.ik': { zh: 'IK 运动', en: 'IK Move' },
    'help.title': { zh: 'reBot Arm B601-DM 网页控制台 — 功能说明', en: 'reBot Arm B601-DM Web Console — Feature Guide' },
    'help.s1.title': { zh: '一、连接与模式', en: '1. Connection & Mode' },
    'help.s1.connect': { zh: '<b>连接 ROS</b>：输入 rosbridge 的 WebSocket 地址（如 <code>ws://localhost:9090</code>），点击连接。连接后自动订阅关节状态、夹爪状态和机械臂状态。', en: '<b>Connect ROS</b>: Enter the rosbridge WebSocket address (e.g. <code>ws://localhost:9090</code>) and click Connect. It then auto-subscribes to joint state, gripper state and arm status.' },
    'help.s1.disconnect': { zh: '<b>断开</b>：断开与 rosbridge 的连接，停止所有订阅和发布。', en: '<b>Disconnect</b>: Drops the rosbridge link and stops all subscriptions and publishers.' },
    'help.s1.control': { zh: '<b>控制锁</b>：勾选后允许网页向 ROS 发送控制命令。未勾选时拖滑块只动 3D 模型，不发 ROS 命令。连接到真机控制器时命令会驱动真实硬件，请谨慎操作。', en: '<b>Control lock</b>: When checked the web may send ROS commands. When unchecked sliders only move the 3D model. With the real controller, commands drive real hardware — use with care.' },
    'help.s2.title': { zh: '二、机械臂控制', en: '2. Arm Control' },
    'help.s2.enable': { zh: '<b>使能</b>：给所有电机通电，机械臂进入可接受命令状态。每次重启控制器后需重新使能。预期：日志显示「enabled」，状态栏显示「已使能」。', en: '<b>Enable</b>: Powers all motors so the arm accepts commands. Re-enable after every controller restart. Expected: log shows "enabled", status shows "Enabled".' },
    'help.s2.disable': { zh: '<b>失能</b>：所有电机断电，手臂自由下垂。紧急情况或长时间不用时使用。失能后不接受运动命令。', en: '<b>Disable</b>: Cuts power to all motors; the arm hangs free. Use for emergencies or long idle. Motion commands are rejected while disabled.' },
    'help.s2.home': { zh: '<b>安全回零</b>：平滑插值回到所有关节零位（0°），速度受限。建议在异常或任务结束后使用。预期：6 个关节缓慢回到 0°，3D 模型同步跟随。', en: '<b>Safe Home</b>: Smoothly interpolates back to all-joint zero (0°) at limited speed. Use after an anomaly or task. Expected: 6 joints slowly return to 0°, 3D model follows.' },
    'help.s3.title': { zh: '三、夹爪控制', en: '3. Gripper Control' },
    'help.s3.open': { zh: '<b>夹爪打开</b>：夹爪运动到最大开口（90mm）。预期：夹爪张开，3D 模型夹爪同步。', en: '<b>Open Gripper</b>: Opens to the maximum width (90mm). Expected: gripper opens, 3D model stays in sync.' },
    'help.s3.close': { zh: '<b>夹爪闭合</b>：夹爪运动到完全闭合（0mm），施加默认 0.3Nm 夹持力。预期：夹爪闭合并保持夹持力。', en: '<b>Close Gripper</b>: Closes fully (0mm) with default 0.3Nm grip force. Expected: gripper closes and holds force.' },
    'help.s3.slider': { zh: '<b>夹爪滑块</b>：拖动可设置任意开口（0~90mm），实时控制夹爪宽度。', en: '<b>Gripper slider</b>: Drag to set any opening (0–90mm) and control gripper width in real time.' },
    'help.s4.title': { zh: '四、重力补偿', en: '4. Gravity Compensation' },
    'help.s4.start': { zh: '<b>启动重补</b>：切换到 MIT 模式，使用 Pinocchio 计算重力力矩并叠加低增益位置保持。启动后手臂可悬停不掉，松手后保持当前姿态。预期：日志显示「gravity compensation started」，状态栏显示「运行中」。', en: '<b>Start Comp.</b>: Switches to MIT mode, computes gravity torque with Pinocchio and adds low-gain position hold. The arm floats and holds its pose on release. Expected: log shows "gravity compensation started", status shows "Running".' },
    'help.s4.stop': { zh: '<b>停止重补</b>：切回 POS_VEL 位置模式，启动 hold 循环保持当前姿态。预期：日志显示「gravity compensation stopped」，状态栏显示「未运行」。', en: '<b>Stop Comp.</b>: Returns to POS_VEL mode and starts a hold loop to keep the current pose. Expected: log shows "gravity compensation stopped", status shows "Not running".' },
    'help.s4.query': { zh: '<b>查询状态</b>：查询重力补偿是否运行中，以及锁定目标角度（启动时记录的关节角度）。用于在不改变状态的情况下确认当前运行状态。', en: '<b>Query Status</b>: Reports whether gravity comp. is running and the locked target angles (recorded at start). Use to check the running state without changing it.' },
    'help.s5.title': { zh: '五、Pose 运动', en: '5. Pose Motion' },
    'help.s5.xyz': { zh: '<b>X / Y / Z</b>：输入末端目标位置（米），默认 X=0.30, Y=0.00, Z=0.30。', en: '<b>X / Y / Z</b>: Enter the end-effector target position (meters), default X=0.30, Y=0.00, Z=0.30.' },
    'help.s5.duration': { zh: '<b>时长</b>：IK 平滑轨迹的执行时间（0.4~8 秒），默认 2.0 秒；时间越长，运动越慢。', en: '<b>Duration</b>: IK smooth-trajectory execution time (0.4–8 s), default 2.0 s; longer time means slower motion.' },
    'help.s5.ik': { zh: '<b>IK 运动</b>：先解算逆运动学，再按照上面的时长执行平滑轨迹。需勾选控制锁 + 使能。预期：机械臂在指定时间内运动到目标位置，日志显示执行结果。', en: '<b>IK Move</b>: Solves inverse kinematics, then runs a smooth trajectory over the duration above. Requires control lock + enable. Expected: arm reaches the target in the set time, log shows the result.' },
    'help.s6.title': { zh: '六、关节滑块', en: '6. Joint Sliders' },
    'help.s6.body': { zh: '拖动 joint1~joint6 滑块控制单个关节角度（弧度）。勾选控制锁后控制真机，未勾选只动 3D 模型。滑块有节流（约 50ms），避免总线过载。建议从末端关节（joint5/joint6）开始小幅度测试。', en: 'Drag joint1–joint6 sliders to set a single joint angle (radians). With the control lock checked it controls the real arm; otherwise only the 3D model moves. Sliders are throttled (~50ms) to avoid bus overload. Start with small moves on the distal joints (joint5/joint6).' },
    'help.s7.title': { zh: '七、诊断与日志', en: '7. Diagnostics & Log' },
    'help.s7.diag': { zh: '<b>检测 ROS</b>：扫描当前 ROS 环境的 topic 和 service，检查关键接口是否可用。', en: '<b>Detect ROS</b>: Scans the current ROS topics and services and checks whether key interfaces are available.' },
    'help.s7.log': { zh: '<b>ROS 实时日志</b>：显示所有 ROS 操作的请求和响应，包括成功/失败/警告。排查问题时先看这里。', en: '<b>ROS Live Log</b>: Shows every ROS request and response — success / failure / warning. Look here first when troubleshooting.' },
    'help.s7.preset': { zh: '<b>姿态预设</b>：预设的关节角度组合，一键到达常用姿态。', en: '<b>Pose presets</b>: Stored joint-angle combinations; one tap reaches a common pose.' },
    'help.warn.title': { zh: '⚠ 安全须知', en: '⚠ Safety Notes' },
    'help.warn.1': { zh: '1. 首次使用真机时，先用 Fake Driver 验证关节方向和限位。', en: '1. Before using the real arm, verify joint directions and limits with the Fake Driver first.' },
    'help.warn.2': { zh: '2. 连接真机控制器时命令会驱动真实硬件，请谨慎操作。', en: '2. With the real controller, commands drive real hardware — operate with care.' },
    'help.warn.3': { zh: '3. 关节滑块建议从末端关节小幅度开始，确认方向正确后再大范围运动。', en: '3. Start joint sliders with small moves on distal joints; confirm direction before larger motions.' },
    'help.warn.4': { zh: '4. 任何异常立即点「失能」或取消控制锁。', en: '4. On any anomaly, immediately tap "Disable" or uncheck the control lock.' },
    'help.warn.5': { zh: '5. 重力补偿启动后手臂会主动保持姿态，先测重补再测滑块更安全。', en: '5. With gravity comp. on the arm actively holds its pose; test comp. before sliders for safety.' },
    'help.close': { zh: '关闭', en: 'Close' },
    'preset.title': { zh: '姿态预设', en: 'Pose Presets' },
    'joint.title': { zh: '关节角度', en: 'Joint Angles' },
    'layer.title': { zh: '空间图层', en: 'Spatial Layers' },
    'layer.envelope': { zh: '显示估算可达包络', en: 'Show estimated reach envelope' },
    'layer.ghost': { zh: '显示目标残影', en: 'Show target ghost' },
    'adv.title': { zh: '高级控制', en: 'Advanced' },
    'adv.play': { zh: '循环播放取放路径', en: 'Loop pick-place path' },
    'adv.plan': { zh: '规划到当前姿态', en: 'Plan to current pose' },
    'adv.drag': { zh: '启用 TCP 拖拽', en: 'Enable TCP drag' },
    'adv.stop': { zh: '停止播放/退出拖拽', en: 'Stop / exit drag' },
    'teach.title': { zh: 'TCP 示教', en: 'TCP Teach' },
    'teach.record': { zh: '开始录制', en: 'Record' },
    'teach.replay': { zh: '回放', en: 'Replay' },
    'teach.export': { zh: '导出 ROS waypoint', en: 'Export ROS waypoints' },
    'teach.clear': { zh: '清空', en: 'Clear' },
    'teach.status': { zh: '未录制', en: 'Not recording' },
    'teach.placeholder': { zh: '导出的 waypoints 会显示在这里', en: 'Exported waypoints appear here' },
    'metrics.title': { zh: '机械臂信息', en: 'Arm Info' },
    'metrics.model': { zh: '模型', en: 'Model' },
    'metrics.modelVal': { zh: 'Rebot_ARM-B601-DM', en: 'Rebot_ARM-B601-DM' },
    'metrics.dof': { zh: '自由度', en: 'DOF' },
    'metrics.dofVal': { zh: '6 DOF + 夹爪', en: '6 DOF + gripper' },
    'metrics.payload': { zh: '建议负载', en: 'Payload' },
    'metrics.payloadVal': { zh: '1.5 kg', en: '1.5 kg' },
    'metrics.frame': { zh: '坐标', en: 'Frame' },
    'metrics.frameVal': { zh: 'X 前 / Y 左 / Z 上', en: 'X fwd / Y left / Z up' },
    'load.text': { zh: '正在加载 Rebot_ARM-B601-DM 机械臂模型...', en: 'Loading Rebot_ARM-B601-DM arm model...' },
    'st.online': { zh: '在线', en: 'Online' },
    'st.connecting': { zh: '连接中', en: 'Connecting' },
    'st.error': { zh: '错误', en: 'Error' },
    'st.offline': { zh: '离线', en: 'Offline' },
    'st.notRunning': { zh: '未运行', en: 'Not running' },
    'st.running': { zh: '运行中', en: 'Running' },
    'st.cameraOffline': { zh: '离线', en: 'Offline' },
    'st.cameraWaitFrame': { zh: '等待画面', en: 'Awaiting frame' },
    'st.cameraWaitTopic': { zh: '等待话题', en: 'Awaiting topic' },
    'st.cameraError': { zh: '错误', en: 'Error' },
    'st.cameraDataError': { zh: '数据异常', en: 'Bad data' },
    'st.diagFound': { zh: '已发现', en: 'Found' },
    'st.diagFoundWait': { zh: '已发现 / 等待', en: 'Found / waiting' },
    'st.diagWait': { zh: '等待', en: 'Waiting' },
    'st.visionNone': { zh: '未发现', en: 'None found' },
    'msg.rosNotConnected': { zh: 'ROS 未连接', en: 'ROS not connected' },
    'msg.controlLockClosed': { zh: '控制锁未打开，网页只更新仿真，不会控制 ROS。', en: 'Control lock is off — the web only updates the sim, no ROS commands are sent.' },
    'msg.gripperSimOnly': { zh: '夹爪已更新到网页仿真；ROS 未连接。', en: 'Gripper updated in web sim; ROS not connected.' },
    'msg.gripperCmdPublished': { zh: '夹爪指令已发布：{mm} 毫米{fb}', en: 'Gripper command sent: {mm} mm{fb}' },
    'msg.gripperArrived': { zh: '夹爪已到位：ROS反馈 {mm} 毫米', en: 'Gripper reached target: ROS feedback {mm} mm' },
    'msg.gripperMoving': { zh: '夹爪运动中：指令 {cmd} 毫米 / ROS反馈 {mm} 毫米', en: 'Gripper moving: cmd {cmd} mm / ROS feedback {mm} mm' },
    'msg.disconnectHome': { zh: '断开防护：正在安全回零…', en: 'Disconnect guard: safe-homing…' },
    'msg.disconnectDisable': { zh: '断开防护：正在失能…', en: 'Disconnect guard: disabling…' },
    'msg.visionFillDone': { zh: '已把视觉目标填入 Pose 输入框', en: 'Filled the vision target into the Pose inputs' },
    'msg.noVisionTarget': { zh: '没有可用的视觉目标。', en: 'No vision target available.' },
    'msg.visionCoordError': { zh: '视觉目标坐标异常。', en: 'Vision target coordinates invalid.' },
    'msg.visionNoExactAbove': { zh: '{color} 目标上方没有精确 IK 解（最佳误差 {error}mm），已停止移动。', en: 'No exact IK solution above {color} (best error {error}mm); movement stopped.' },
    'msg.ikSolving': { zh: '{label}：IK 解算中', en: '{label}: solving IK' },
    'msg.ikNoSolution': { zh: 'IK 没有返回可用关节解。', en: 'IK returned no usable joint solution.' },
    'msg.ikApprox': { zh: '{message}，继续执行近似解。', en: '{message}; proceeding with approximate solution.' },
    'msg.noHeldObject': { zh: '当前没有已夹持的视觉物体，请先执行视觉抓取。', en: 'No held vision object; run a vision pick first.' },
    'msg.graspDemoDone': { zh: '视觉抓取演示完成，按短边夹紧到 {mm} 毫米', en: 'Vision pick done; squeezed to {mm} mm on the short edge' },
    'msg.placeDone': { zh: '视觉放置完成：{color} 已释放，机械臂已抬升', en: 'Vision place done: {color} released, arm lifted' },
    'msg.gripperNotReached': { zh: '{label}未确认到位，已停止本轮抓取，避免夹爪未全开就关闭', en: '{label} not confirmed reached; aborted this pick to avoid closing before fully open' },
    'msg.llmStartPrompt': { zh: '请先点击"启动 AI 助手"', en: 'Please click "Start AI Assistant" first' },
    'msg.llmConnecting': { zh: '正在连接 text-agent...', en: 'Connecting to text-agent...' },
    'msg.llmConnected': { zh: 'text-agent 已连接，可以开始对话。', en: 'text-agent connected; ready to chat.' },
    'msg.llmConnectFail': { zh: '连接失败: {err}', en: 'Connection failed: {err}' },
    'msg.llmStopped': { zh: '已停止。', en: 'Stopped.' },
    'msg.llmLoadCfgFail': { zh: '加载配置失败', en: 'Failed to load config' },
    'log.rosOfflineFirst': { zh: 'rosbridge 离线，请先连接', en: 'rosbridge offline; connect first' },
    'log.controlLockOpen': { zh: '控制锁已打开', en: 'Control lock opened' },
    'log.mirrorPaused': { zh: '已暂停 ROS 镜像，避免旧反馈把滑块拉回', en: 'ROS mirror paused to stop stale feedback pulling sliders back' },
    'log.lowLevelFallbackInfo': { zh: '未发现轨迹动作接口，轨迹按钮将使用低层回放', en: 'No trajectory action found; trajectory buttons use low-level playback' },
    'log.rosapiFallback': { zh: 'rosapi 不可用，改用实时话题时间判断（{err}）', en: 'rosapi unavailable; using live topic-time checks ({err})' },
    'log.lowLevelFallbackWarn': { zh: '未发现 FollowJointTrajectory 动作，改用低层回放', en: 'No FollowJointTrajectory action found; using low-level playback' },
    'log.lowLevelStart': { zh: '低层回放开始（{n} 个点）', en: 'Low-level playback started ({n} points)' },
    'log.lowLevelDone': { zh: '低层回放完成', en: 'Low-level playback complete' },
    'log.lowLevelCancelled': { zh: '低层回放已取消', en: 'Low-level playback cancelled' },
    'log.mujocoSyncActive': { zh: '已收到 MuJoCo 同步数据（{reason}），仅用于网页/数字孪生显示，控制通道不变', en: 'MuJoCo sync data received ({reason}); used only for web/digital-twin display, control routing is unchanged' },
    'log.stopPlayback': { zh: '已请求停止低层回放', en: 'Requested stop of low-level playback' },
    'log.visionFill': { zh: '视觉目标 -> Pose 输入框', en: 'Vision target -> Pose inputs' },
    'log.autoTarget': { zh: '自动目标随机选择：{color}', en: 'Auto target randomly selected: {color}' },
    'log.visionSkipMove': { zh: '{label}：已在目标附近，跳过重复移动', en: '{label}: already near target, skipping repeat move' },
    'log.visionRelocate': { zh: '视觉二次定位：{color} 位置已更新', en: 'Vision re-localize: {color} position updated' },
    'log.graspDone': { zh: '视觉抓取演示完成，夹爪指令 {mm} 毫米', en: 'Vision pick done; gripper command {mm} mm' },
    'log.placeIgnored': { zh: '放下物体已忽略：当前没有已夹持目标', en: 'Place ignored: no held target' },
    'log.placeDoneLog': { zh: '视觉放置完成：{color} 已释放并抬升', en: 'Vision place done: {color} released and lifted' },
    'log.mcpAnimParseFail': { zh: 'MCP 动画事件解析失败', en: 'MCP animation event parse failed' },
    'log.simAttach': { zh: '网页动画：{color} 已绑定到夹爪跟随', en: 'Web anim: {color} bound to gripper follow' },
    'log.simRelease': { zh: '网页动画：已释放上一件跟随物体', en: 'Web anim: released previous followed object' },
    'log.gripperCmd': { zh: '夹爪指令 {mm} 毫米 -> {topic}', en: 'Gripper command {mm} mm -> {topic}' },
    'log.gripperDone': { zh: '{label}完成{fb}', en: '{label} done{fb}' },
    'log.ikResult': { zh: 'IK {result}：[{q}]', en: 'IK {result}: [{q}]' },
    'log.ikSuccess': { zh: '成功', en: 'success' },
    'log.ikFail': { zh: '失败', en: 'failed' },
    'log.rosCallSuccess': { zh: 'ROS 调用成功', en: 'ROS call succeeded' },
    'log.rosCallFail': { zh: 'ROS 调用失败', en: 'ROS call failed' },
    'log.rosCallDone': { zh: 'ROS 调用完成', en: 'ROS call complete' },
    'log.disconnectHomeStart': { zh: '断开防护：开始安全回零', en: 'Disconnect guard: safe-home started' },
    'log.disconnectHomeDone': { zh: '断开防护：安全回零完成', en: 'Disconnect guard: safe-home complete' },
    'log.disconnectDisableStart': { zh: '断开防护：开始失能', en: 'Disconnect guard: disable started' },
    'log.disconnectDisableDone': { zh: '断开防护：失能完成，正在断开 ROS', en: 'Disconnect guard: disabled, disconnecting ROS' },
    'btn.pickBusy': { zh: '抓取中...', en: 'Picking...' },
    'btn.placeBusy': { zh: '放置中...', en: 'Placing...' },
    'fb.gripperFb': { zh: '，当前 ROS反馈 {mm} 毫米', en: ', current ROS feedback {mm} mm' },
    'fb.gripperSrcFb': { zh: '，{src} 反馈 {mm} 毫米', en: ', {src} feedback {mm} mm' },
    'fb.visionCount': { zh: '{count} 个 / 目标 {color}', en: '{count} / target {color}' },
    'fb.armStatus': { zh: '{enabled}，模式 {mode}，状态 {machine}{errors}', en: '{enabled}, mode {mode}, state {machine}{errors}' },
    'fb.errorMax': { zh: '最大 {max} 度 {joint} / RMS {rms} 度', en: 'Max {max}° {joint} / RMS {rms}°' },
    'fb.gravityDetail': { zh: ' / {detail}', en: ' / {detail}' },
    'vision.colorRed': { zh: '红色', en: 'red' },
    'vision.colorYellow': { zh: '黄色', en: 'yellow' },
    'vision.colorBlue': { zh: '蓝色', en: 'blue' },
    'llm.started': { zh: '已启动', en: 'Started' },
    'llm.starting': { zh: '启动中...', en: 'Starting...' },
    'llm.startFail': { zh: '启动失败', en: 'Start failed' },
    'llm.backendInfo': { zh: '代理后端: {textAgent}  ·  MCP: {mcp}', en: 'Agent backend: {textAgent}  ·  MCP: {mcp}' },
    'llm.connectingVm': { zh: '正在连接虚拟机中的 text-agent 服务...', en: 'Connecting to the text-agent service in the VM...' },
    'llm.welcome': { zh: '已连接到 text-agent。\n直接输入中文指令即可，例如：\n• 查询机械臂状态\n• 移动到 X=0.3 Y=0 Z=0.3\n• 打开夹爪\n• 抓取红色物块', en: 'Connected to text-agent.\nType a natural-language command, e.g.:\n• Query arm status\n• Move to X=0.3 Y=0 Z=0.3\n• Open gripper\n• Pick up the red block' },
    'llm.connectFailDetail': { zh: '连接失败: {err}\n请确认虚拟机中已启动:\n  python3 -m rebotarm_agent.rebotarm_text_agent --http-server --yes', en: 'Connection failed: {err}\nMake sure the VM is running:\n  python3 -m rebotarm_agent.rebotarm_text_agent --http-server --yes' },
    'llm.stoppedChat': { zh: '已停止对话。', en: 'Chat stopped.' },
    'llm.thinking': { zh: '思考中...', en: 'Thinking...' },
    'llm.errorMsg': { zh: '错误: {err}', en: 'Error: {err}' },
    'llm.unknownError': { zh: '未知错误', en: 'Unknown error' },
    'llm.noReply': { zh: '(无回复)', en: '(No reply)' },
    'llm.toolEvent': { zh: '🔧 {name}\n  参数: {args}\n  结果: {result}', en: '🔧 {name}\n  args: {args}\n  result: {result}' },
    'llm.requestFail': { zh: '请求失败: {err}', en: 'Request failed: {err}' },
    'st.enabled': { zh: '已使能', en: 'Enabled' },
    'st.disabled': { zh: '已失能', en: 'Disabled' },
    'st.waitNode': { zh: '等待节点', en: 'Awaiting node' },
    'st.serviceUnavailable': { zh: '服务不可用', en: 'Service unavailable' },
    'st.encodingUnsupported': { zh: '不支持', en: 'Unsupported' },
    'msg.reqEnable': { zh: '已请求使能', en: 'Enable requested' },
    'msg.reqDisable': { zh: '已请求失能', en: 'Disable requested' },
    'msg.reqSafeHome': { zh: '已请求安全回零', en: 'Safe-home requested' },
    'msg.reqGravityStart': { zh: '已请求启动重力补偿', en: 'Gravity comp. start requested' },
    'msg.reqGravityStop': { zh: '已请求停止重力补偿', en: 'Gravity comp. stop requested' },
    'msg.reqGravityQuery': { zh: '已请求查询重力补偿', en: 'Gravity comp. status requested' },
    'msg.reqIkMove': { zh: '已请求 IK 平滑运动（{sec} 秒）', en: 'IK smooth move requested ({sec} s)' },
    'msg.httpsWsBlocked': { zh: '当前页面是 HTTPS，浏览器会拦截 ws:// 连接。请用 http://localhost:3001 打开页面后继续填写这个 IP 地址。', en: 'This page is HTTPS; the browser blocks ws://. Open it via http://localhost:3001, then fill in this IP address.' },
    'msg.serviceNotFound': { zh: 'ROS 已连接，但未发现服务 {name}', en: 'ROS connected but service {name} not found' },
    'msg.safeHomeFail': { zh: '安全回零失败', en: 'Safe-home failed' },
    'msg.disableFail': { zh: '失能失败', en: 'Disable failed' },
    'msg.disconnectGuardFail': { zh: '断开防护失败，ROS 保持连接：{err}', en: 'Disconnect guard failed; ROS stays connected: {err}' },
    'msg.goalAccepted': { zh: '动作目标已接受', en: 'Goal accepted' },
    'msg.goalRejected': { zh: '动作目标被拒绝', en: 'Goal rejected' },
    'msg.gripperReached': { zh: '夹爪到达 {mm} 毫米', en: 'Gripper reached {mm} mm' },
    'msg.visionMoveAbort': { zh: '视觉移动流程中止', en: 'Vision move flow aborted' },
    'msg.visionPickAbort': { zh: '视觉抓取流程中止', en: 'Vision pick flow aborted' },
    'msg.visionPlaceAbort': { zh: '视觉放置流程中止', en: 'Vision place flow aborted' },
    'msg.stepFailed': { zh: '{label}失败，已停止后续动作', en: '{label} failed; subsequent moves stopped' },
    'msg.visionMoveFail': { zh: '视觉目标动作下发失败', en: 'Vision target move dispatch failed' },
    'msg.ikApproxFallback': { zh: 'IK 未完全收敛，但返回了可用近似解。', en: 'IK did not fully converge but returned a usable approximate solution.' },
    'msg.moveAboveTarget': { zh: '移动到 {color} 目标上方', en: 'Move above {color} target' },
    'msg.pickOpenGripper': { zh: '视觉抓取：夹爪完全打开', en: 'Vision pick: open gripper fully' },
    'msg.pickMoveAbove': { zh: '视觉抓取：移动到 {color} 上方', en: 'Vision pick: move above {color}' },
    'msg.pickAlign': { zh: '视觉抓取：垂直对正 {color}', en: 'Vision pick: vertical align {color}' },
    'msg.pickPreDescend': { zh: '视觉抓取：垂直预下探 {color}', en: 'Vision pick: pre-descend {color}' },
    'msg.pickDescend': { zh: '视觉抓取：下探 {color}', en: 'Vision pick: descend to {color}' },
    'msg.pickSqueeze': { zh: '视觉抓取：夹紧 {color}', en: 'Vision pick: squeeze {color}' },
    'msg.pickBlueLift': { zh: '视觉抓取：蓝色强制离桌抬起 z={z}', en: 'Vision pick: blue forced lift-off z={z}' },
    'msg.pickBlueTransit': { zh: '视觉抓取：蓝色强制高位中转 z={z}', en: 'Vision pick: blue forced high transit z={z}' },
    'msg.pickLift': { zh: '视觉抓取：离桌抬起 {color}', en: 'Vision pick: lift-off {color}' },
    'msg.pickRaise': { zh: '视觉抓取：抬高 {color}', en: 'Vision pick: raise {color}' },
    'msg.pickTransit': { zh: '视觉抓取：安全中转 {color}', en: 'Vision pick: safe transit {color}' },
    'msg.placeMoveAbove': { zh: '视觉放置：移动到 {color} 放置点上方', en: 'Vision place: move above {color} drop point' },
    'msg.placeDescend': { zh: '视觉放置：垂直下探 {color}', en: 'Vision place: descend to {color}' },
    'msg.placeOpen': { zh: '视觉放置：打开夹爪释放 {color}', en: 'Vision place: open gripper, release {color}' },
    'msg.placeLift': { zh: '视觉放置：释放后抬升 {color}', en: 'Vision place: lift after release {color}' },
    'msg.avoidLift': { zh: '视觉避让：先抬离 {color}', en: 'Vision avoid: lift off {color} first' },
    'msg.avoidMove': { zh: '视觉避让：高位移动到 {color}', en: 'Vision avoid: high move to {color}' },
    'panel.expand': { zh: '展开侧边栏', en: 'Expand sidebar' },
    'log.gripperCmdShort': { zh: '夹爪指令 {mm} 毫米', en: 'Gripper command {mm} mm' },
    'log.jointBatch': { zh: '{label} -> ROS {n} 轴', en: '{label} -> ROS {n} joints' },
    'log.visionGraspPose': { zh: '视觉抓取姿态：{color} 夹持宽度 {mm}mm，yaw {yaw}deg，抬升 z={lift}，中转 z={transit}', en: 'Vision grasp pose: {color} grip {mm}mm, yaw {yaw}deg, lift z={lift}, transit z={transit}' },
    'log.visionYawFlip': { zh: '{color} 原夹持方向不可达，已切换到可达方向', en: '{color}: original grasp yaw unreachable; switched to a reachable orientation' },
    'log.visionHeightFallback': { zh: '{color} 请求的上方高度不可达，已采用 z={z}m', en: '{color}: requested approach height unreachable; using z={z}m' },
    'reason.mujocoCamera': { zh: 'MuJoCo 相机反馈', en: 'MuJoCo camera feedback' },
    'reason.mujocoPhysics': { zh: 'MuJoCo 物理关节反馈', en: 'MuJoCo physics joint feedback' },
    'reason.mujocoObject': { zh: 'MuJoCo 物体状态反馈', en: 'MuJoCo object-state feedback' },
    'fb.errors': { zh: '，错误 {codes}', en: ', errors {codes}' },
    'fb.visionTarget': { zh: '{color} x {x} y {y} z {z} / 夹紧 {mm}mm / yaw {yaw}deg', en: '{color} x {x} y {y} z {z} / grip {mm}mm / yaw {yaw}deg' },
    'joint.j1': { zh: 'J1 底座偏航', en: 'J1 Base Yaw' },
    'joint.j2': { zh: 'J2 肩部', en: 'J2 Shoulder' },
    'joint.j3': { zh: 'J3 肘部', en: 'J3 Elbow' },
    'joint.j4': { zh: 'J4 腕部俯仰', en: 'J4 Wrist Pitch' },
    'joint.j5': { zh: 'J5 腕部偏航', en: 'J5 Wrist Yaw' },
    'joint.j6': { zh: 'J6 工具旋转', en: 'J6 Tool Roll' },
    'joint.gripper': { zh: 'J7 夹爪', en: 'J7 Gripper' },
    'joint.degSuffix': { zh: '{val} 度', en: '{val}°' },
    'joint.gripSuffix': { zh: '{val} 毫米', en: '{val} mm' },
    'preset.ready': { zh: '就绪', en: 'Ready' },
    'preset.forward': { zh: '前方工作', en: 'Forward' },
    'preset.left': { zh: '左侧抓取', en: 'Left Pick' },
    'preset.right': { zh: '右侧放置', en: 'Right Place' },
    'preset.inspect': { zh: '检测', en: 'Inspect' },
    'preset.fold': { zh: '折叠', en: 'Fold' },
    'sim.exitDrag': { zh: '退出 TCP 拖拽', en: 'Exit TCP drag' },
    'sim.dragGreen': { zh: '拖动绿色 TCP 标记', en: 'Drag the green TCP marker' },
    'sim.edgeSnap': { zh: '边界吸附 · ', en: 'Edge snap · ' },
    'sim.errorMm': { zh: '误差 {mm}mm', en: 'Error {mm}mm' },
    'sim.converging': { zh: '收敛中 {mm}mm', en: 'Converging {mm}mm' },
    'sim.doneMm': { zh: '完成 {mm}mm', en: 'Done {mm}mm' },
    'sim.bestEffortMm': { zh: '已尽量收敛 {mm}mm', en: 'Best effort {mm}mm' },
    'sim.tcpDrag': { zh: 'TCP 拖拽', en: 'TCP drag' },
    'sim.tcpConverge': { zh: 'TCP 收敛', en: 'TCP converge' },
    'sim.generatedReady': { zh: '已生成：Ready -> 当前姿态', en: 'Generated: Ready -> current pose' },
    'sim.noReplay': { zh: '没有可回放的 waypoint', en: 'No waypoints to replay' },
    'sim.replaying': { zh: '正在回放示教轨迹', en: 'Replaying teach trajectory' },
    'sim.replayDone': { zh: '回放完成', en: 'Replay complete' },
    'sim.noExport': { zh: '没有可导出的 waypoint', en: 'No waypoints to export' },
    'sim.exported': { zh: '已导出 {n} 个 waypoint', en: 'Exported {n} waypoints' },
    'sim.stopRecord': { zh: '停止录制', en: 'Stop recording' },
    'sim.recording': { zh: '录制中：{n} 个 waypoint', en: 'Recording: {n} waypoints' },
    'sim.recorded': { zh: '已录制 {n} 个 waypoint，{sec} 秒', en: 'Recorded {n} waypoints, {sec} s' },
    'sim.unreachable': { zh: '目标不可达：{label}', en: 'Target unreachable: {label}' },
    'sim.clickPoint': { zh: '点击点', en: 'click point' },
    'sim.planTarget': { zh: '{label} -> TCP 上方 {mm}mm', en: '{label} -> above TCP {mm}mm' },
    'sim.reachText': { zh: '平面 {planar} / 估算 {workspace} 毫米 · 3D {spatial} 毫米', en: 'Planar {planar} / Est. {workspace} mm · 3D {spatial} mm' },
    'sim.mmShort': { zh: '{val}毫米', en: '{val}mm' },
    'sim.redBlock': { zh: '红色方块', en: 'Red block' },
    'sim.blueBlock': { zh: '蓝色方块', en: 'Blue block' },
   'sim.cylinder': { zh: '圆柱', en: 'Cylinder' },
   'sim.pickZone': { zh: '抓取区', en: 'Pick zone' },
   'sim.placeZone': { zh: '放置区', en: 'Place zone' },
   'client.connected': { zh: 'ROS 已连接', en: 'ROS connected' },
    'client.connecting': { zh: '正在连接 {url}', en: 'Connecting {url}' },
    'client.wsError': { zh: 'ROS WebSocket 出错', en: 'ROS WebSocket error' },
    'client.disconnected': { zh: 'ROS 连接已断开', en: 'ROS connection dropped' },
    'client.closed': { zh: 'ROS 已断开', en: 'ROS disconnected' },
    'client.notConnected': { zh: 'ROS 未连接', en: 'ROS not connected' },
    'client.badMsg': { zh: '收到无法解析的 ROS 消息', en: 'Received unparseable ROS message' },
    'client.serviceFailed': { zh: 'ROS 服务调用失败', en: 'ROS service call failed' },
    'log.batchDefault': { zh: '批量目标', en: 'Batch target' },
    'msg.simLowLevelSuffix': { zh: '{label}（仿真低层回放）', en: '{label} (sim low-level playback)' },
    'log.lowLevelSuffix': { zh: '{label}；低层回放', en: '{label}; low-level playback' },
    'msg.ikLowLevelSuffix': { zh: '{label}（IK 低层回放）', en: '{label} (IK low-level playback)' },
    'client.actionFailed': { zh: 'ROS 动作执行失败', en: 'ROS action failed' },
  };
  let currentLang = 'zh';
  try {
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved && SUPPORTED.indexOf(saved) >= 0) currentLang = saved;
  } catch (_) {}
  const listeners = new Set();
  function lookup(key) {
    const entry = DICT[key];
    if (!entry) return null;
    return entry[currentLang] != null ? entry[currentLang] : entry.zh;
  }
  function t(key, params) {
    let str = lookup(key);
    if (str == null) return key;
    if (params && typeof str === 'string') {
      str = str.replace(/\{(\w+)\}/g, (_, name) => (params[name] != null ? String(params[name]) : `{${name}}`));
    }
    return str;
  }
  function applyI18n(root) {
    const scope = root || document;
    scope.querySelectorAll('[data-i18n]').forEach((el) => {
      const val = lookup(el.getAttribute('data-i18n'));
      if (val != null) el.textContent = val;
    });
    scope.querySelectorAll('[data-i18n-html]').forEach((el) => {
      const val = lookup(el.getAttribute('data-i18n-html'));
      if (val != null) el.innerHTML = val;
    });
    scope.querySelectorAll('[data-i18n-ph]').forEach((el) => {
      const val = lookup(el.getAttribute('data-i18n-ph'));
      if (val != null) el.setAttribute('placeholder', val);
    });
    scope.querySelectorAll('[data-i18n-title]').forEach((el) => {
      const val = lookup(el.getAttribute('data-i18n-title'));
      if (val != null) el.setAttribute('title', val);
    });
    document.documentElement.lang = currentLang === 'en' ? 'en' : 'zh-CN';
  }
  function setLang(lang) {
    if (SUPPORTED.indexOf(lang) < 0 || lang === currentLang) return;
    currentLang = lang;
    try { localStorage.setItem(STORAGE_KEY, lang); } catch (_) {}
    applyI18n(document);
    listeners.forEach((fn) => { try { fn(lang); } catch (_) {} });
  }
  function getLang() { return currentLang; }
  function onLangChange(fn) {
    if (typeof fn === 'function') listeners.add(fn);
    return () => listeners.delete(fn);
  }
  function initLangToggle() {
    const toggle = document.getElementById('lang-toggle');
    if (!toggle) return;
    function syncToggle() {
      const lang = getLang();
      toggle.querySelectorAll('.lang-btn').forEach((btn) => {
        btn.classList.toggle('active', btn.dataset.lang === lang);
      });
    }
    toggle.addEventListener('click', (e) => {
      const btn = e.target.closest('.lang-btn');
      if (!btn) return;
      setLang(btn.dataset.lang);
    });
    onLangChange(syncToggle);
    syncToggle();
  }
  window.rebotI18n = { t, setLang, getLang, applyI18n, onLangChange, dict: DICT };
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => { applyI18n(document); initLangToggle(); });
  } else {
    applyI18n(document);
    initLangToggle();
  }
})();
