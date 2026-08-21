(function () {
const NS = 'rebotarm';
 const t = window.rebotI18n ? window.rebotI18n.t : (k) => k;
 const URL_STORAGE_KEY = 'rebotarm.ros.url';
  function loadSavedUrl() { try { return localStorage.getItem(URL_STORAGE_KEY) || ''; } catch (_) { return ''; } }
  function saveUrl(url) { try { localStorage.setItem(URL_STORAGE_KEY, url); } catch (_) {} }
 const OPEN_GRIPPER_M = 0.09;
  const CLOSE_GRIPPER_M = 0;
  const GRIPPER_BASE_GAP_M = 0;
  const GRIPPER_VISUAL_TRAVEL_M = 0.057;
  const GRIPPER_EFFECTIVE_GAP_M = 0.057;
  const GRIPPER_CLOSED_DISPLAY_SNAP_M = 0.003;
  const GRASP_SQUEEZE_M = 0.004;
  const MIN_OBJECT_GRASP_M = 0.018;
  const VISION_TRANSIT_Z_M = 0.32;
  const VISION_TRANSIT_Z_BY_COLOR_M = {
    blue: 0.410
  };
  const VISION_MOVE_ABOVE_FALLBACK_Z_BY_COLOR_M = {
    blue: 0.180
  };
  const VISION_FIRST_LIFT_CLEARANCE_M = 0.085;
  const VISION_FIRST_LIFT_MIN_M = 0.275;
  const VISION_POSE_SKIP_M = 0.006;
  const VISION_VERTICAL_ALIGN_CLEARANCE_M = 0.075;
  const VISION_PREGRASP_CLEARANCE_M = 0.038;
  const VISION_MIN_VERTICAL_ALIGN_Z_M = 0.235;
  const VISION_FIRST_LIFT_MIN_BY_COLOR_M = {
    blue: 0.390
  };
  const JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
  const REQUIRED_TOPICS = {
    jointStates: `/${NS}/joint_states`,
    physicsJointStates: `/${NS}/mujoco/physics_joint_states`,
    armStatus: `/${NS}/arm_status`,
    gripper: `/${NS}/gripper/state`,
    cameraImage: `/${NS}/mujoco/overhead_rgb/image_raw`,
    objectStates: `/${NS}/mujoco/object_states`,
    visionDetections: `/${NS}/vision/color_blocks/detections`,
    simAnimation: `/${NS}/sim/animation_event`
  };
  const REQUIRED_SERVICES = {
    gravityStart: `/${NS}/gravity_compensation/start`,
    gravityStop: `/${NS}/gravity_compensation/stop`,
    gravityStatus: `/${NS}/gravity_compensation/status`,
    recordStart: `/${NS}/mujoco/record/start`,
    recordStop: `/${NS}/mujoco/record/stop`,
    recordReplay: `/${NS}/mujoco/record/replay`,
    recordClear: `/${NS}/mujoco/record/clear`
  };

  const els = {
    url: document.getElementById('ros-url'),
    connect: document.getElementById('ros-connect'),
    disconnect: document.getElementById('ros-disconnect'),
    safeDisconnect: document.getElementById('ros-safe-disconnect'),
    mirror: document.getElementById('ros-mirror'),
    control: document.getElementById('ros-control-enable'),
    status: document.getElementById('ros-status'),
    message: document.getElementById('ros-message'),
    feedbackError: document.getElementById('ros-feedback-error'),
    enable: document.getElementById('ros-enable'),
    disable: document.getElementById('ros-disable'),
    safeHome: document.getElementById('ros-safe-home'),
    gravityStatus: document.getElementById('ros-gravity-status'),
    gravityStart: document.getElementById('ros-gravity-start'),
    gravityStop: document.getElementById('ros-gravity-stop'),
    gravityQuery: document.getElementById('ros-gravity-status-query'),
   rosOpenGripper: document.getElementById('ros-open-gripper'),
   closeGripper: document.getElementById('ros-close-gripper'),
   clearLog: document.getElementById('ros-clear-log'),
   log: document.getElementById('ros-log'),
   cameraCanvas: document.getElementById('ros-camera-canvas'),
    cameraStatus: document.getElementById('ros-camera-status'),
    cameraTopic: document.getElementById('ros-camera-topic'),
    visionStatus: document.getElementById('ros-vision-status'),
    visionTarget: document.getElementById('ros-vision-target'),
    visionColor: document.getElementById('ros-vision-color'),
    visionApproachZ: document.getElementById('ros-vision-approach-z'),
    visionGraspZ: document.getElementById('ros-vision-grasp-z'),
    visionFillPose: document.getElementById('ros-vision-fill-pose'),
    visionMoveAbove: document.getElementById('ros-vision-move-above'),
    visionPickDemo: document.getElementById('ros-vision-pick-demo'),
    visionPlaceDemo: document.getElementById('ros-vision-place-demo'),
    vlim: document.getElementById('ros-vlim'),
    trajectoryDuration: document.getElementById('ros-trajectory-duration'),
    poseX: document.getElementById('ros-pose-x'),
    poseY: document.getElementById('ros-pose-y'),
    poseZ: document.getElementById('ros-pose-z'),
    poseDuration: document.getElementById('ros-pose-duration'),
   checkIk: document.getElementById('ros-check-ik'),
   stopPath: document.getElementById('stop-path')
  };

 if (!window.ReBotRosClient || !els.connect) return;

  if (els.url && !els.url.value) {
    const saved = loadSavedUrl();
    if (saved) els.url.value = saved;
  }
  const client = new window.ReBotRosClient({ namespace: NS, url: els.url ? els.url.value : '' });
 window.reBotRos = client;

  const lastSent = new Map();
  const simTargetAngles = new Map();
  const mirrorHoldUntil = new Map();
  const COMMAND_INTERVAL_MS = 45;
 const MIRROR_HOLD_MS = 1800;
  let latestJointPositions = null;
  let latestJointStateAt = 0;
  let latestPhysicsJointStateAt = 0;
  let latestGripperPosition = null;
  let latestGripperVelocity = null;
  let latestGripperAt = 0;
  let listedTopics = new Set();
  let listedServices = new Set();
  let listedActionServers = new Set();
  let mujocoSyncAnnounced = false;
  let fakeDriverDetected = false;
  let lowLevelPlayback = null;
  let lastTargetPoseSent = 0;
  let latestVisionPayload = null;
  let latestVisionAt = 0;
  let selectedVisionTarget = null;
  let lastVisionTarget = null;
  let heldVisionTarget = null;
  let autoVisionTargetColor = '';
  let visionSequenceBusy = false;
  let lastVisionOp = null;
  let safeDisconnectBusy = false;
  let gravityCompensationActive = false;
  let lastStatusState = null;
  let gravityStatusSource = 'initial';
  let gravityStatusPollInFlight = false;

  client.subscribe(REQUIRED_TOPICS.jointStates, 'sensor_msgs/msg/JointState', (msg) => handleJointStates(msg, false), { throttleRate: 80 });
  client.subscribe(REQUIRED_TOPICS.physicsJointStates, 'sensor_msgs/msg/JointState', (msg) => handleJointStates(msg, true), { throttleRate: 33 });
  client.subscribe(REQUIRED_TOPICS.gripper, 'rebotarm_msgs/msg/JointMotorState', handleGripperState, { throttleRate: 80 });
  client.subscribe(REQUIRED_TOPICS.armStatus, 'rebotarm_msgs/msg/ArmStatus', handleArmStatus, { throttleRate: 200 });
  client.subscribe(REQUIRED_TOPICS.cameraImage, 'sensor_msgs/msg/Image', handleCameraImage, { throttleRate: 250 });
  client.subscribe(REQUIRED_TOPICS.objectStates, 'std_msgs/msg/String', handleMujocoObjectStates, { throttleRate: 33 });
  client.subscribe(REQUIRED_TOPICS.visionDetections, 'std_msgs/msg/String', handleVisionDetections, { throttleRate: 180 });
  client.subscribe(REQUIRED_TOPICS.simAnimation, 'std_msgs/msg/String', handleSimAnimationEvent, { throttleRate: 0 });
  if (els.cameraTopic) els.cameraTopic.textContent = REQUIRED_TOPICS.cameraImage;

  client.addEventListener('status', (event) => {
    const detail = event.detail || {};
    setStatus(detail.state, detail.message);
    if (detail.state !== 'connecting') {
      writeLog(detail.message || detail.state, detail.state === 'error' ? 'error' : detail.state === 'open' ? 'ok' : 'info');
    }
    updateDiagnostics();
   if (detail.state === 'closed' || detail.state === 'error') {
      latestPhysicsJointStateAt = 0;
      mujocoSyncAnnounced = false;
      fakeDriverDetected = false;
      updateGravityStatus(false, t('msg.rosNotConnected'), 'connection');
   }
    if (detail.state === 'open') {
      latestPhysicsJointStateAt = 0;
      mujocoSyncAnnounced = false;
      fakeDriverDetected = false;
      window.setTimeout(() => {
        runDiagnostics();
      }, 250);
    }
  });

  els.connect.addEventListener('click', () => {
   const nextUrl = els.url.value.trim();
   if (!canConnectWebSocketUrl(nextUrl)) return;
    saveUrl(nextUrl);
   client.autoReconnect = true;
    client.connect(nextUrl);
  });
  els.disconnect.addEventListener('click', disconnectRos);
  window.addEventListener('pagehide', () => {
    client.autoReconnect = false;
    if (client.socket) client.socket.close();
  });
  els.enable.addEventListener('click', () => guardedCall(() => client.enable(), t('msg.reqEnable')));
  els.disable.addEventListener('click', () => {
    cancelLowLevelPlayback();
    guardedCall(() => client.disable(), t('msg.reqDisable'), true);
  });
  els.safeHome.addEventListener('click', () => guardedCall(() => client.safeHome(), t('msg.reqSafeHome')));
  els.gravityStart.addEventListener('click', () => {
    cancelLowLevelPlayback();
    guardedOptionalService(
      REQUIRED_SERVICES.gravityStart,
      () => client.startGravityCompensation(),
      t('msg.reqGravityStart')
    );
  });
  els.gravityStop.addEventListener('click', () => {
    cancelLowLevelPlayback();
    guardedOptionalService(
      REQUIRED_SERVICES.gravityStop,
      () => client.stopGravityCompensation(),
      t('msg.reqGravityStop'),
      true
    );
  });
  els.gravityQuery.addEventListener('click', queryGravityCompensation);
 els.rosOpenGripper.addEventListener('click', () => sendGripper(OPEN_GRIPPER_M, { requireControl: true }));
 els.closeGripper.addEventListener('click', () => sendGripper(CLOSE_GRIPPER_M, { requireControl: true }));
 els.clearLog.addEventListener('click', () => { els.log.innerHTML = ''; });
  els.checkIk.addEventListener('click', checkIk);
  document.getElementById('ros-help-top')?.addEventListener('click', () => document.getElementById('ros-help-dialog')?.showModal());
  document.getElementById('ros-help-close')?.addEventListener('click', () => document.getElementById('ros-help-dialog')?.close());
  const sidebar = document.querySelector('.control-panel');
  const appShell = document.querySelector('.app-shell');
  const collapseBtn = document.getElementById('sidebar-collapse');
  collapseBtn?.addEventListener('click', () => {
    if (!sidebar) return;
    sidebar.classList.toggle('collapsed');
    const collapsed = sidebar.classList.contains('collapsed');
    collapseBtn.textContent = collapsed ? '▶' : '◀';
    collapseBtn.title = collapsed ? t('panel.expand') : t('panel.collapse');
 });
 if (els.visionColor) els.visionColor.addEventListener('change', () => {
    if (els.visionColor.value === 'auto') autoVisionTargetColor = '';
    if (!visionSequenceBusy) updateSelectedVisionTarget();
  });
  if (els.visionFillPose) els.visionFillPose.addEventListener('click', fillPoseFromVisionTarget);
  if (els.visionMoveAbove) els.visionMoveAbove.addEventListener('click', moveAboveVisionTarget);
  if (els.visionPickDemo) els.visionPickDemo.addEventListener('click', runVisionPickDemo);
  if (els.visionPlaceDemo) els.visionPlaceDemo.addEventListener('click', runVisionPlaceDemo);
  if (els.stopPath) {
    els.stopPath.addEventListener('click', () => {
      cancelLowLevelPlayback();
      writeLog(t('log.stopPlayback'), 'warn');
    });
  }

  els.control.addEventListener('change', () => {
    if (els.control.checked) writeLog(t('log.controlLockOpen'), 'info');
  });

  waitForSimApi((sim) => sim.onCommand((command) => forwardSimCommand(command)));

  setStatus('closed', t('msg.rosNotConnected'));
  updateDiagnostics();
  window.setInterval(updateDiagnostics, 1000);
  window.setInterval(pollGravityCompensationStatus, 500);

  function handleJointStates(msg, isPhysicsFeedback) {
    if (!window.reBotSim || !Array.isArray(msg.name) || !Array.isArray(msg.position)) return;
    const now = performance.now();
    if (isPhysicsFeedback) {
      latestPhysicsJointStateAt = now;
      announceMujocoSync(t('reason.mujocoPhysics'));
    }
    const useForDisplay = isPhysicsFeedback || !hasFreshPhysicsJointFeedback(now);
    const next = {};
    msg.name.forEach((name, index) => {
      const simName = normalizeJointName(name);
      if (!simName || typeof msg.position[index] !== 'number') return;
      next[simName] = msg.position[index];
    });

    // /joint_states is the canonical controller/hardware feedback in every
    // deployment, including real-hardware + MuJoCo digital-twin mode.  Keep it
    // separate from MuJoCo physics feedback so display synchronization cannot
    // change trajectory starts or gripper completion checks.
    if (!isPhysicsFeedback && Object.keys(next).length) {
      latestJointPositions = { ...(latestJointPositions || {}), ...next };
      latestJointStateAt = now;
    }
    if (!isPhysicsFeedback) updateFeedbackError(next);

    if (useForDisplay && (isPhysicsFeedback || els.mirror.checked) && Object.keys(next).length) {
      const mirrored = {};
      Object.entries(next).forEach(([name, value]) => {
        // The hardware publishes a single finger travel in /joint_states.  The
        // simulator's `gripper` joint represents the complete opening and must
        // drive both fingers symmetrically, so do not write either URDF finger
        // joint directly here.
        if (name === 'finger_left' || name === 'finger_right') return;
        const holdUntil = mirrorHoldUntil.get(name) || 0;
        const target = simTargetAngles.get(name);
        const reachedTarget = typeof target === 'number' && Math.abs(target - value) < 0.025;
        if (isPhysicsFeedback || reachedTarget || now > holdUntil) {
          mirrorHoldUntil.delete(name);
          mirrored[name] = value;
        }
      });

      const leftOpening = Number(next.finger_left);
      const rightOpening = Number(next.finger_right);
      const fingerOpening = Number.isFinite(leftOpening)
        ? leftOpening
        : (Number.isFinite(rightOpening) ? -rightOpening : NaN);
      if (Number.isFinite(fingerOpening)) {
        const gripperWidth = snapClosedGripperForDisplay(fingerOpeningToGripperCommand(fingerOpening));
        const holdUntil = mirrorHoldUntil.get('gripper') || 0;
        const target = simTargetAngles.get('gripper');
        const reachedTarget = typeof target === 'number' && Math.abs(target - gripperWidth) < 0.003;
        if (isPhysicsFeedback || reachedTarget || now > holdUntil) {
          mirrorHoldUntil.delete('gripper');
          mirrored.gripper = gripperWidth;
        }
      }
      if (Object.keys(mirrored).length) {
        window.reBotSim.setAngles(mirrored, {
          source: isPhysicsFeedback ? 'mujoco-physics' : 'ros',
          emit: false
        });
      }
    }
    updateDiagnostics();
  }

  function handleGripperState(msg) {
    if (typeof msg.position === 'number') {
      latestGripperPosition = msg.position;
      latestGripperAt = performance.now();
    }
    if (typeof msg.velocity === 'number') {
      latestGripperVelocity = msg.velocity;
    }
    if (els.mirror.checked && !hasFreshPhysicsJointFeedback() && window.reBotSim && typeof msg.position === 'number') {
      const holdUntil = mirrorHoldUntil.get('gripper') || 0;
      const target = simTargetAngles.get('gripper');
      const reachedTarget = typeof target === 'number' && Math.abs(target - msg.position) < 0.003;
      if (reachedTarget || performance.now() > holdUntil) {
        mirrorHoldUntil.delete('gripper');
        window.reBotSim.setGripperWidth(snapClosedGripperForDisplay(msg.position), { source: 'ros', animate: false });
      }
    }
    if (!visionSequenceBusy && typeof msg.position === 'number' && simTargetAngles.has('gripper')) {
      const target = simTargetAngles.get('gripper');
      const err = Math.abs(target - msg.position);
     if (err < 0.003) {
        setMessage(t('msg.gripperArrived', {mm: Math.round(msg.position * 1000)}));
     } else {
        setMessage(t('msg.gripperMoving', {cmd: Math.round(target * 1000), mm: Math.round(msg.position * 1000)}));
     }
    }
    updateDiagnostics();
  }

  function handleArmStatus(msg) {
    fakeDriverDetected = String(msg && msg.mode || '').toLowerCase().startsWith('fake_');
    const enabled = msg.enabled ? t('st.enabled') : t('st.disabled');
    const mode = msg.mode || 'unknown';
   const machine = msg.state_machine || 'unknown';
    const errors = Array.isArray(msg.error_codes) && msg.error_codes.length ? t('fb.errors', {codes: msg.error_codes.join(', ')}) : '';
   if (!visionSequenceBusy) {
      setMessage(t('fb.armStatus', {enabled, mode, machine, errors}));
   }
    updateGravityStatus(machine === 'GRAVITY_COMP', machine, 'arm');
    updateDiagnostics();
  }

  function forwardSimCommand(command) {
    if (command && command.type === 'tcp-target') {
      forwardTcpTarget(command);
      return;
    }
    if (command && command.type === 'joint-batch') {
      forwardJointBatch(command);
      return;
    }
    if (!command || command.type !== 'joint') return;
    simTargetAngles.set(command.name, command.value);
    mirrorHoldUntil.set(command.name, performance.now() + MIRROR_HOLD_MS);

    if (els.mirror.checked && !els.control.checked && command.source === 'slider') {
      els.mirror.checked = false;
      writeLog(t('log.mirrorPaused'), 'warn');
    }

    if (!controlAllowed(false)) return;

    const now = performance.now();
    const last = lastSent.get(command.name) || 0;
    if (now - last < COMMAND_INTERVAL_MS) return;
    lastSent.set(command.name, now);

    if (command.name === 'gripper') {
     client.publishGripperCommand(command.value);
      writeLog(t('log.gripperCmdShort', {mm: (command.value * 1000).toFixed(0)}), 'info');
     return;
    }
    client.publishJointCommand(command.name, command.value, { vlim: getVlim() });
  }

  function forwardTcpTarget(command) {
    if (!client.connected || !command || !command.target_ros) return;
    const now = performance.now();
    if (now - lastTargetPoseSent < COMMAND_INTERVAL_MS) return;
    lastTargetPoseSent = now;
    client.publishTargetPose({
      position: command.target_ros,
      orientation: { x: 0, y: 0, z: 0, w: 1 }
    });
  }

  function forwardJointBatch(command) {
    const joints = command && command.joints && typeof command.joints === 'object' ? command.joints : {};
    const names = [...JOINT_NAMES, 'gripper'].filter((name) => typeof joints[name] === 'number' && Number.isFinite(joints[name]));
    if (!names.length) return;

    const holdUntil = performance.now() + MIRROR_HOLD_MS;
    names.forEach((name) => {
      simTargetAngles.set(name, joints[name]);
      mirrorHoldUntil.set(name, holdUntil);
    });

    if (!controlAllowed(false)) return;

    names.forEach((name) => {
      lastSent.set(name, 0);
      if (name === 'gripper') {
        client.publishGripperCommand(joints[name]);
      } else {
        client.publishJointCommand(name, joints[name], { vlim: getVlim() });
      }
   });
    writeLog(t('log.jointBatch', {label: command.label || command.source || t('log.batchDefault'), n: names.length}), 'ok');
 }

  async function checkIk() {
    if (!controlAllowed(true)) return;
    const pose = readPose();
    const duration = getPoseDuration();
    client.publishTargetPose(pose);
    await guardedCall(
     () => client.moveToPose(pose, duration),
      t('msg.reqIkMove', {sec: duration.toFixed(1)}),
     true
    );
  }

  async function queryGravityCompensation(options) {
    const result = await guardedOptionalService(
      REQUIRED_SERVICES.gravityStatus,
      () => client.gravityCompensationStatus(),
      t('msg.reqGravityQuery'),
      true,
      options
    );
    if (result) updateGravityStatus(Boolean(result.success), result.message || '', 'service');
  }

  async function pollGravityCompensationStatus() {
    if (
      !client.connected ||
      !gravityCompensationActive ||
      gravityStatusPollInFlight
    ) return;

    gravityStatusPollInFlight = true;
    try {
      await queryGravityCompensation({ silent: true });
    } finally {
      gravityStatusPollInFlight = false;
    }
  }

  async function runDiagnostics() {
    updateDiagnostics();
    if (!client.connected) {
      writeLog(t('log.rosOfflineFirst'), 'warn');
      return;
    }
    try {
      const [topics, services, actions] = await Promise.all([
        client.getRosTopics(),
        client.getRosServices(),
        client.getRosActionServers()
      ]);
      const topicList = topics.topics || [];
      const serviceList = services.services || [];
      const actionList = actions.action_servers || [];
      listedTopics = new Set(topicList);
      listedServices = new Set(serviceList);
      listedActionServers = new Set(actionList);
      writeLog(
        `rosapi: ${topicList.length} topics, ${serviceList.length} services, ${actionList.length} actions`,
        'ok'
      );
      if (els.visionStatus && !topicList.includes(REQUIRED_TOPICS.visionDetections)) {
        els.visionStatus.textContent = t('st.waitNode');
      }
      if (!listedServices.has(REQUIRED_SERVICES.gravityStatus)) {
        updateGravityStatus(false, t('st.serviceUnavailable'));
      }
      if (!hasActionServer(`/${NS}/follow_joint_trajectory`)) {
        writeLog(t('log.lowLevelFallbackInfo'), 'info');
      }
   } catch (error) {
      writeLog(t('log.rosapiFallback', {err: error.message || error}), 'warn');
   }
  }

  function buildTrajectoryPoints(waypoints, totalDuration) {
    const firstT = waypoints[0].t || 0;
    const lastT = waypoints[waypoints.length - 1].t || firstT + 1;
    const span = Math.max(lastT - firstT, 1);
    const points = [makeTrajectoryPoint(getCurrentRosPositions(), 0.05)];
    waypoints.forEach((point, index) => {
      const ratio = waypoints.length === 1 ? 1 : Math.max(0, (point.t - firstT) / span);
      const seconds = Math.max(0.2, index === waypoints.length - 1 ? totalDuration : ratio * totalDuration);
      points.push(makeTrajectoryPoint(JOINT_NAMES.map((name) => Number(point.joints[name] || 0)), seconds));
    });
    return points;
  }

  async function sendTrajectory(points, optimisticMessage) {
    if (!points.length) return;
   if (shouldUseLowLevelTrajectory()) {
      setMessage(t('msg.simLowLevelSuffix', {label: optimisticMessage}));
      writeLog(t('log.lowLevelSuffix', {label: optimisticMessage}), 'info');
     await replayTrajectoryLowLevel(points);
      return;
    }
    if (!hasActionServer(`/${NS}/follow_joint_trajectory`)) {
      writeLog(t('log.lowLevelFallbackWarn'), 'warn');
      await replayTrajectoryLowLevel(points);
      return;
    }
    await guardedCall(() => client.followJointTrajectory(JOINT_NAMES, points), optimisticMessage);
  }

  async function replayTrajectoryLowLevel(points) {
    cancelLowLevelPlayback();
    const playback = { cancelled: false };
    lowLevelPlayback = playback;
   const started = performance.now();
    writeLog(t('log.lowLevelStart', {n: points.length}), 'ok');
   for (const point of points) {
      if (playback.cancelled || !controlAllowed(false)) break;
      const targetMs = rosTimeToSeconds(point.time_from_start) * 1000;
      const waitMs = Math.max(0, targetMs - (performance.now() - started));
      if (waitMs > 0) await sleep(waitMs);
      JOINT_NAMES.forEach((name, index) => {
        const pos = Number(point.positions[index]);
        if (Number.isFinite(pos)) {
          simTargetAngles.set(name, pos);
          client.publishJointCommand(name, pos, { vlim: getVlim() });
        }
      });
      syncSimArmFromTrajectoryPoint(point);
    }
    if (lowLevelPlayback === playback) lowLevelPlayback = null;
    writeLog(playback.cancelled ? t('log.lowLevelCancelled') : t('log.lowLevelDone'), playback.cancelled ? 'warn' : 'ok');
  }

  function syncSimArmFromTrajectoryPoint(point) {
    if (!window.reBotSim || typeof window.reBotSim.setAngles !== 'function') return;
    if (hasFreshPhysicsJointFeedback()) return;
    const angles = {};
    JOINT_NAMES.forEach((name, index) => {
      const pos = Number(point.positions[index]);
      if (Number.isFinite(pos)) angles[name] = pos;
    });
    if (Object.keys(angles).length) {
      window.reBotSim.setAngles(angles, { source: 'trajectory-playback' });
    }
  }

  function cancelLowLevelPlayback() {
    if (lowLevelPlayback) lowLevelPlayback.cancelled = true;
  }

  function shouldUseLowLevelTrajectory() {
    // Control routing is independent from the display feedback source.  A
    // hardware controller and MuJoCo digital twin may legitimately run at the
    // same time.  The fake driver identifies pure simulation explicitly and
    // keeps its proven low-level playback path; real hardware uses its action
    // server even while MuJoCo synchronization is active.
    const simulationTaskServerDetected = listedServices.has(REQUIRED_SERVICES.recordStart);
    return fakeDriverDetected
      || simulationTaskServerDetected
      || !hasActionServer(`/${NS}/follow_joint_trajectory`);
  }

  function announceMujocoSync(reason) {
    if (mujocoSyncAnnounced) return;
    mujocoSyncAnnounced = true;
    writeLog(t('log.mujocoSyncActive', {reason}), 'info');
 }

  function hasActionServer(actionName) {
    return listedActionServers.has(actionName);
  }

  function makeTrajectoryPoint(positions, seconds) {
    return {
      positions,
      velocities: JOINT_NAMES.map(() => 0),
      accelerations: [],
      effort: [],
      time_from_start: secondsToRosTime(seconds)
    };
  }

  function getCurrentRosPositions() {
    const source = latestJointPositions || (window.reBotSim && window.reBotSim.getAngles ? window.reBotSim.getAngles() : {});
    return JOINT_NAMES.map((name) => Number(source[name] || 0));
  }

  function getTeachWaypoints() {
    if (!window.reBotSim || typeof window.reBotSim.getTeachingWaypoints !== 'function') return [];
    return window.reBotSim.getTeachingWaypoints().filter((point) => point && point.joints);
  }

  function readPose() {
    return {
      position: {
        x: Number(els.poseX.value) || 0,
        y: Number(els.poseY.value) || 0,
        z: Number(els.poseZ.value) || 0
      },
      orientation: { x: 0, y: 0, z: 0, w: 1 }
    };
  }

  function controlAllowed(interactive) {
    if (!client.connected) {
      if (interactive) setStatus('closed', t('msg.rosNotConnected'));
      return false;
    }
    if (!els.control.checked) {
      if (interactive) setMessage(t('msg.controlLockClosed'));
      return false;
    }
    return true;
  }

  function canConnectWebSocketUrl(url) {
    if (window.location.protocol === 'https:' && /^ws:\/\//i.test(url)) {
      const message = t('msg.httpsWsBlocked');
      setStatus('error', message);
      writeLog(message, 'error');
      return false;
    }
    return true;
  }

  async function guardedOptionalService(serviceName, call, optimisticMessage, allowWithoutControl, options) {
   if (listedServices.size && !listedServices.has(serviceName)) {
      const message = t('msg.serviceNotFound', {name: serviceName});
     updateGravityStatus(false, t('st.serviceUnavailable'));
      setMessage(message);
      if (!(options && (options.auto || options.silent))) writeLog(message, 'warn');
      return null;
    }
    return guardedCall(call, optimisticMessage, allowWithoutControl, {
      keepConnectionStatus: true,
      silent: Boolean(options && options.silent)
    });
  }

  async function disconnectRos() {
    if (safeDisconnectBusy) return;
    cancelLowLevelPlayback();

    if (!els.safeDisconnect || !els.safeDisconnect.checked || !client.connected) {
      client.disconnect();
      return;
    }

    safeDisconnectBusy = true;
    els.disconnect.disabled = true;
    try {
      setMessage(t('msg.disconnectHome'));
      writeLog(t('log.disconnectHomeStart'), 'info');
      const homeResult = await client.safeHome();
      if (homeResult && homeResult.success === false) {
        throw new Error(homeResult.message || t('msg.safeHomeFail'));
      }
      writeLog(t('log.disconnectHomeDone'), 'ok');

      setMessage(t('msg.disconnectDisable'));
      writeLog(t('log.disconnectDisableStart'), 'info');
      const disableResult = await client.disable();
      if (disableResult && disableResult.success === false) {
        throw new Error(disableResult.message || t('msg.disableFail'));
      }
      writeLog(t('log.disconnectDisableDone'), 'ok');
      client.disconnect();
   } catch (error) {
      const message = t('msg.disconnectGuardFail', {err: error && error.message ? error.message : error});
     setMessage(message);
      writeLog(message, 'error');
    } finally {
      safeDisconnectBusy = false;
      els.disconnect.disabled = false;
    }
  }

  async function guardedCall(call, optimisticMessage, allowWithoutControl, options) {
    if (!client.connected) {
      setStatus('closed', t('msg.rosNotConnected'));
      return null;
    }
    if (!allowWithoutControl && !controlAllowed(false)) {
      setMessage(t('msg.controlLockClosed'));
      return null;
    }
    try {
      if (!(options && options.silent)) {
        setMessage(optimisticMessage);
        writeLog(optimisticMessage, 'info');
      }
      const result = await call();
      const message = formatServiceResult(result);
      if (!(options && options.silent)) {
        setMessage(message);
        writeLog(message, result && result.accepted === false ? 'warn' : 'ok');
      }
      return result;
    } catch (error) {
      const message = error && error.message ? error.message : t('log.rosCallFail');
      if (options && options.silent) return null;
      if (options && options.keepConnectionStatus && client.connected) {
        setMessage(message);
      } else {
        setStatus('error', message);
      }
      writeLog(message, 'error');
      return null;
    }
  }

  function formatServiceResult(result) {
    if (!result) return t('log.rosCallDone');
    if (typeof result.accepted === 'boolean') return result.accepted ? t('msg.goalAccepted') : t('msg.goalRejected');
   if (typeof result.message === 'string' && result.message) return result.message;
    if (typeof result.reached_position === 'number') return t('msg.gripperReached', {mm: Math.round(result.reached_position * 1000)});
    if (Array.isArray(result.q_solution)) return t('log.ikResult', {result: result.success ? t('log.ikSuccess') : t('log.ikFail'), q: result.q_solution.map((v) => Number(v).toFixed(3)).join(', ')});
   if (typeof result.success === 'boolean') return result.success ? t('log.rosCallSuccess') : t('log.rosCallFail');
    return t('log.rosCallDone');
  }

  function updateDiagnostics() {
    updateCameraStatusFromTopic();
  }

  function markTopicDiag(el, topic) {
    const last = client.getLastMessageAt(topic);
    if (!client.connected) {
      markDiag(el, false, '--');
      return;
    }
    if (!last) {
      markDiag(el, null, listedTopics.has(topic) ? (topic === REQUIRED_TOPICS.armStatus ? t('st.diagFound') : t('st.diagFoundWait')) : t('st.diagWait'));
      return;
    }
    const age = (Date.now() - last) / 1000;
    const liveLimit = topic === REQUIRED_TOPICS.armStatus ? 90 : topic === REQUIRED_TOPICS.cameraImage ? 3.0 : 2.5;
    markDiag(el, age < liveLimit, `${age.toFixed(1)}s`);
  }

  function markDiag(el, ok, text) {
    if (!el) return;
    const box = el.closest('.diag-item');
    if (box) {
      box.classList.toggle('ok', ok === true);
      box.classList.toggle('warn', ok === null);
      box.classList.toggle('bad', ok === false);
    }
    el.textContent = text;
  }

  function normalizeJointName(name) {
    const text = String(name || '').toLowerCase();
    if (text.endsWith('finger_left') || text.endsWith('/finger_left')) return 'finger_left';
    if (text.endsWith('finger_right') || text.endsWith('/finger_right')) return 'finger_right';
    const match = text.match(/joint[_-]?([1-6])$/) || text.match(/j([1-6])$/);
    return match ? `joint${match[1]}` : null;
  }

  function hasFreshPhysicsJointFeedback(now) {
    const current = Number.isFinite(Number(now)) ? Number(now) : performance.now();
    return latestPhysicsJointStateAt > 0 && current - latestPhysicsJointStateAt < 500;
  }

  function handleCameraImage(msg) {
    announceMujocoSync(t('reason.mujocoCamera'));
    if (!els.cameraCanvas || !msg) return;
    const width = Number(msg.width) || 0;
    const height = Number(msg.height) || 0;
    if (width <= 0 || height <= 0) {
      setCameraStatus(t('st.cameraError'), 'error');
      return;
    }

    const bytes = rosImageBytes(msg.data);
    if (!bytes) {
      setCameraStatus(t('st.cameraDataError'), 'error');
      return;
    }

    const encoding = String(msg.encoding || 'rgb8').toLowerCase();
    const channels = encoding === 'rgba8' || encoding === 'bgra8' ? 4 : 3;
    const supported = encoding === 'rgb8' || encoding === 'bgr8' || encoding === 'rgba8' || encoding === 'bgra8';
    if (!supported) {
      setCameraStatus(encoding || t('st.encodingUnsupported'), 'warn');
      return;
    }

    if (els.cameraCanvas.width !== width) els.cameraCanvas.width = width;
    if (els.cameraCanvas.height !== height) els.cameraCanvas.height = height;
    const ctx = els.cameraCanvas.getContext('2d');
    const frame = ctx.createImageData(width, height);
    const dst = frame.data;
    const step = Number(msg.step) || width * channels;
    const bgr = encoding === 'bgr8' || encoding === 'bgra8';

    for (let y = 0; y < height; y += 1) {
      const row = y * step;
      for (let x = 0; x < width; x += 1) {
        const src = row + x * channels;
        const out = (y * width + x) * 4;
        dst[out] = bgr ? bytes[src + 2] : bytes[src];
        dst[out + 1] = bytes[src + 1];
        dst[out + 2] = bgr ? bytes[src] : bytes[src + 2];
        dst[out + 3] = channels === 4 ? bytes[src + 3] : 255;
      }
    }

    ctx.putImageData(frame, 0, 0);
    setCameraStatus(`${width}x${height}`, 'online');
    updateDiagnostics();
  }

  function rosImageBytes(data) {
    if (!data) return null;
    if (Array.isArray(data)) return data;
    if (data instanceof Uint8Array) return data;
    if (typeof data === 'string') {
      try {
        const binary = window.atob(data);
        const bytes = new Uint8Array(binary.length);
        for (let i = 0; i < binary.length; i += 1) bytes[i] = binary.charCodeAt(i);
        return bytes;
      } catch (error) {
        return null;
      }
    }
    if (data.buffer instanceof ArrayBuffer) return new Uint8Array(data.buffer);
    return null;
  }

  function updateCameraStatusFromTopic() {
    if (!els.cameraStatus) return;
    if (!client.connected) {
      setCameraStatus(t('st.cameraOffline'), 'error');
      return;
    }
    const last = client.getLastMessageAt(REQUIRED_TOPICS.cameraImage);
    if (!last) {
      setCameraStatus(listedTopics.has(REQUIRED_TOPICS.cameraImage) ? t('st.cameraWaitFrame') : t('st.cameraWaitTopic'), 'warn');
      return;
    }
    const age = (Date.now() - last) / 1000;
    if (age > 3.0) {
      setCameraStatus(`${age.toFixed(1)}s`, 'warn');
    }
  }

  function setCameraStatus(text, state) {
    if (!els.cameraStatus) return;
    els.cameraStatus.textContent = text;
    els.cameraStatus.classList.toggle('online', state === 'online');
    els.cameraStatus.classList.toggle('warn', state === 'warn');
    els.cameraStatus.classList.toggle('error', state === 'error');
  }

  function handleVisionDetections(msg) {
    if (!els.visionStatus && !els.visionTarget) return;
    let payload = null;
    try {
      payload = JSON.parse(msg && msg.data ? msg.data : '{}');
    } catch (error) {
      if (els.visionStatus) els.visionStatus.textContent = t('st.cameraDataError');
      return;
    }

    latestVisionPayload = payload;
    latestVisionAt = performance.now();
    const count = Number(payload.count) || 0;
   if (els.visionStatus) {
      els.visionStatus.textContent = count ? t('fb.visionCount', {count, color: payload.target_color || '--'}) : t('st.visionNone');
   }
    if (!visionSequenceBusy) updateSelectedVisionTarget();
  }

  function handleMujocoObjectStates(msg) {
    let payload = null;
    try {
      payload = JSON.parse(msg && msg.data ? msg.data : '{}');
    } catch (error) {
      return;
    }
    announceMujocoSync(t('reason.mujocoObject'));
    if (window.reBotSim && typeof window.reBotSim.syncMujocoObjectStates === 'function') {
      window.reBotSim.syncMujocoObjectStates(payload.objects || []);
    }
  }

  function handleSimAnimationEvent(msg) {
    let payload = null;
    try {
      payload = JSON.parse(msg && msg.data ? msg.data : '{}');
    } catch (error) {
      writeLog(t('log.mcpAnimParseFail'), 'warn');
      return;
    }

    const event = String(payload.event || payload.action || '').toLowerCase();
    if (event === 'attach_object') {
      const target = payload.target && typeof payload.target === 'object'
        ? payload.target
        : { color: payload.color };
      attachSimCarriedObject(target);
    } else if (event === 'release_object') {
      releaseSimCarriedObject();
    }
  }

  function updateSelectedVisionTarget() {
    const mode = els.visionColor ? String(els.visionColor.value || 'auto') : 'auto';
    const target = mode === 'auto'
      ? (chooseVisionTarget(autoVisionTargetColor) || chooseRandomVisionTarget())
      : chooseVisionTarget(mode);
    selectedVisionTarget = target;
    renderVisionTarget(target);
  }

  function renderVisionTarget(target) {
    if (!els.visionTarget) return;
    if (!target) {
      els.visionTarget.textContent = '--';
      return;
    }
    const resolvedApproachZ = Number(target.resolved_move_above_z_m);
    const approachZ = Number.isFinite(resolvedApproachZ)
      ? resolvedApproachZ
      : getVisionApproachZ(target);
   const graspPlan = estimateVisionGraspPlan(target);
    els.visionTarget.textContent = t('fb.visionTarget', {color: target.color, x: Number(target.x).toFixed(3), y: Number(target.y).toFixed(3), z: approachZ.toFixed(3), mm: Math.round(graspPlan.physicalGap * 1000), yaw: Math.round(graspPlan.yawRad * 180 / Math.PI)});
 }

  function chooseRandomVisionTarget() {
    const detections = latestVisionPayload && Array.isArray(latestVisionPayload.detections)
      ? latestVisionPayload.detections
      : [];
    const colors = [...new Set(
      detections
        .filter((item) => item && item.color)
        .map((item) => String(item.color))
    )];
    if (!colors.length) return null;
    const alternatives = colors.filter((color) => color !== autoVisionTargetColor);
    const pool = alternatives.length ? alternatives : colors;
    autoVisionTargetColor = pool[Math.floor(Math.random() * pool.length)];
    return chooseVisionTarget(autoVisionTargetColor);
  }

  function chooseVisionTarget(preferredColor) {
    const detections = latestVisionPayload && Array.isArray(latestVisionPayload.detections)
      ? latestVisionPayload.detections
      : [];
    if (!detections.length) return null;
    const color = preferredColor || (els.visionColor ? String(els.visionColor.value || 'auto') : 'auto');
    if (color && color !== 'auto') {
      return chooseMujocoAssociatedDetection(
        color,
        detections.filter((item) => item && item.color === color)
      );
    }
    const payloadTarget = latestVisionPayload.target;
    if (payloadTarget && payloadTarget.color) {
      const associated = chooseMujocoAssociatedDetection(
        String(payloadTarget.color),
        detections.filter((item) => item && item.color === payloadTarget.color)
      );
      if (associated) return associated;
    }
    return detections[0] || null;
  }

  function chooseMujocoAssociatedDetection(color, candidates) {
    if (!Array.isArray(candidates) || !candidates.length) return null;
    if (!window.reBotSim || typeof window.reBotSim.getSceneCollisionMap !== 'function') {
      return candidates[0];
    }
    const map = window.reBotSim.getSceneCollisionMap();
    const expected = map && map.source === 'mujoco_object_states'
      && map.objects && map.objects[color] && map.objects[color].position;
    if (!expected) return candidates[0];

    const ranked = candidates
      .map((item) => ({
        item,
        distance: Math.hypot(Number(item.x) - expected.x, Number(item.y) - expected.y)
      }))
      .filter((entry) => Number.isFinite(entry.distance))
      .sort((a, b) => a.distance - b.distance);
    return ranked.length && ranked[0].distance <= 0.055 ? ranked[0].item : null;
  }

  async function waitForFreshVisionTarget(preferredColor, timeoutMs) {
    const start = performance.now();
    const initialVisionAt = latestVisionAt;
    let target = chooseVisionTarget(preferredColor);
    while (performance.now() - start < timeoutMs) {
      const fresh = latestVisionAt > initialVisionAt || performance.now() - latestVisionAt < 450;
      target = chooseVisionTarget(preferredColor) || target;
      if (target && fresh) return cloneVisionTarget(target);
      await sleep(80);
    }
    return target ? cloneVisionTarget(target) : null;
  }

  function fillPoseFromVisionTarget() {
    const target = selectedVisionTarget || chooseVisionTarget();
    const resolvedApproachZ = Number(target && target.resolved_move_above_z_m);
    const approachZ = Number.isFinite(resolvedApproachZ)
      ? resolvedApproachZ
      : getVisionApproachZ(target);
    const pose = poseFromVisionTarget(approachZ, target);
    if (!pose) return;
    writePoseInputs(pose);
    if (client.connected) client.publishTargetPose(pose);
    setMessage(t('msg.visionFillDone'));
    writeLog(t('log.visionFill'), 'ok');
  }

  async function moveAboveVisionTarget() {
    if (!controlAllowed(true)) return;
    const mode = els.visionColor ? String(els.visionColor.value || 'auto') : 'auto';
    let target = mode === 'auto'
      ? (selectedVisionTarget || chooseVisionTarget())
      : (selectedVisionTarget || chooseVisionTarget(mode));
    if (!target) {
      setMessage(t('msg.noVisionTarget'));
      return;
    }
    const duration = getPoseDuration();
    setVisionBusy(true, 'move');
    try {
      const requestedApproachZ = getVisionApproachZ(target);
      target = await resolveReachableVisionTarget(target, requestedApproachZ);
      selectedVisionTarget = target;
      renderVisionTarget(target);
      const resolvedApproachZ = Number(target.resolved_move_above_z_m);
      const approachZ = Number.isFinite(resolvedApproachZ) ? resolvedApproachZ : requestedApproachZ;
      const pose = poseFromVisionTarget(approachZ, target);
      if (!pose) return;
      const previousTarget = lastVisionTarget && !sameVisionTarget(lastVisionTarget, target)
        ? lastVisionTarget
        : null;
      const route = [];
      if (previousTarget) {
        const previousResolvedZ = Number(previousTarget.resolved_move_above_z_m);
        appendVisionRoutePose(
          route,
          poseFromVisionTarget(
            Number.isFinite(previousResolvedZ) ? previousResolvedZ : getVisionTransitZ(previousTarget),
            previousTarget
          ),
          t('msg.avoidLift', {color: previousTarget.color})
        );
        appendVisionRoutePose(route, pose, t('msg.moveAboveTarget', {color: target.color}));
      }
      for (const waypoint of route) {
        await runVisionMoveStep(waypoint.pose, Math.max(1.1, duration * 0.60), waypoint.label);
      }
      const routedToPose = route.length && poseDistance(route[route.length - 1].pose, pose) < VISION_POSE_SKIP_M;
      if (!routedToPose) {
        await runVisionMoveStep(pose, duration, t('msg.moveAboveTarget', {color: target.color}));
      }
      lastVisionTarget = cloneVisionTarget(target);
    } catch (error) {
      const message = error && error.message ? error.message : t('msg.visionMoveAbort');
      setMessage(message);
      writeLog(message, 'warn');
    } finally {
      setVisionBusy(false);
    }
  }

  async function resolveReachableVisionTarget(target, z) {
    if (!target || !client.connected) return target;
    const x = Number(target.x);
    const y = Number(target.y);
    const targetZ = Number(z);
    if (![x, y, targetZ].every(Number.isFinite)) return target;

    const baseYaw = estimateVisionGraspPlan(target).yawRad;
    const color = String(target.color || '');
    const width = Number(target.width_m);
    const height = Number(target.height_m);
    const rotationallySymmetric = color === 'red'
      || color === 'yellow'
      || (Number.isFinite(width) && Number.isFinite(height) && Math.abs(width - height) < 0.008);
    const rawYawCandidates = [baseYaw, baseYaw + Math.PI];
    if (rotationallySymmetric) rawYawCandidates.push(baseYaw + Math.PI / 2, baseYaw - Math.PI / 2);
    const yawCandidates = rawYawCandidates
      .map((yaw) => Math.atan2(Math.sin(yaw), Math.cos(yaw)))
      .filter((yaw, index, values) => values.findIndex((value) => Math.abs(value - yaw) < 1e-6) === index);
    const fallbackZ = Number(VISION_MOVE_ABOVE_FALLBACK_Z_BY_COLOR_M[color]);
    const zCandidates = [targetZ];
    if (Number.isFinite(fallbackZ) && Math.abs(fallbackZ - targetZ) > 1e-6) zCandidates.push(fallbackZ);
    let best = { yawRad: baseYaw, z: targetZ, errorM: Infinity };

    for (const candidateZ of zCandidates) {
      for (let index = 0; index < yawCandidates.length; index += 1) {
        const yawRad = yawCandidates[index];
        const pose = {
          position: { x, y, z: candidateZ },
          orientation: topDownOrientationWithYaw(yawRad)
        };
        try {
          const ik = await client.solveMoveToPoseIK(pose);
          const errorM = readIkPositionError(ik);
          if (errorM < best.errorM) best = { yawRad, z: candidateZ, errorM };
          if (ik && ik.success === true) {
            reportResolvedVisionPose(target, baseYaw, targetZ, yawRad, candidateZ);
            return {
              ...target,
              resolved_grasp_yaw_rad: yawRad,
              resolved_move_above_z_m: candidateZ
            };
          }
        } catch (error) {
          throw error;
        }
      }
    }
    const bestErrorMm = Number.isFinite(best.errorM) ? (best.errorM * 1000).toFixed(1) : '--';
    throw new Error(t('msg.visionNoExactAbove', {color: target.color, error: bestErrorMm}));
  }

  function readIkPositionError(ik) {
    const direct = Number(ik && ik.position_error);
    if (Number.isFinite(direct)) return direct;
    const match = String(ik && ik.message || '').match(/error=([0-9.]+)\s*mm/i);
    return match ? Number(match[1]) / 1000 : Infinity;
  }

  function reportResolvedVisionPose(target, baseYaw, requestedZ, resolvedYaw, resolvedZ) {
    const yawDelta = Math.abs(Math.atan2(Math.sin(resolvedYaw - baseYaw), Math.cos(resolvedYaw - baseYaw)));
    if (yawDelta > 1e-6) writeLog(t('log.visionYawFlip', {color: target.color}), 'info');
    if (Math.abs(resolvedZ - requestedZ) > 1e-6) {
      writeLog(t('log.visionHeightFallback', {color: target.color, z: resolvedZ.toFixed(3)}), 'warn');
    }
  }

  async function runVisionPickDemo() {
    if (visionSequenceBusy) return;
    if (!controlAllowed(true)) return;
    const preferredColor = els.visionColor ? String(els.visionColor.value || 'auto') : 'auto';
    let target = await waitForFreshVisionTarget(preferredColor, 700);
    if (preferredColor === 'auto') {
      target = chooseRandomVisionTarget() || target;
      if (target) {
        selectedVisionTarget = target;
       renderVisionTarget(target);
        writeLog(t('log.autoTarget', {color: target.color}), 'info');
     }
    }
    if (!target) {
      setMessage(t('msg.noVisionTarget'));
      return;
    }

    const previousTarget = lastVisionTarget && !sameVisionTarget(lastVisionTarget, target)
      ? lastVisionTarget
      : null;
    let plan = buildVisionPickPlan(target);
   if (!plan) return;
   writeLog(
      t('log.visionGraspPose', {color: target.color, mm: Math.round(plan.graspPlan.physicalGap * 1000), yaw: Math.round(plan.graspPlan.yawRad * 180 / Math.PI), lift: plan.firstLiftPose.position.z.toFixed(3), transit: plan.transitPose.position.z.toFixed(3)}),
     'info'
    );

    const duration = getPoseDuration();
    let lastPose = null;
    const runIfNeeded = async (pose, moveDuration, label) => {
     if (lastPose && poseDistance(lastPose, pose) < VISION_POSE_SKIP_M) {
        writeLog(t('log.visionSkipMove', {label}), 'info');
       return { success: true, skipped: true };
      }
      const result = await runVisionMoveStep(pose, moveDuration, label);
      lastPose = pose;
      return result;
    };

    setVisionBusy(true, 'pick');
    try {
      releaseSimCarriedObject();
      await commandGripperAndWait(OPEN_GRIPPER_M, t('msg.pickOpenGripper'), {
        timeoutMs: 2600,
        minWaitMs: 850,
        tolerance: 0.006,
        requireReached: true,
        afterMs: 180
      });

      const route = buildVisionTransitRoute(previousTarget, target);
      for (const waypoint of route) {
        await runIfNeeded(waypoint.pose, Math.max(1.2, duration * 0.65), waypoint.label);
      }

      const refinedTarget = await waitForFreshVisionTarget(target.color, 420);
      if (refinedTarget && visionTargetShifted(refinedTarget, target, 0.008)) {
        target = refinedTarget;
        plan = buildVisionPickPlan(target);
       if (!plan) return;
        writeLog(t('log.visionRelocate', {color: target.color}), 'info');
       writeLog(
          t('log.visionGraspPose', {color: target.color, mm: Math.round(plan.graspPlan.physicalGap * 1000), yaw: Math.round(plan.graspPlan.yawRad * 180 / Math.PI), lift: plan.firstLiftPose.position.z.toFixed(3), transit: plan.transitPose.position.z.toFixed(3)}),
         'info'
        );
      }

     writePoseInputs(plan.approachPose);
      await runIfNeeded(plan.approachPose, duration, t('msg.pickMoveAbove', {color: target.color}));

     const alignDuration = Math.max(1.0, duration * 0.55);
      await runIfNeeded(plan.verticalAlignPose, alignDuration, t('msg.pickAlign', {color: target.color}));

     const pregraspDuration = Math.max(0.85, duration * 0.45);
      await runIfNeeded(plan.pregraspPose, pregraspDuration, t('msg.pickPreDescend', {color: target.color}));

     const descendDuration = Math.max(1.1, duration * 0.65);
      await runIfNeeded(plan.graspPose, descendDuration, t('msg.pickDescend', {color: target.color}));

      await commandGripperAndWait(plan.graspPlan.command, t('msg.pickSqueeze', {color: target.color}), {
       timeoutMs: 2100,
        minWaitMs: 850,
        tolerance: 0.006,
        allowContactStop: true,
        afterMs: 220
      });
      attachSimCarriedObject(target);

      const firstLiftDuration = Math.max(1.25, duration * 0.75);
     if (String(target.color || '') === 'blue') {
        await runVisionMoveStep(plan.firstLiftPose, firstLiftDuration, t('msg.pickBlueLift', {z: plan.firstLiftPose.position.z.toFixed(3)}));
       lastPose = plan.firstLiftPose;
        await runVisionMoveStep(plan.transitPose, Math.max(1.45, duration * 0.70), t('msg.pickBlueTransit', {z: plan.transitPose.position.z.toFixed(3)}));
       lastPose = plan.transitPose;
     } else {
        await runIfNeeded(plan.firstLiftPose, firstLiftDuration, t('msg.pickLift', {color: target.color}));
      }

     const liftDuration = Math.max(1.8, duration * 0.85);
      await runIfNeeded(plan.approachPose, liftDuration, t('msg.pickRaise', {color: target.color}));

     const finalTransitDuration = Math.max(1.1, duration * 0.60);
      await runIfNeeded(plan.transitPose, finalTransitDuration, t('msg.pickTransit', {color: target.color}));

     lastVisionTarget = cloneVisionTarget(target);
      setMessage(t('msg.graspDemoDone', {mm: Math.round(plan.graspPlan.physicalGap * 1000)}));
      writeLog(t('log.graspDone', {mm: Math.round(plan.graspPlan.command * 1000)}), 'ok');
   } catch (error) {
      const message = error && error.message ? error.message : t('msg.visionPickAbort');
      setMessage(message);
      writeLog(message, 'warn');
    } finally {
      setVisionBusy(false);
    }
  }

  async function runVisionPlaceDemo() {
    if (visionSequenceBusy) return;
    if (!controlAllowed(true)) return;

    const simCarriedColor = window.reBotSim && typeof window.reBotSim.getCarriedObject === 'function'
      ? window.reBotSim.getCarriedObject()
      : '';
    const target = heldVisionTarget
      || (simCarriedColor && lastVisionTarget && String(lastVisionTarget.color) === String(simCarriedColor)
        ? lastVisionTarget
        : null);
    if (!target) {
      setMessage(t('msg.noHeldObject'));
      writeLog(t('log.placeIgnored'), 'warn');
      return;
    }

    const plan = buildVisionPickPlan(target);
    if (!plan) return;
    const duration = getPoseDuration();
    setVisionBusy(true, 'place');
    try {
      await runVisionMoveStep(
       plan.approachPose,
       Math.max(1.1, duration * 0.65),
        t('msg.placeMoveAbove', {color: target.color})
     );
      await runVisionMoveStep(
       plan.graspPose,
       Math.max(1.1, duration * 0.65),
        t('msg.placeDescend', {color: target.color})
     );
      await commandGripperAndWait(OPEN_GRIPPER_M, t('msg.placeOpen', {color: target.color}), {
       timeoutMs: 2600,
        minWaitMs: 850,
        tolerance: 0.006,
        requireReached: true,
        afterMs: 220
      });
      releaseSimCarriedObject();
      await runVisionMoveStep(
       plan.approachPose,
       Math.max(1.5, duration * 0.8),
        t('msg.placeLift', {color: target.color})
     );
     lastVisionTarget = cloneVisionTarget(target);
      setMessage(t('msg.placeDone', {color: target.color}));
      writeLog(t('log.placeDoneLog', {color: target.color}), 'ok');
   } catch (error) {
      const message = error && error.message ? error.message : t('msg.visionPlaceAbort');
      setMessage(message);
      writeLog(message, 'warn');
    } finally {
      setVisionBusy(false);
    }
  }

  async function runVisionMoveStep(pose, duration, label) {
    writePoseInputs(pose);
    client.publishTargetPose(pose);
    const result = await sendVisionMoveGoal(pose, duration, label);
   if (!movementSucceeded(result)) {
      throw new Error(t('msg.stepFailed', {label}));
   }
    if (!(result && result.localPlayback)) {
      await sleep(duration * 1000 + 300);
    }
    return result;
  }

  function movementSucceeded(result) {
    if (!result) return false;
    if (result.success === false || result.accepted === false) return false;
    return true;
  }

  function buildVisionPickPlan(target) {
    const approachZ = getVisionApproachZ(target);
    const graspZ = getVisionGraspZ(target);
    const transitPose = poseFromVisionTarget(getVisionTransitZ(target), target);
    const approachPose = poseFromVisionTarget(approachZ, target);
    const verticalAlignZ = Math.min(
      approachZ,
      Math.max(graspZ + VISION_VERTICAL_ALIGN_CLEARANCE_M, VISION_MIN_VERTICAL_ALIGN_Z_M)
    );
    const pregraspZ = Math.min(
      verticalAlignZ,
      Math.max(graspZ + VISION_PREGRASP_CLEARANCE_M, graspZ)
    );
    const verticalAlignPose = poseFromVisionTarget(verticalAlignZ, target);
    const pregraspPose = poseFromVisionTarget(pregraspZ, target);
    const graspPose = poseFromVisionTarget(graspZ, target);
    const firstLiftZ = Math.min(
      approachZ,
      Math.max(graspZ + VISION_FIRST_LIFT_CLEARANCE_M, getVisionFirstLiftMinZ(target))
    );
    const firstLiftPose = poseFromVisionTarget(firstLiftZ, target);
    if (!transitPose || !approachPose || !verticalAlignPose || !pregraspPose || !graspPose || !firstLiftPose) return null;
    return {
      target,
      approachZ,
      graspZ,
      transitPose,
      approachPose,
      verticalAlignPose,
      pregraspPose,
      graspPose,
      firstLiftPose,
      graspPlan: estimateVisionGraspPlan(target)
    };
  }

  function buildVisionTransitRoute(previousTarget, target) {
    const route = [];
    if (previousTarget) {
      appendVisionRoutePose(
        route,
       poseFromVisionTarget(getVisionTransitZ(previousTarget), previousTarget),
        t('msg.avoidLift', {color: previousTarget.color})
     );
    }
    appendVisionRoutePose(
      route,
     poseFromVisionTarget(getVisionTransitZ(target), target),
      t('msg.avoidMove', {color: target.color})
   );
    return route;
  }

  function appendVisionRoutePose(route, pose, label) {
    if (!pose) return;
    const last = route.length ? route[route.length - 1].pose : null;
    if (last && poseDistance(last, pose) < 0.025) return;
    route.push({ pose, label });
  }

  function poseDistance(left, right) {
    const values = [
      left && left.position && left.position.x,
      left && left.position && left.position.y,
      left && left.position && left.position.z,
      right && right.position && right.position.x,
      right && right.position && right.position.y,
      right && right.position && right.position.z
    ].map(Number);
    if (!values.every(Number.isFinite)) return Infinity;
    return Math.hypot(values[0] - values[3], values[1] - values[4], values[2] - values[5]);
  }

  function poseFromVisionTarget(zOverride, targetOverride) {
    const target = targetOverride || selectedVisionTarget || chooseVisionTarget();
    if (!target) {
      setMessage(t('msg.noVisionTarget'));
      return null;
    }
    const x = Number(target.x);
    const y = Number(target.y);
    const z = Number(zOverride);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      setMessage(t('msg.visionCoordError'));
      return null;
    }
    selectedVisionTarget = target;
    const graspPlan = estimateVisionGraspPlan(target);
    return {
      position: { x, y, z },
      orientation: topDownOrientationWithYaw(graspPlan.yawRad)
    };
  }

  async function sendVisionMoveGoal(pose, duration, optimisticMessage) {
    if (shouldUseLowLevelTrajectory() || !hasActionServer(`/${NS}/move_to_pose`)) {
      return moveToPoseViaIkTrajectory(pose, duration, optimisticMessage);
    }

    try {
      setMessage(optimisticMessage);
      writeLog(optimisticMessage, 'info');
      const result = await client.moveToPose(pose, duration);
      const message = formatServiceResult(result);
      setMessage(message);
      writeLog(message, result && result.accepted === false ? 'warn' : 'ok');
      return { ...(result || {}), localPlayback: false };
    } catch (error) {
      const message = error && error.message ? error.message : t('msg.visionMoveFail');
      setStatus('error', message);
      writeLog(message, 'error');
      return { success: false, localPlayback: false };
    }
  }

 async function moveToPoseViaIkTrajectory(pose, duration, optimisticMessage) {
    setMessage(t('msg.ikSolving', {label: optimisticMessage}));
    writeLog(t('msg.ikSolving', {label: optimisticMessage}), 'info');
   const ik = await guardedCall(
     () => client.solveMoveToPoseIK(pose),
      t('msg.ikSolving', {label: optimisticMessage}),
     true,
      { keepConnectionStatus: true }
    );
    if (!ik || !Array.isArray(ik.q_solution) || !ik.q_solution.length) {
      setMessage(t('msg.ikNoSolution'));
      return { success: false, localPlayback: true };
    }
    const ikBestEffort = ik.success === false;
   if (ikBestEffort) {
     const message = ik.message || t('msg.ikApproxFallback');
      setMessage(t('msg.ikApprox', {message}));
      writeLog(t('msg.ikApprox', {message}), 'warn');
   }

    const start = getCurrentRosPositions();
    const goal = JOINT_NAMES.map((name, index) => {
      const value = Number(ik.q_solution[index]);
      return Number.isFinite(value) ? value : start[index];
    });
   const points = buildSmoothJointMovePoints(start, goal, duration);
    await sendTrajectory(points, t('msg.ikLowLevelSuffix', {label: optimisticMessage}));
   return { success: true, localPlayback: true, bestEffort: ikBestEffort };
  }

  function buildSmoothJointMovePoints(start, goal, duration) {
    const seconds = clamp(Number(duration) || 2, 0.4, 8);
    const count = Math.max(10, Math.ceil(seconds * 30));
    const points = [makeTrajectoryPoint(start, 0.05)];
    for (let index = 1; index <= count; index += 1) {
      const ratio = index / count;
      const eased = ratio * ratio * (3 - 2 * ratio);
      const positions = goal.map((value, jointIndex) => {
        const from = Number(start[jointIndex]) || 0;
        return from + (value - from) * eased;
      });
      points.push(makeTrajectoryPoint(positions, Math.max(0.06, seconds * ratio)));
    }
    return points;
  }

  function writePoseInputs(pose) {
    if (els.poseX) els.poseX.value = Number(pose.position.x).toFixed(3);
    if (els.poseY) els.poseY.value = Number(pose.position.y).toFixed(3);
    if (els.poseZ) els.poseZ.value = Number(pose.position.z).toFixed(3);
  }

  function getVisionApproachZ(target) {
    const transitZ = getVisionTransitZ(target);
    const detected = target && Number.isFinite(Number(target.z)) ? Number(target.z) : transitZ;
    const requested = Number(els.visionApproachZ && els.visionApproachZ.value);
    const value = Number.isFinite(requested) ? requested : detected;
    return clamp(Math.max(value, transitZ), 0.08, 0.42);
  }

  function getVisionTransitZ(target) {
    const color = target && target.color ? String(target.color) : '';
    const value = VISION_TRANSIT_Z_BY_COLOR_M[color];
    return Number.isFinite(value) ? value : VISION_TRANSIT_Z_M;
  }

  function getVisionGraspZ(target) {
    const safe = estimateVisionGraspZ(target);
    const requested = Number(els.visionGraspZ && els.visionGraspZ.value);
    const value = Number.isFinite(requested) ? Math.max(requested, safe) : safe;
    return clamp(value, 0.06, 0.25);
  }

  function estimateVisionGraspZ(target) {
    const color = target && target.color ? String(target.color) : '';
    const safeByColor = {
      red: 0.137,
      blue: 0.132,
      yellow: 0.164
    };
    if (Number.isFinite(safeByColor[color])) return safeByColor[color];
    const fallback = target && Number.isFinite(Number(target.z)) ? Math.max(Number(target.z) - 0.035, 0.13) : 0.14;
    return fallback;
  }

  function getVisionFirstLiftMinZ(target) {
    const color = target && target.color ? String(target.color) : '';
    const value = VISION_FIRST_LIFT_MIN_BY_COLOR_M[color];
    return Number.isFinite(value) ? value : VISION_FIRST_LIFT_MIN_M;
  }

  function sameVisionTarget(left, right) {
    if (!left || !right) return false;
    if (String(left.color || '') !== String(right.color || '')) return false;
    const dx = Number(left.x) - Number(right.x);
    const dy = Number(left.y) - Number(right.y);
    return Number.isFinite(dx) && Number.isFinite(dy) && Math.hypot(dx, dy) < 0.035;
  }

  function visionTargetShifted(left, right, threshold) {
    if (!left || !right) return false;
    if (String(left.color || '') !== String(right.color || '')) return false;
    const dx = Number(left.x) - Number(right.x);
    const dy = Number(left.y) - Number(right.y);
    const dz = Number(left.z || 0) - Number(right.z || 0);
    if (![dx, dy, dz].every(Number.isFinite)) return false;
    return Math.hypot(dx, dy, dz) > threshold;
  }

  function cloneVisionTarget(target) {
    if (!target || typeof target !== 'object') return null;
    return { ...target };
  }

  function attachSimCarriedObject(target) {
    const color = target && target.color ? String(target.color) : '';
    if (!color) return;
    heldVisionTarget = cloneVisionTarget(target);
    if (!window.reBotSim || typeof window.reBotSim.attachObject !== 'function') return;
   if (window.reBotSim.attachObject(color)) {
      writeLog(t('log.simAttach', {color}), 'ok');
   }
  }

  function releaseSimCarriedObject() {
    heldVisionTarget = null;
    if (!window.reBotSim || typeof window.reBotSim.releaseObject !== 'function') return;
    if (window.reBotSim.releaseObject({ settleOnTable: true })) {
      writeLog(t('log.simRelease'), 'info');
    }
  }

  function estimateVisionGraspPlan(target) {
    const fallbackByColor = {
      red: 0.05,
      yellow: 0.044,
      blue: 0.044
    };
    const width = Number(target && target.width_m);
    const height = Number(target && target.height_m);
    let crossSection = Number(target && target.shortest_m);
    let yawRad = 0;

    const color = target && target.color ? String(target.color) : '';
    if (Number.isFinite(width) && Number.isFinite(height) && width > 0 && height > 0) {
      const candidates = [
        { crossSection: height, yawRad: 0 },
        { crossSection: width, yawRad: -Math.PI / 2 }
      ].sort((left, right) => {
        const leftFits = left.crossSection <= GRIPPER_EFFECTIVE_GAP_M;
        const rightFits = right.crossSection <= GRIPPER_EFFECTIVE_GAP_M;
        if (leftFits !== rightFits) return leftFits ? -1 : 1;
        return left.crossSection - right.crossSection;
      });
      crossSection = candidates[0].crossSection;
      yawRad = candidates[0].yawRad;
    } else {
      if (!Number.isFinite(crossSection) || crossSection <= 0) {
        crossSection = fallbackByColor[color] || 0.05;
      }
      const reportedYaw = Number(target && target.grasp_yaw_rad);
      if (Number.isFinite(reportedYaw)) {
        yawRad = reportedYaw;
      }
    }

    const nominal = fallbackByColor[color];
    if (Number.isFinite(nominal)) {
      crossSection = Math.max(crossSection, nominal);
    }

    const resolvedYaw = Number(target && target.resolved_grasp_yaw_rad);
    if (Number.isFinite(resolvedYaw)) yawRad = resolvedYaw;

    const physicalGap = clamp(
      crossSection - GRASP_SQUEEZE_M,
      MIN_OBJECT_GRASP_M,
      GRIPPER_EFFECTIVE_GAP_M
    );
    return {
      command: physicalGapToGripperCommand(physicalGap),
      physicalGap,
      yawRad
    };
  }

  function physicalGapToGripperCommand(physicalGap) {
    const travel = Math.max(GRIPPER_VISUAL_TRAVEL_M, 0.001);
    return clamp(((physicalGap - GRIPPER_BASE_GAP_M) / travel) * OPEN_GRIPPER_M, CLOSE_GRIPPER_M, OPEN_GRIPPER_M);
  }

  function topDownOrientationWithYaw(yawRad) {
    const yaw = Number.isFinite(yawRad) ? yawRad : 0;
    const sy = Math.sin(yaw / 2);
    const cy = Math.cos(yaw / 2);
    const s90 = Math.SQRT1_2;
    return {
      x: -sy * s90,
      y: cy * s90,
      z: sy * s90,
      w: cy * s90
    };
  }

  function getPoseDuration() {
    return clamp(Number(els.poseDuration && els.poseDuration.value) || 2, 0.4, 8);
  }

  function setVisionBusy(busy, operation) {
    visionSequenceBusy = busy;
    lastVisionOp = busy ? operation : null;
    if (els.visionPickDemo) {
      els.visionPickDemo.disabled = busy;
      els.visionPickDemo.textContent = busy && operation === 'pick' ? t('btn.pickBusy') : t('camera.pick');
    }
    if (els.visionPlaceDemo) {
      els.visionPlaceDemo.disabled = busy;
      els.visionPlaceDemo.textContent = busy && operation === 'place' ? t('btn.placeBusy') : t('camera.place');
    }
    if (els.visionMoveAbove) els.visionMoveAbove.disabled = busy;
    if (els.visionFillPose) els.visionFillPose.disabled = busy;
  }

  function updateFeedbackError(feedback) {
    if (!els.feedbackError || !window.reBotSim || !feedback || !Object.keys(feedback).length) return;
    const simAngles = typeof window.reBotSim.getAngles === 'function' ? window.reBotSim.getAngles() : {};
    let maxError = 0;
    let sumSq = 0;
    let count = 0;
    let worstJoint = '';

    Object.entries(feedback).forEach(([name, value]) => {
      const target = simTargetAngles.has(name) ? simTargetAngles.get(name) : simAngles[name];
      if (typeof target !== 'number') return;
      const error = Math.abs(target - value);
      if (error > maxError) {
        maxError = error;
        worstJoint = name;
      }
      sumSq += error * error;
      count += 1;
    });

    if (!count) return;
   const rms = Math.sqrt(sumSq / count);
    els.feedbackError.textContent = t('fb.errorMax', {max: (maxError * 180 / Math.PI).toFixed(2), joint: worstJoint || '', rms: (rms * 180 / Math.PI).toFixed(2)});
   els.feedbackError.style.color = maxError < 0.035 ? '#d7fff4' : (maxError < 0.12 ? '#ffe0b0' : '#ffd1c9');
  }

  function updateGravityStatus(active, detail, source) {
    const nextActive = Boolean(active);
    const nextSource = source || 'system';
    // ArmStatus arrives faster than the detailed status service. Once the
    // service has supplied its lock-target text, do not let the short machine
    // state overwrite it on every ArmStatus message and make the UI flicker.
    if (
      nextSource === 'arm' &&
      gravityStatusSource === 'service' &&
      nextActive === gravityCompensationActive
    ) return;

    gravityCompensationActive = nextActive;
    gravityStatusSource = nextSource;
    if (!els.gravityStatus) return;
    els.gravityStatus.textContent = nextActive ? t('st.running') : t('st.notRunning');
    if (detail && detail !== 'GRAVITY_COMP') {
      els.gravityStatus.textContent += ` / ${detail}`;
    }
    els.gravityStatus.style.color = nextActive ? '#d7fff4' : '#ffe0b0';
  }

  function maybeSendGripper(position) {
    syncSimGripper(position);
    if (!client.connected) {
      setMessage(t('msg.gripperSimOnly'));
      return;
    }
    if (!controlAllowed(false)) {
      setMessage(t('msg.controlLockClosed'));
      return;
    }
    publishGripper(position);
  }

  function sendGripper(position, options) {
    syncSimGripper(position);
    if (
      options &&
      options.requireControl &&
      !controlAllowed(true, { skipConfirm: true })
    ) return;
    if (!client.connected) {
      setStatus('closed', t('msg.rosNotConnected'));
      return;
    }
    publishGripper(position);
  }

  async function commandGripperAndWait(position, label, options) {
    const settings = {
      timeoutMs: 1800,
      minWaitMs: 500,
      tolerance: 0.006,
      settleMs: 260,
      afterMs: 0,
      allowContactStop: false,
      requireReached: false,
      ...(options || {})
    };
    publishGripper(position);
    setMessage(label);

    const start = performance.now();
    const initialFeedbackAt = latestGripperAt;
    const initialJointFeedback = gripperJointFeedback();
    let lastPosition = readGripperFeedbackPosition(position);
    let lastRepublishAt = start;
    let stableSince = start;
    let sawFreshFeedback = false;
    let reached = false;
    let current = lastPosition;
    let source = latestGripperAt > 0 ? 'gripper/state' : '';

    while (performance.now() - start < settings.timeoutMs) {
      await sleep(80);
      const now = performance.now();
      if (now - lastRepublishAt > 520) {
        client.publishGripperCommand(position);
        lastRepublishAt = now;
      }

      const jointFeedback = gripperJointFeedback();
      const hasFreshGripperState = latestGripperAt > initialFeedbackAt && now - latestGripperAt < 700;
      const hasFreshJointState = jointFeedback.fresh && jointFeedback.stamp !== initialJointFeedback.stamp;
      const hasFreshFeedback = hasFreshJointState || hasFreshGripperState;
      if (!hasFreshFeedback) {
        if (!settings.requireReached && now - start > Math.max(settings.minWaitMs, 900)) break;
        continue;
      }

      sawFreshFeedback = true;
      current = hasFreshJointState ? jointFeedback.widthCommand : Number(latestGripperPosition);
      source = hasFreshJointState ? 'joint_states/finger_left' : 'gripper/state';
      const velocity = Number(latestGripperVelocity);
      const closeEnough = Number.isFinite(current) && Math.abs(current - position) <= settings.tolerance;
      const barelyMoving = Number.isFinite(velocity)
        ? Math.abs(velocity) < 0.0025
        : Number.isFinite(current) && Number.isFinite(lastPosition) && Math.abs(current - lastPosition) < 0.0015;
      reached = reached || closeEnough;

      if (barelyMoving) {
        if (now - stableSince >= settings.settleMs && now - start >= settings.minWaitMs) {
          if (closeEnough || (!settings.requireReached && settings.allowContactStop)) break;
        }
      } else {
        stableSince = now;
      }

      if (closeEnough && now - start >= settings.minWaitMs) break;
      lastPosition = current;
    }

    if (settings.afterMs > 0) await sleep(settings.afterMs);
   if (settings.requireReached && !reached) {
      const message = t('msg.gripperNotReached', {label});
     setMessage(message);
      writeLog(message, 'warn');
      throw new Error(message);
    }
   const feedback = sawFreshFeedback && Number.isFinite(Number(current))
      ? t('fb.gripperSrcFb', {src: source, mm: Math.round(current * 1000)})
     : '';
    writeLog(t('log.gripperDone', {label, fb: feedback}), 'ok');
 }

  function readGripperFeedbackPosition(commandPosition) {
    const jointFeedback = gripperJointFeedback();
    if (jointFeedback.fresh && Number.isFinite(jointFeedback.widthCommand)) {
      return jointFeedback.widthCommand;
    }
    if (Number.isFinite(Number(latestGripperPosition))) {
      return Number(latestGripperPosition);
    }
    return Number(commandPosition);
  }

  function gripperJointFeedback() {
    const source = latestJointPositions || {};
    const left = Number(source.finger_left);
    if (!Number.isFinite(left)) {
      return { fresh: false, widthCommand: NaN, stamp: 0 };
    }
    return {
      fresh: true,
      widthCommand: fingerOpeningToGripperCommand(left),
      stamp: latestJointStateAt || 0
    };
  }

  function fingerOpeningToGripperCommand(opening) {
    return clamp((Number(opening) / 0.0285) * OPEN_GRIPPER_M, CLOSE_GRIPPER_M, OPEN_GRIPPER_M);
  }

  function snapClosedGripperForDisplay(position) {
    const value = clamp(Number(position), CLOSE_GRIPPER_M, OPEN_GRIPPER_M);
    return value <= GRIPPER_CLOSED_DISPLAY_SNAP_M ? CLOSE_GRIPPER_M : value;
  }

  function publishGripper(position) {
    syncSimGripper(position);
    client.publishGripperCommand(position);
    simTargetAngles.set('gripper', position);
    if (!hasFreshPhysicsJointFeedback()) {
      mirrorHoldUntil.set('gripper', performance.now() + 1200);
    }
    const feedback = typeof latestGripperPosition === 'number' ? t('fb.gripperFb', {mm: Math.round(latestGripperPosition * 1000)}) : '';
    setMessage(t('msg.gripperCmdPublished', {mm: Math.round(position * 1000), fb: feedback}));
    writeLog(t('log.gripperCmd', {mm: Math.round(position * 1000), topic: '/' + NS + '/gripper/cmd'}), 'ok');
   window.setTimeout(() => {
      if (client.connected) client.publishGripperCommand(position);
    }, 120);
  }

  function syncSimGripper(position) {
    if (!window.reBotSim || typeof window.reBotSim.setGripperWidth !== 'function') return;
    if (hasFreshPhysicsJointFeedback()) return;
    window.reBotSim.setGripperWidth(position, { source: 'ui', animate: true });
  }

  function getVlim() {
    return clamp(Number(els.vlim.value) || 1.2, 0.05, 3);
  }

  function getTrajectoryDuration() {
    return clamp(Number(els.trajectoryDuration.value) || 6, 1, 30);
  }

  function secondsToRosTime(seconds) {
    const sec = Math.floor(seconds);
    return { sec, nanosec: Math.round((seconds - sec) * 1e9) };
  }

  function rosTimeToSeconds(time) {
    return Number(time && time.sec ? time.sec : 0) + Number(time && time.nanosec ? time.nanosec : 0) * 1e-9;
  }

  function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
  }

  function sleep(ms) {
    return new Promise((resolve) => window.setTimeout(resolve, ms));
  }

  function waitForSimApi(callback) {
    if (window.reBotSim && typeof window.reBotSim.onCommand === 'function') {
      callback(window.reBotSim);
      return;
    }
    window.setTimeout(() => waitForSimApi(callback), 50);
  }

  function setStatus(state, message) {
    lastStatusState = state;
    els.status.className = 'mini-pill';
    if (state === 'open') {
      els.status.classList.add('online');
      els.status.textContent = t('st.online');
    } else if (state === 'connecting') {
      els.status.classList.add('warn');
      els.status.textContent = t('st.connecting');
    } else if (state === 'error') {
      els.status.classList.add('error');
      els.status.textContent = t('st.error');
    } else {
      els.status.textContent = t('st.offline');
    }
    setMessage(message);
  }

  function setMessage(message) {
    if (!els.message) return;
    const next = message || '';
    if (els.message.textContent !== next) els.message.textContent = next;
  }

  function writeLog(message, level) {
    if (!els.log || !message) return;
    const line = document.createElement('div');
    line.className = `ros-log-line ${level || 'info'}`;
    const now = new Date();
    line.innerHTML = `<time>${now.toLocaleTimeString()}</time><span></span>`;
    line.querySelector('span').textContent = String(message);
    els.log.prepend(line);
    while (els.log.children.length > 80) els.log.lastElementChild.remove();
  }

  if (window.rebotI18n) {
    window.rebotI18n.onLangChange(() => {
      // Re-render ROS connection status pill
      if (lastStatusState !== null && els.status) {
        if (lastStatusState === 'open') {
          els.status.textContent = t('st.online');
        } else if (lastStatusState === 'connecting') {
          els.status.textContent = t('st.connecting');
        } else if (lastStatusState === 'error') {
          els.status.textContent = t('st.error');
        } else {
          els.status.textContent = t('st.offline');
        }
      }
      // Re-render gravity compensation status
      if (els.gravityStatus) {
        els.gravityStatus.textContent = gravityCompensationActive ? t('st.running') : t('st.notRunning');
        els.gravityStatus.style.color = gravityCompensationActive ? '#d7fff4' : '#ffe0b0';
      }
      // Re-render vision pick/place demo buttons
      if (els.visionPickDemo) {
        els.visionPickDemo.textContent = visionSequenceBusy && lastVisionOp === 'pick' ? t('btn.pickBusy') : t('camera.pick');
      }
      if (els.visionPlaceDemo) {
        els.visionPlaceDemo.textContent = visionSequenceBusy && lastVisionOp === 'place' ? t('btn.placeBusy') : t('camera.place');
      }
    });
  }
})();
