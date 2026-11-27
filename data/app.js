"use strict";

(() => {
  const byId = (id) => document.getElementById(id);
  const fields = ["x", "y", "z", "phi"];
  const form = byId("target-form");
  const socketUrl = (() => {
    const configuredHost = new URLSearchParams(location.search).get("host");
    const host = configuredHost || location.hostname || "192.168.4.1";
    return `${location.protocol === "https:" ? "wss:" : "ws:"}//${host}:81/`;
  })();

  let socket = null;
  let reconnectTimer = null;
  let heartbeatTimer = null;
  let statusTimer = null;
  let lastPongAt = 0;
  let currentState = "UNKNOWN";
  let profile = "empty";
  let lastSolution = null;
  let limits = null;
  let trajectoryActive = false;

  function showError(message) {
    const banner = byId("error-banner");
    banner.textContent = message;
    banner.hidden = false;
  }

  function clearError() {
    byId("error-banner").hidden = true;
    byId("error-banner").textContent = "";
  }

  function send(message) {
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      showError("The arm is not connected. Reconnect before sending a command.");
      return false;
    }
    socket.send(JSON.stringify(message));
    return true;
  }

  function setConnection(connected, label) {
    byId("connection-dot").className = `status-dot ${connected ? "online" : "offline"}`;
    byId("connection-label").textContent = label;
    updateControls();
  }

  function updateState(state) {
    currentState = state || currentState;
    byId("state-label").textContent = currentState.replaceAll("_", " ");
    updateControls();
  }

  function updateControls() {
    const connected = Boolean(socket && socket.readyState === WebSocket.OPEN);
    const moveState = currentState === "IDLE" || currentState === "ERROR";
    const posture = byId("posture-select").value;
    const selected = lastSolution && lastSolution.solutions
      ? lastSolution.solutions[posture]
      : null;
    byId("preview-button").disabled = !connected;
    byId("move-button").disabled = !connected || !moveState ||
      !selected || !selected.valid || trajectoryActive;
    byId("posture-select").disabled = !lastSolution || !lastSolution.solutions;
    byId("payload-select").disabled = !connected || currentState !== "IDLE";
    byId("stop-button").disabled = !connected;
    byId("clear-estop-button").disabled = !connected;
  }

  function readTarget() {
    const target = {};
    for (const field of fields) {
      const input = form.elements[field];
      const value = input.valueAsNumber;
      if (!Number.isFinite(value)) {
        input.focus();
        throw new Error(`Enter a valid ${field.toUpperCase()} target.`);
      }
      target[field] = value;
    }
    return target;
  }

  function targetMatches(solutionInput, target) {
    return solutionInput && fields.every((field) =>
      Number.isFinite(Number(solutionInput[field])) &&
      Math.abs(Number(solutionInput[field]) - target[field]) < 0.001
    );
  }

  function invalidatePreview() {
    lastSolution = null;
    window.ArmVisualizer.setSolution(null);
    byId("preview-label").textContent = "Target changed — preview again";
    byId("target-distance").textContent = "—";
    byId("math-steps").replaceChildren();
    const empty = document.createElement("li");
    empty.className = "empty-state";
    empty.textContent = "Preview the updated target to compare valid postures.";
    byId("math-steps").append(empty);
    byId("math-count").textContent = "Preview required";
    updateControls();
  }

  function preview() {
    try {
      clearError();
      const target = readTarget();
      invalidatePreview();
      send({ type: "ik_preview", ...target });
    } catch (error) {
      showError(error.message);
    }
  }

  function move(event) {
    event.preventDefault();
    try {
      clearError();
      const target = readTarget();
      const posture = byId("posture-select").value;
      const candidate = lastSolution && lastSolution.solutions
        ? lastSolution.solutions[posture]
        : null;
      if (!candidate || !candidate.valid) {
        showError("Preview the target and select a valid posture before moving.");
        return;
      }
      if (currentState !== "IDLE" && currentState !== "ERROR") {
        showError("The arm must be idle or recovering from a target error.");
        return;
      }
      if (!targetMatches(lastSolution && lastSolution.input, target)) {
        invalidatePreview();
        showError("The target changed after its last preview. Preview it again before moving.");
        return;
      }
      if (send({
        type: "move_command",
        ...target,
        posture,
        gripper: byId("gripper-toggle").value
      })) {
        trajectoryActive = true;
        byId("progress-label").textContent = "Motion queued";
        updateControls();
      }
    } catch (error) {
      showError(error.message);
    }
  }

  function renderSteps(steps) {
    const list = byId("math-steps");
    list.replaceChildren();
    if (!Array.isArray(steps) || steps.length === 0) {
      const empty = document.createElement("li");
      empty.className = "empty-state";
      empty.textContent = "No derivation steps were returned.";
      list.append(empty);
      byId("math-count").textContent = "No steps";
      return;
    }

    byId("math-count").textContent = `${steps.length} steps`;
    for (const step of steps) {
      const item = document.createElement("li");
      const top = document.createElement("div");
      top.className = "step-topline";
      const label = document.createElement("span");
      label.className = "step-label";
      label.textContent = String(step.label || "step").replaceAll("_", " ");
      const value = document.createElement("span");
      value.className = "step-value";
      const numericValue = Number(step.value);
      const isAngle = /^theta[0-3]_/.test(step.label || "") || step.label === "base_twist";
      value.textContent = Number.isFinite(numericValue)
        ? `${numericValue.toFixed(4)}${isAngle ? ` rad · ${(numericValue * 180 / Math.PI).toFixed(2)}°` : ""}`
        : "—";
      const expression = document.createElement("div");
      expression.className = "step-expression";
      expression.textContent = step.expression || "";
      top.append(label, value);
      item.append(top, expression);
      list.append(item);
    }
  }

  function handleIkSolution(message) {
    let target;
    try {
      target = readTarget();
    } catch (error) {
      return;
    }
    if (!targetMatches(message.input, target)) return;

    lastSolution = message;
    const postureSelect = byId("posture-select");
    for (const option of postureSelect.options) {
      const candidate = message.solutions && message.solutions[option.value];
      option.disabled = !candidate || !candidate.valid;
      option.textContent = option.value === "elbow_up" ? "Elbow up" : "Elbow down";
      if (candidate && !candidate.valid) {
        option.textContent += ` — ${candidate.error || "invalid"}`;
      }
    }
    if (postureSelect.selectedOptions[0] && postureSelect.selectedOptions[0].disabled) {
      const valid = Array.from(postureSelect.options).find((option) => !option.disabled);
      if (valid) postureSelect.value = valid.value;
    }
    renderSteps(message.steps);
    byId("preview-label").textContent = Object.values(message.solutions || {}).some((item) => item.valid)
      ? "Candidate postures for this target"
      : "No valid posture for this target";
    byId("target-distance").textContent = `${Math.hypot(message.input.x, message.input.y).toFixed(1)} cm radial · ${message.input.z.toFixed(1)} cm high`;
    window.ArmVisualizer.setSolution(message);
    updateControls();
  }

  function handleMessage(message) {
    switch (message.type) {
      case "pong":
        lastPongAt = Date.now();
        break;
      case "ik_solution":
        handleIkSolution(message);
        break;
      case "state_update":
        updateState(message.state);
        if (message.payload_profile) {
          profile = message.payload_profile;
          byId("payload-select").value = profile;
          byId("profile-summary").textContent = profile.toUpperCase();
        }
        if (message.gripper) byId("gripper-toggle").value = message.gripper;
        if (message.state !== "MOVING") {
          trajectoryActive = false;
          byId("progress-bar").style.width = "0%";
          byId("progress-label").textContent = "No active trajectory";
        }
        break;
      case "trajectory_progress": {
        trajectoryActive = message.progress < 1;
        const percent = Math.max(0, Math.min(100, Number(message.progress) * 100));
        byId("progress-bar").style.width = `${percent}%`;
        byId("progress-label").textContent = `${percent.toFixed(0)}% · ${message.elapsed_ms} / ${message.duration_ms} ms`;
        updateControls();
        break;
      }
      case "error":
        trajectoryActive = false;
        showError(`${message.code || "ERROR"}: ${message.message || "The command failed."}`);
        updateControls();
        break;
      default:
        break;
    }
  }

  function connect() {
    clearTimeout(reconnectTimer);
    setConnection(false, "Connecting");
    try {
      socket = new WebSocket(socketUrl);
    } catch (error) {
      setConnection(false, "Connection error");
      reconnectTimer = setTimeout(connect, 2000);
      return;
    }

    socket.addEventListener("open", () => {
      lastPongAt = Date.now();
      setConnection(true, "Connected");
      send({ type: "ping" });
      preview();
    });
    socket.addEventListener("message", (event) => {
      try {
        handleMessage(JSON.parse(event.data));
      } catch (error) {
        showError("Received an invalid message from the arm.");
      }
    });
    socket.addEventListener("close", () => {
      setConnection(false, "Disconnected");
      reconnectTimer = setTimeout(connect, 1500);
    });
    socket.addEventListener("error", () => {
      setConnection(false, "Connection error");
    });
  }

  async function refreshStatus() {
    try {
      const response = await fetch("/api/status", { cache: "no-store" });
      if (!response.ok) throw new Error(`Status request failed (${response.status})`);
      const status = await response.json();
      updateState(status.state);
      if (status.last_error) showError(status.last_error);
    } catch (error) {
      // The WebSocket indicator remains authoritative if HTTP polling is unavailable.
    }
  }

  async function loadLimits() {
    try {
      const response = await fetch("/api/limits", { cache: "no-store" });
      if (!response.ok) throw new Error(`Limits request failed (${response.status})`);
      limits = await response.json();
      if (!limits.links || !limits.reach) throw new Error("Arm limits response is incomplete.");
      window.ArmVisualizer.setLimits(limits);
      byId("reach-value").textContent = `${Number(limits.reach.min).toFixed(1)}–${Number(limits.reach.max).toFixed(1)} cm`;
      profile = limits.payload_profile || "empty";
      byId("payload-select").value = profile;
      byId("profile-summary").textContent = profile.toUpperCase();
    } catch (error) {
      byId("reach-value").textContent = "Unavailable";
      showError(`Could not load arm limits: ${error.message}`);
    }
  }

  byId("preview-button").addEventListener("click", preview);
  form.addEventListener("submit", move);
  for (const field of fields) {
    form.elements[field].addEventListener("input", invalidatePreview);
  }
  byId("posture-select").addEventListener("change", updateControls);
  byId("stop-button").addEventListener("click", () => send({ type: "stop_request" }));
  byId("clear-estop-button").addEventListener("click", () => send({ type: "clear_estop" }));
  byId("payload-select").addEventListener("change", (event) => {
    const nextProfile = event.target.value;
    if (!send({ type: "set_payload_profile", profile: nextProfile })) {
      event.target.value = profile;
      return;
    }
    profile = nextProfile;
    byId("profile-summary").textContent = profile.toUpperCase();
  });

  heartbeatTimer = setInterval(() => {
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    if (Date.now() - lastPongAt > 4000) {
      socket.close();
      return;
    }
    send({ type: "ping" });
  }, 1000);
  statusTimer = setInterval(refreshStatus, 2500);
  window.addEventListener("beforeunload", () => {
    clearInterval(heartbeatTimer);
    clearInterval(statusTimer);
    clearTimeout(reconnectTimer);
    if (socket) socket.close();
  });

  updateControls();
  loadLimits();
  refreshStatus();
  connect();
})();
