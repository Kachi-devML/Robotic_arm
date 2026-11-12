"use strict";

(() => {
  const canvas = document.getElementById("arm-canvas");
  const context = canvas.getContext("2d");
  let limits = null;
  let solution = null;

  function setLimits(value) {
    limits = value;
    draw();
  }

  function setSolution(value) {
    solution = value;
    draw();
  }

  function resize() {
    const ratio = window.devicePixelRatio || 1;
    const bounds = canvas.getBoundingClientRect();
    canvas.width = Math.max(1, Math.round(bounds.width * ratio));
    canvas.height = Math.max(1, Math.round(bounds.height * ratio));
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    draw();
  }

  function posturePoints(posture) {
    const links = limits.links;
    const radians = Math.PI / 180;
    const theta1 = posture.theta1 * radians;
    const theta2 = posture.theta2 * radians;
    const theta3 = posture.theta3 * radians;
    const shoulder = { r: 0, z: links.shoulder_height_cm };
    const elbow = {
      r: links.l1_cm * Math.cos(theta1),
      z: links.shoulder_height_cm + links.l1_cm * Math.sin(theta1)
    };
    const wrist = {
      r: elbow.r + links.l2_cm * Math.cos(theta1 + theta2),
      z: elbow.z + links.l2_cm * Math.sin(theta1 + theta2)
    };
    const effector = {
      r: wrist.r + links.l3_cm * Math.cos(theta1 + theta2 + theta3),
      z: wrist.z + links.l3_cm * Math.sin(theta1 + theta2 + theta3)
    };
    return [{ r: 0, z: 0 }, shoulder, elbow, wrist, effector];
  }

  function draw() {
    if (!context) return;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    context.clearRect(0, 0, width, height);
    if (!limits) {
      context.fillStyle = "#91a0b8";
      context.font = "14px system-ui, sans-serif";
      context.fillText("Waiting for arm limits…", 20, 30);
      return;
    }

    const links = limits.links;
    const maxReach = Math.max(
      limits.reach.max + links.l3_cm,
      links.l1_cm + links.l2_cm + links.l3_cm
    );
    const maxHeight = links.shoulder_height_cm + links.l1_cm + links.l2_cm + links.l3_cm;
    const rMin = -5;
    const rMax = maxReach + 4;
    const zMin = -4;
    const zMax = maxHeight + 4;
    const scale = Math.min((width - 72) / (rMax - rMin), (height - 46) / (zMax - zMin));
    const left = (width - (rMax - rMin) * scale) / 2;
    const top = (height - (zMax - zMin) * scale) / 2;
    const point = (r, z) => ({
      x: left + (r - rMin) * scale,
      y: height - top - (z - zMin) * scale
    });

    context.lineWidth = 1;
    context.strokeStyle = "#26354b";
    context.fillStyle = "#71829b";
    context.font = "10px system-ui, sans-serif";
    for (let r = 0; r <= maxReach; r += 5) {
      const p = point(r, 0);
      context.beginPath();
      context.moveTo(p.x, point(0, zMin).y);
      context.lineTo(p.x, point(0, zMax).y);
      context.stroke();
      context.fillText(`${r}`, p.x + 3, height - 7);
    }
    for (let z = 0; z <= maxHeight; z += 5) {
      const p = point(0, z);
      context.beginPath();
      context.moveTo(point(rMin, z).x, p.y);
      context.lineTo(point(rMax, z).x, p.y);
      context.stroke();
      context.fillText(`${z}`, 4, p.y - 3);
    }

    context.setLineDash([5, 5]);
    context.strokeStyle = "#708098";
    context.beginPath();
    context.moveTo(point(0, 0).x, point(0, 0).y);
    context.lineTo(point(0, links.shoulder_height_cm).x, point(0, links.shoulder_height_cm).y);
    context.stroke();
    context.setLineDash([]);

    if (solution && solution.solutions) {
      const candidates = [
        { key: "elbow_up", label: "Elbow up", color: "#60a5fa" },
        { key: "elbow_down", label: "Elbow down", color: "#fbbf24" }
      ].sort((a, b) => Number(solution.solutions[a.key].valid) - Number(solution.solutions[b.key].valid));

      for (const candidate of candidates) {
        const posture = solution.solutions[candidate.key];
        if (posture.error === "TARGET_UNREACHABLE") continue;
        if (![posture.theta1, posture.theta2, posture.theta3].every(Number.isFinite)) continue;
        const points = posturePoints(posture).map(({ r, z }) => point(r, z));
        const color = posture.valid ? candidate.color : "#fb7185";
        context.strokeStyle = color;
        context.fillStyle = color;
        context.lineWidth = posture.valid ? 3.5 : 2.5;
        context.setLineDash(posture.valid ? [] : [7, 5]);
        context.beginPath();
        context.moveTo(points[0].x, points[0].y);
        for (let i = 1; i < points.length; i += 1) context.lineTo(points[i].x, points[i].y);
        context.stroke();
        context.setLineDash([]);
        for (let i = 1; i < points.length; i += 1) {
          context.beginPath();
          context.arc(points[i].x, points[i].y, i === points.length - 1 ? 3 : 4, 0, Math.PI * 2);
          context.fill();
        }
        const end = points[points.length - 1];
        context.font = "11px system-ui, sans-serif";
        context.fillText(`${candidate.label}${posture.valid ? "" : " · limit"}`, end.x + 7, end.y - 7);
      }

      const input = solution.input;
      if (input && Number.isFinite(input.x) && Number.isFinite(input.y) && Number.isFinite(input.z)) {
        const target = point(Math.hypot(input.x, input.y), input.z);
        context.strokeStyle = "#f8fafc";
        context.fillStyle = "#0e1726";
        context.lineWidth = 2;
        context.beginPath();
        context.arc(target.x, target.y, 6, 0, Math.PI * 2);
        context.fill();
        context.stroke();
        context.fillStyle = "#f8fafc";
        context.fillText("Target", target.x + 9, target.y - 8);
      }
    }

    context.fillStyle = "#8192aa";
    context.font = "10px system-ui, sans-serif";
    context.fillText("radial distance (cm)", width - 122, height - 7);
  }

  window.addEventListener("resize", resize);
  window.ArmVisualizer = { setLimits, setSolution };
  resize();
})();
