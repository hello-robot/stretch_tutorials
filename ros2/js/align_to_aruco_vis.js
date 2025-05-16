const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

// Define constants for visualization
const VISUAL_SCALE = 100;

// Utility function for coordinate conversion
function toCanvasCoords(x, y) {
  return [canvas.width / 2 + x * VISUAL_SCALE, canvas.height / 2 - y * VISUAL_SCALE];
}

function radians(deg) {
  return (deg * Math.PI) / 180;
}

function degrees(rad) {
  return (rad * 180) / Math.PI;
}

function getRotationMatrixZ(theta) {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [c, -s, 0, 0],
    [s, c, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

function mat4mulVec4(mat, vec) {
  const result = new Array(4).fill(0);
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      result[i] += mat[i][j] * vec[j];
    }
  }
  return result;
}

function updateSliderDisplays() {
  document.getElementById("offsetVal").textContent =
    document.getElementById("offset").value;
  document.getElementById("markerXVal").textContent =
    document.getElementById("markerX").value;
  document.getElementById("markerYVal").textContent =
    document.getElementById("markerY").value;
  document.getElementById("markerRotVal").textContent =
    document.getElementById("markerRot").value;
}

function drawGrid() {
  const gridSize = 25; // Size of grid squares in pixels
  const width = canvas.width;
  const height = canvas.height;

  // Draw light gray grid lines
  ctx.strokeStyle = "#e0e0e0";
  ctx.lineWidth = 1;

  // Vertical lines
  for (let x = 0; x < width; x += gridSize) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }

  // Horizontal lines
  for (let y = 0; y < height; y += gridSize) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  // Draw darker axes
  ctx.strokeStyle = "#cccccc";
  ctx.lineWidth = 2;

  // X-axis (horizontal)
  ctx.beginPath();
  ctx.moveTo(0, height / 2);
  ctx.lineTo(width, height / 2);
  ctx.stroke();

  // Y-axis (vertical)
  ctx.beginPath();
  ctx.moveTo(width / 2, 0);
  ctx.lineTo(width / 2, height);
  ctx.stroke();
}

// Add animation variables
let animating = false;
let animationStep = 0;
const totalAnimationSteps = 100;
let robotPosition = [0, 0]; // x, y
let robotRotation = 0;
let animationPhases = [];
let requestId = null;

function startAnimation() {
  const animateButton = document.getElementById("animateButton");

  if (animating || animationStep >= totalAnimationSteps) {
    // Stop any existing animation
    cancelAnimationFrame(requestId);
    animating = false;
    animationStep = 0;
    animateButton.textContent = "Animate Robot";
    draw(); // Redraw the canvas in its final state
    return;
  }

  // Start animation
  animating = true;
  animateButton.textContent = "Stop Animation";

  // Calculate animation parameters
  const offset = parseFloat(document.getElementById("offset").value);
  const marker_x = parseFloat(document.getElementById("markerX").value);
  const marker_y = parseFloat(document.getElementById("markerY").value);
  const marker_rot_deg = parseFloat(document.getElementById("markerRot").value);

  const rotation_angle = radians(marker_rot_deg);
  const R_mat = getRotationMatrixZ(rotation_angle);

  // Calculate the target position
  const P_dash = [0, -offset, 0, 1];
  const P = [marker_x, marker_y, 0, 1];

  const X = mat4mulVec4(R_mat, P_dash);
  const P_base = [X[0] + P[0], X[1] + P[1], 0, 1];
  const base_x = P_base[0];
  const base_y = P_base[1];

  const phi = Math.atan2(base_y, base_x);
  const dist = Math.sqrt(base_x ** 2 + base_y ** 2);
  const z_rot_marker = rotation_angle;
  const z_rot_base = -phi + z_rot_marker + Math.PI;

  // Initialize robot state
  robotPosition = [0, 0];
  robotRotation = 0;

  // Define animation phases (turn, move, turn)
  animationPhases = [
    {
      // Phase 1: Turn to face target
      steps: Math.floor(totalAnimationSteps * 0.25),
      action: (progress) => {
        robotRotation = progress * -phi;
      },
    },
    {
      // Phase 2: Move to target position
      steps: Math.floor(totalAnimationSteps * 0.5),
      action: (progress) => {
        robotPosition = [progress * base_x, progress * base_y];
      },
    },
    {
      // Phase 3: Final rotation to align with marker
      steps: Math.floor(totalAnimationSteps * 0.25),
      action: (progress) => {
        // Target is z_rot_base but we need to calculate the difference from current phi
        const finalRotCorrection = -z_rot_base;
        robotRotation = -phi + progress * finalRotCorrection;
      },
    },
  ];

  animationStep = 0;
  animate();
}

function animate() {
  if (!animating) return;

  // Clear and redraw the background
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawGrid();

  // Draw marker, axes, etc.
  const offset = parseFloat(document.getElementById("offset").value);
  const marker_x = parseFloat(document.getElementById("markerX").value);
  const marker_y = parseFloat(document.getElementById("markerY").value);
  const marker_rot_deg = parseFloat(document.getElementById("markerRot").value);

  // Calculate the current animation phase
  let currentPhase = 0;
  let currentPhaseStep = animationStep;

  // Find the current phase
  for (let i = 0; i < animationPhases.length; i++) {
    if (currentPhaseStep < animationPhases[i].steps) {
      currentPhase = i;
      break;
    } else {
      currentPhaseStep -= animationPhases[i].steps;
    }
  }

  // Execute phase animation
  for (let i = 0; i <= currentPhase; i++) {
    const phase = animationPhases[i];
    const progress =
      i === currentPhase ? currentPhaseStep / phase.steps : 1.0; // Completed phases use full progress

    // Apply phase action
    phase.action(progress);
  }

  // Convert world coordinates to canvas coordinates
  const [robotX, robotY] = toCanvasCoords(robotPosition[0], robotPosition[1]);

  // Render the robot
  drawRobot(robotX, robotY, robotRotation);

  // Render static elements
  renderStaticScene(offset, marker_x, marker_y, marker_rot_deg);

  // Advance animation
  animationStep++;
  if (animationStep >= totalAnimationSteps) {
    animating = false;
    return; // Keep the button labeled "Stop Animation"
  }

  requestId = requestAnimationFrame(animate);
}

// Preload the robot image
const robotImg = new Image();
robotImg.src = "../images/align_aruco_stretch_base.png";
function drawRobot(x, y, rotation) {
  // Uniformly scale the robot image based on its natural aspect ratio
  const scale = 2.0; // Adjust this value to scale up/down uniformly
  ctx.save();

  // Center stretch base image with offsets
  const xOffset = 0;
  const yOffset = 9;
  ctx.translate(x + xOffset, y + yOffset);
  rotation = rotation + Math.PI / 2; // Adjust rotation to match the robot's orientation
  ctx.rotate(rotation);

  if (robotImg.complete && robotImg.naturalWidth && robotImg.naturalHeight) {
    const aspect = robotImg.naturalWidth / robotImg.naturalHeight;
    const baseSize = 60; // base size for scaling
    let drawWidth, drawHeight;
    if (aspect >= 1) {
      drawWidth = baseSize * scale;
      drawHeight = (baseSize / aspect) * scale;
    } else {
      drawHeight = baseSize * scale;
      drawWidth = (baseSize * aspect) * scale;
    }
    ctx.drawImage(
      robotImg,
      -drawWidth / 2,
      -drawHeight / 2,
      drawWidth,
      drawHeight
    );
  } else {
    // If image not loaded yet, draw a placeholder circle
    const robotSize = 120;
    ctx.beginPath();
    ctx.arc(0, 0, robotSize / 2, 0, 2 * Math.PI);
    ctx.fillStyle = "#4285F4";
    ctx.fill();
  }

  ctx.restore();
}

// Preload the marker image
const markerImg = new Image();
markerImg.src = "../images/aruco_131_6x6.png";
markerImg.onload = function() {
  // Redraw when the image is loaded
  if (!animating) draw();
};

function renderStaticScene(offset, marker_x, marker_y, marker_rot_deg) {
  // Set better text rendering
  ctx.font = "15px Arial";
  ctx.textBaseline = "middle";

  // Define visualization scale once to use consistently
  const visualScale = 100;

  const marker_translation = [marker_x, marker_y, 0];
  const rotation_angle = radians(marker_rot_deg);
  const R_mat = getRotationMatrixZ(rotation_angle);

  const P_dash = [0, -offset, 0, 1];
  const P = [marker_translation[0], marker_translation[1], 0, 1];

  const X = mat4mulVec4(R_mat, P_dash);
  const P_base = [X[0] + P[0], X[1] + P[1], 0, 1];
  const base_x = P_base[0];
  const base_y = P_base[1];

  const phi = Math.atan2(base_y, base_x);
  const dist = Math.sqrt(base_x ** 2 + base_y ** 2);
  const z_rot_marker = rotation_angle;
  const z_rot_base = -phi + z_rot_marker + Math.PI;

  const phi_deg = degrees(phi).toFixed(1);
  const z_rot_base_deg = degrees(z_rot_base).toFixed(1);
  const reverse_offset_angle = Math.atan2(-base_y, -base_x);

  function drawArrow(x1, y1, dx, dy, color, label = null, options = {}) {
    // Set default options
    const defaults = {
      lineWidth: 2,
      headLength: 10,
      bold: false,
      labelOffset: { x: 5, y: -5 },
      boldFont: false
    };

    const settings = {...defaults, ...options};
    const x2 = x1 + dx;
    const y2 = y1 + dy;

    // Draw the line with a slight shadow effect
    ctx.save();
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.strokeStyle = color;
    ctx.lineWidth = settings.bold ? 3 : settings.lineWidth;
    ctx.shadowColor = "rgba(0, 0, 0, 0.3)";
    ctx.shadowBlur = 3;
    ctx.shadowOffsetX = 1;
    ctx.shadowOffsetY = 1;
    ctx.stroke();
    ctx.restore();

    // Draw the arrowhead
    const angle = Math.atan2(dy, dx);
    const headLength = settings.bold ? 15 : settings.headLength;
    const angleOffset = Math.PI / 7; // slightly wider arrowhead angle

    const x3 = x2 - headLength * Math.cos(angle - angleOffset);
    const y3 = y2 - headLength * Math.sin(angle - angleOffset);
    const x4 = x2 - headLength * Math.cos(angle + angleOffset);
    const y4 = y2 - headLength * Math.sin(angle + angleOffset);

    ctx.beginPath();
    ctx.moveTo(x2, y2);
    ctx.lineTo(x3, y3);
    ctx.lineTo(x4, y4);
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();

    // Optional label without background
    if (label) {
      // Label text - adding text shadow for better readability
      ctx.save();
      ctx.fillStyle = color;
      ctx.shadowColor = "white";
      ctx.shadowBlur = 3;
      ctx.shadowOffsetX = 0;
      ctx.shadowOffsetY = 0;
      if (settings.boldFont) {
        ctx.font = 'bold 18px Arial';
      }
      ctx.fillText(label, x2 + settings.labelOffset.x, y2 + settings.labelOffset.y);
      ctx.restore();
    }
  }

  // Helper function for drawing arcs
  function drawArc(centerX, centerY, radius, startAngle, endAngle, color) {
    const steps = 100;
    const angleStep = (endAngle - startAngle) / steps;
    
    ctx.beginPath();
    for (let i = 0; i <= steps; i++) {
      const angle = startAngle + angleStep * i;
      const x = centerX + (radius / 100) * Math.cos(angle);
      const y = centerY + (radius / 100) * Math.sin(angle);
      const [px, py] = toCanvasCoords(x, y);
      
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 3]);
    ctx.stroke();
    ctx.restore();
    ctx.setLineDash([]);
    
    return (startAngle + endAngle) / 2; // Return midpoint angle for label
  }

  const [ox, oy] = toCanvasCoords(0, 0);
  const [mx, my] = toCanvasCoords(marker_x, marker_y);
  const [bx, by] = toCanvasCoords(base_x, base_y);

  // Draw marker-local x/y axes
  const axisLen = 0.15;
  const angle = rotation_angle;
  const x_axis = [axisLen * Math.cos(angle), axisLen * Math.sin(angle)];
  const y_axis = [-axisLen * Math.sin(angle), axisLen * Math.cos(angle)];

  // Draw the marker image centered at (mx, my), rotated by marker_rot_deg
  // Image size (adjust as needed)
  const imgSize = 40;
  // If image is already loaded (from cache), draw immediately
  if (markerImg.complete) {
    ctx.save();
    ctx.translate(mx, my);
    ctx.rotate(-rotation_angle);
    ctx.drawImage(markerImg, -imgSize / 2, -imgSize / 2, imgSize, imgSize);
    ctx.restore();
    // Draw axes on top of the loaded image
    drawAxes();
  } else {
    // If image isn't loaded yet, draw axes anyway
    drawAxes();
  }

  // Draw distance arrow
  drawArrow(
    ox,
    oy,
    bx - ox,
    by - oy,
    "#4285F4",
    `dist=${dist.toFixed(2)}`,
    { labelOffset: { x: -(bx - ox) / 2 + 5, y: -(by - oy) / 2 } }
  );

  // φ arc
  const arc_radius = 20;
  const midPhiAngle = drawArc(0, 0, arc_radius, 0, phi, "hsla(196, 86%, 29%, 1)");

  const [textX, textY] = toCanvasCoords(
    0.2 * Math.cos(midPhiAngle),
    0.2 * Math.sin(midPhiAngle)
  );

  // Add phi label
  const phiLabel = `φ=${phi_deg}°`;
  ctx.save();
  ctx.fillStyle = "hsla(196, 86%, 29%, 1)";
  ctx.shadowColor = "white";
  ctx.shadowBlur = 3;
  ctx.shadowOffsetX = 0;
  ctx.shadowOffsetY = 0;
  ctx.fillText(phiLabel, textX + 10, textY);
  ctx.restore();

  // z_rot_base arc
  const arc_radius_z = 20;
  const theta_start = reverse_offset_angle + Math.PI;
  const midZRotAngle = drawArc(
    base_x,
    base_y,
    arc_radius_z,
    theta_start,
    theta_start + z_rot_base,
    "hsla(196, 86%, 29%, 1)"
  );

  const labelDist = (arc_radius_z * 1.2) / 100;
  const label_x = base_x + labelDist * Math.cos(midZRotAngle);
  const label_y = base_y + labelDist * Math.sin(midZRotAngle);
  const [lx, ly] = toCanvasCoords(label_x, label_y);

  const zRotLabel = `z_rot_base=${z_rot_base_deg}°`;
  ctx.save();
  ctx.fillStyle = "hsla(196, 86%, 29%, 1)";
  ctx.shadowColor = "white";
  ctx.shadowBlur = 3;
  ctx.shadowOffsetX = 0;
  ctx.shadowOffsetY = 0;
  ctx.fillText(zRotLabel, lx + 10, ly - 5);
  ctx.restore();

  // Axes
  drawArrow(ox, oy, 40, 0, "red", "x");
  drawArrow(ox, oy, 0, -40, "green", "y");

  // Initial pose arrow (along x-axis)
  if (!animating) {
    const initial_pose_len = 0.35;
    drawArrow(
      ox,
      oy,
      initial_pose_len * visualScale,
      0,
      "hsla(240, 57.77%, 49.22%, 1)",
      "Initial Pose",
      {
        bold: true,
        boldFont: true,
        labelOffset: { x: -85, y: 15 },
      }
    );
  }
  
  // Function to draw axes on top of the marker image
  function drawAxes() {
    const axis_length_px = 40;
    
    const x_dx = axis_length_px * Math.cos(angle);
    const x_dy = -axis_length_px * Math.sin(angle);
    const y_dx = -axis_length_px * Math.sin(angle);
    const y_dy = -axis_length_px * Math.cos(angle);
    
    drawArrow(mx, my, x_dx, x_dy, "red", "x");
    drawArrow(mx, my, y_dx, y_dy, "green", "y");
  }


  // Final pose arrow
  const final_pose_len = 0.35;
  const final_angle = z_rot_base + phi;
  const final_dx = final_pose_len * Math.cos(final_angle);
  const final_dy = final_pose_len * Math.sin(final_angle);
  const [fdx, fdy] = [final_dx * visualScale, -final_dy * visualScale];

  if (!animating) {
    // Only draw final pose arrow when not animating
    drawArrow(bx, by, fdx, fdy, "hsla(240, 57.77%, 49.22%, 1)", "Final Pose", {
      bold: true,
      boldFont: true,
      labelOffset: { x: 5, y: -10 },
    });
  }

  // Draw offset point
  ctx.save();
  ctx.shadowColor = "rgba(0, 0, 0, 0.5)";
  ctx.shadowBlur = 5;
  ctx.shadowOffsetX = 2;
  ctx.shadowOffsetY = 2;
  ctx.fillStyle = "#1a237e";
  ctx.beginPath();
  ctx.arc(bx, by, 6, 0, 2 * Math.PI);
  ctx.fill();
  ctx.restore();

  ctx.fillStyle = "white";
  ctx.beginPath();
  ctx.arc(bx, by, 3, 0, 2 * Math.PI);
  ctx.fill();

  const offsetLabel = "Offset";
  ctx.save();
  ctx.fillStyle = "#1a237e";
  ctx.shadowColor = "white";
  ctx.shadowBlur = 3;
  ctx.shadowOffsetX = 0;
  ctx.shadowOffsetY = 0;
  ctx.fillText(offsetLabel, bx + 10, by + 10);
  ctx.restore();
}

function resetInputs() {
  document.getElementById("offset").value = 0.75;
  document.getElementById("markerX").value = 1.2;
  document.getElementById("markerY").value = 1.2;
  document.getElementById("markerRot").value = -90;
  updateSliderDisplays();

  // Stop animation if running
  if (animating || animationStep >= totalAnimationSteps) {
    animating = false;
    animationStep = 0;
    document.getElementById("animateButton").textContent = "Animate Robot";
    cancelAnimationFrame(requestId);
  }

  draw();
}

function draw() {
  // Only draw regular scene if not animating
  if (!animating) {
    const offset = parseFloat(document.getElementById("offset").value);
    const marker_x = parseFloat(document.getElementById("markerX").value);
    const marker_y = parseFloat(document.getElementById("markerY").value);
    const marker_rot_deg = parseFloat(document.getElementById("markerRot").value);

    // Clear and setup canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw grid lines
    drawGrid();

    // Render the static scene
    renderStaticScene(offset, marker_x, marker_y, marker_rot_deg);
  }
}

// Attach listeners
document.querySelectorAll("input").forEach((input) => {
  input.addEventListener("input", () => {
    updateSliderDisplays();
    // Stop animation if running when sliders change
    if (animating || animationStep >= totalAnimationSteps) {
      animating = false;
      animationStep = 0;
      document.getElementById("animateButton").textContent = "Animate Robot";
      cancelAnimationFrame(requestId);
    }
    draw();
  });
});

draw(); // initial draw
// Make canvas responsive
window.addEventListener("resize", function () {
  // Only redraw, no need to resize canvas as we have a fixed layout
  draw();
});