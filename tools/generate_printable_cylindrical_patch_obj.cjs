#!/usr/bin/env node
const fs = require('fs');
const path = require('path');

const outputDir = path.resolve(__dirname, '..', 'public', 'meshes', 'ruled_surface_examples', 'printable');
const objPath = path.join(outputDir, 'cooperative_cylindrical_patch_printable.obj');
const readmePath = path.join(outputDir, 'README.md');

// Match the cooperative ruled-surface demo dimensions, then convert to a
// print-friendly watertight shell in millimetres.
const cfg = {
  radiusMm: 120,
  spanMm: 300,
  arcLengthMm: 300,
  thicknessMm: 8,
  angularSegments: 96,
  axialSegments: 32,
};

function pushVertex(lines, vertex) {
  lines.push(`v ${vertex[0].toFixed(6)} ${vertex[1].toFixed(6)} ${vertex[2].toFixed(6)}`);
}

function triangle(a, b, c) {
  return `f ${a} ${b} ${c}`;
}

function addQuad(faceLines, a, b, c, d) {
  faceLines.push(triangle(a, b, c));
  faceLines.push(triangle(a, c, d));
}

function addReversedQuad(faceLines, a, b, c, d) {
  faceLines.push(triangle(a, d, c));
  faceLines.push(triangle(a, c, b));
}

function buildShellMesh(config) {
  const angleSpan = config.arcLengthMm / config.radiusMm;
  const thetaMin = -0.5 * angleSpan;
  const thetaMax = 0.5 * angleSpan;
  const innerRadius = config.radiusMm;
  const outerRadius = config.radiusMm + config.thicknessMm;
  const halfSpan = 0.5 * config.spanMm;

  const vertices = [];
  const faces = [];
  const edgeCounts = new Map();

  const addVertex = (vertex) => {
    vertices.push(vertex);
    return vertices.length;
  };

  const countTriangleEdges = (a, b, c) => {
    const edges = [
      [a, b],
      [b, c],
      [c, a],
    ];
    for (const [u, v] of edges) {
      const key = u < v ? `${u}-${v}` : `${v}-${u}`;
      edgeCounts.set(key, (edgeCounts.get(key) || 0) + 1);
    }
  };

  const addFace = (line) => {
    faces.push(line);
    const [, sa, sb, sc] = line.split(/\s+/);
    countTriangleEdges(Number(sa), Number(sb), Number(sc));
  };

  const pushQuad = (mode, a, b, c, d) => {
    const collector = [];
    if (mode === 'forward') {
      addQuad(collector, a, b, c, d);
    } else {
      addReversedQuad(collector, a, b, c, d);
    }
    collector.forEach(addFace);
  };

  const inner = [];
  const outer = [];

  for (let axialIndex = 0; axialIndex <= config.axialSegments; axialIndex += 1) {
    const axialT = axialIndex / config.axialSegments;
    const y = -halfSpan + (config.spanMm * axialT);
    const innerRow = [];
    const outerRow = [];
    for (let angularIndex = 0; angularIndex <= config.angularSegments; angularIndex += 1) {
      const angularT = angularIndex / config.angularSegments;
      const theta = thetaMin + ((thetaMax - thetaMin) * angularT);
      const sinTheta = Math.sin(theta);
      const cosTheta = Math.cos(theta);
      innerRow.push(addVertex([innerRadius * sinTheta, y, innerRadius * cosTheta]));
      outerRow.push(addVertex([outerRadius * sinTheta, y, outerRadius * cosTheta]));
    }
    inner.push(innerRow);
    outer.push(outerRow);
  }

  for (let axialIndex = 0; axialIndex < config.axialSegments; axialIndex += 1) {
    for (let angularIndex = 0; angularIndex < config.angularSegments; angularIndex += 1) {
      const i00 = inner[axialIndex][angularIndex];
      const i10 = inner[axialIndex][angularIndex + 1];
      const i01 = inner[axialIndex + 1][angularIndex];
      const i11 = inner[axialIndex + 1][angularIndex + 1];

      const o00 = outer[axialIndex][angularIndex];
      const o10 = outer[axialIndex][angularIndex + 1];
      const o01 = outer[axialIndex + 1][angularIndex];
      const o11 = outer[axialIndex + 1][angularIndex + 1];

      // Inner cylindrical wall.
      pushQuad('reversed', i00, i10, i11, i01);
      // Outer cylindrical wall.
      pushQuad('forward', o00, o01, o11, o10);
    }
  }

  for (let axialIndex = 0; axialIndex < config.axialSegments; axialIndex += 1) {
    const leftInnerA = inner[axialIndex][0];
    const leftInnerB = inner[axialIndex + 1][0];
    const leftOuterA = outer[axialIndex][0];
    const leftOuterB = outer[axialIndex + 1][0];
    pushQuad('forward', leftOuterA, leftOuterB, leftInnerB, leftInnerA);

    const rightInnerA = inner[axialIndex][config.angularSegments];
    const rightInnerB = inner[axialIndex + 1][config.angularSegments];
    const rightOuterA = outer[axialIndex][config.angularSegments];
    const rightOuterB = outer[axialIndex + 1][config.angularSegments];
    pushQuad('forward', rightInnerA, rightInnerB, rightOuterB, rightOuterA);
  }

  for (let angularIndex = 0; angularIndex < config.angularSegments; angularIndex += 1) {
    const bottomInnerA = inner[0][angularIndex];
    const bottomInnerB = inner[0][angularIndex + 1];
    const bottomOuterA = outer[0][angularIndex];
    const bottomOuterB = outer[0][angularIndex + 1];
    pushQuad('forward', bottomInnerA, bottomOuterA, bottomOuterB, bottomInnerB);

    const topInnerA = inner[config.axialSegments][angularIndex];
    const topInnerB = inner[config.axialSegments][angularIndex + 1];
    const topOuterA = outer[config.axialSegments][angularIndex];
    const topOuterB = outer[config.axialSegments][angularIndex + 1];
    pushQuad('forward', topOuterA, topInnerA, topInnerB, topOuterB);
  }

  const nonManifoldEdges = [...edgeCounts.entries()].filter(([, count]) => count !== 2);

  return {
    vertices,
    faces,
    angleSpan,
    chordWidthMm: 2 * innerRadius * Math.sin(angleSpan / 2),
    nonManifoldEdges,
  };
}

function main() {
  fs.mkdirSync(outputDir, { recursive: true });

  const { vertices, faces, angleSpan, chordWidthMm, nonManifoldEdges } = buildShellMesh(cfg);
  if (nonManifoldEdges.length > 0) {
    throw new Error(`Generated mesh is not watertight; found ${nonManifoldEdges.length} non-manifold edges.`);
  }

  const obj = [];
  obj.push('# Cooperative cylindrical patch printable shell');
  obj.push('# Units: millimetres');
  obj.push('# Generated from the dual-arm cooperative ruled-surface demo dimensions');
  obj.push('o cooperative_cylindrical_patch_printable');
  vertices.forEach((vertex) => pushVertex(obj, vertex));
  faces.forEach((line) => obj.push(line));

  const readme = [
    '# Cooperative Cylindrical Patch Printable Shell',
    '',
    '- File: `cooperative_cylindrical_patch_printable.obj`',
    '- Units: millimetres',
    '- Type: watertight printable shell',
    '',
    'Source dimensions copied from the dual-arm cooperative polishing demo:',
    `- inner radius: ${cfg.radiusMm} mm`,
    `- axial span: ${cfg.spanMm} mm`,
    `- arc length: ${cfg.arcLengthMm} mm`,
    `- angular span: ${(angleSpan * 180 / Math.PI).toFixed(3)} deg`,
    `- chord width: ${chordWidthMm.toFixed(3)} mm`,
    '',
    'Printable shell parameters:',
    `- wall thickness: ${cfg.thicknessMm} mm`,
    `- angular segments: ${cfg.angularSegments}`,
    `- axial segments: ${cfg.axialSegments}`,
    '',
    'Notes:',
    '- The inner cylindrical face matches the demo curvature.',
    '- The shell is thickened outward, capped on all four boundary loops, and is watertight.',
    '- Re-run `node dq_vizlab_academic/tools/generate_printable_cylindrical_patch_obj.cjs` to regenerate.',
  ].join('\n');

  fs.writeFileSync(objPath, `${obj.join('\n')}\n`);
  fs.writeFileSync(readmePath, `${readme}\n`);

  console.log(`Wrote ${objPath}`);
  console.log(`Wrote ${readmePath}`);
  console.log(`Vertices: ${vertices.length}`);
  console.log(`Triangles: ${faces.length}`);
  console.log(`Chord width: ${chordWidthMm.toFixed(3)} mm`);
  console.log(`Angular span: ${(angleSpan * 180 / Math.PI).toFixed(3)} deg`);
}

main();
