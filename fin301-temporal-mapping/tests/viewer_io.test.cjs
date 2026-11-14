'use strict';

const {test} = require('node:test');
const assert = require('node:assert/strict');
const IO = require('../viewer/io.js');
const bytes = text => new TextEncoder().encode(text);
const concat = (...items) => Buffer.concat(items.map(item => Buffer.from(item)));
const pcdHeader = (data, extras = '') =>
  `VERSION .7\nFIELDS intensity z descriptor x y\nSIZE 2 4 1 8 4\nTYPE U F U F I\nCOUNT 1 1 3 1 1\n${extras}DATA ${data}\n`;

test('ASCII PCD honors COUNT offsets and filters only nonfinite XYZ', () => {
  const result = IO.parsePCD(bytes(pcdHeader('ascii', 'WIDTH 3\nHEIGHT 1\nPOINTS 3\n') +
    '7 3 9 8 7 1.25 -2\n8 nan 6 5 4 99 100\n1 6 1 2 3 -4.5 5\n'));
  assert.equal(result.count, 2);
  assert.equal(result.filteredCount, 1);
  assert.deepEqual([...result.positions], [1.25, -2, 3, -4.5, 5, 6]);
  assert.deepEqual(result.bounds, {min: [-4.5, -2, 3], max: [1.25, 5, 6]});
  assert.deepEqual(result.fields, ['intensity', 'z', 'descriptor', 'x', 'y']);
  assert.equal(result.format, 'ascii');
});

test('binary PCD honors interleaved little-endian field byte offsets', () => {
  const body = new Uint8Array(21 * 2);
  const data = new DataView(body.buffer);
  [[1.25, -7, 3], [-2.5, 8, 9]].forEach(([x, y, z], i) => {
    const offset = i * 21;
    data.setUint16(offset, 4096, true);
    data.setFloat32(offset + 2, z, true);
    body.set([5, 6, 7], offset + 6);
    data.setFloat64(offset + 9, x, true);
    data.setInt32(offset + 17, y, true);
  });
  const result = IO.parsePCD(concat(bytes(pcdHeader('binary', 'POINTS 2\n')), body));
  assert.deepEqual([...result.positions], [1.25, -7, 3, -2.5, 8, 9]);
  assert.equal(result.format, 'binary');
});

test('PCD can infer counts and safely use a typed-array slice', () => {
  const payload = bytes('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nDATA ascii\n1 2 3\n4 5 6\n');
  const padded = concat([99, 99], payload, [88]);
  const result = IO.parsePCD(padded.subarray(2, padded.length - 1));
  assert.equal(result.sourceCount, 2);
  assert.deepEqual([...result.positions], [1, 2, 3, 4, 5, 6]);
});

test('malformed and dangerous PCD inputs produce recoverable errors', () => {
  const base = 'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n';
  for (const source of [
    base + 'DATA binary_compressed\n',
    base + 'POINTS 9999999999\nDATA ascii\n',
    base + 'POINTS 2\nDATA ascii\n1 2 3\n',
    base + 'POINTS 1\nDATA ascii\n1 2 cat\n',
    base + 'POINTS 1\nDATA ascii\n1e30 2 3\n',
    base + 'WIDTH 3\nHEIGHT 2\nPOINTS 1\nDATA ascii\n1 2 3\n',
    'FIELDS x x z\nSIZE 4 4 4\nTYPE F F F\nDATA ascii\n1 2 3\n',
  ]) assert.throws(() => IO.parsePCD(bytes(source)), Error);
  assert.throws(() => IO.parsePCD(concat(bytes(base + 'POINTS 1\nDATA binary\n'), [0, 0])), /PCD/);
});

test('P5 preserves whitespace and comment-valued first raster bytes', () => {
  for (const first of [9, 10, 13, 32, 35]) {
    const result = IO.parsePGM(concat(bytes('P5\r\n# header comment\r\n2 2\r\n255\r\n'), [first, 2, 0, 255]));
    assert.deepEqual([...result.data], [first, 2, 0, 255]);
    assert.equal(result.width, 2);
    assert.equal(result.height, 2);
  }
  const loneCR = IO.parsePGM(concat(bytes('P5\n2 1\n255\r'), [10, 32]));
  assert.deepEqual([...loneCR.data], [10, 32]);
});

test('P2 comments and 16-bit big-endian P5 values normalize correctly', () => {
  const p2 = IO.parsePGM(bytes('P2\n# test\n3 1\n1023\n0 # dark\n512 1023\n'));
  assert.deepEqual([...p2.data], [0, 128, 255]);
  const p5 = IO.parsePGM(concat(bytes('P5\n3 1\n65535\n'), [0, 0, 128, 0, 255, 255]));
  assert.deepEqual([...p5.data], [0, 128, 255]);
});

test('PGM rejects truncated, oversized and invalid samples', () => {
  assert.throws(() => IO.parsePGM(concat(bytes('P5\n2 2\n255\n'), [0, 1, 2])), /PGM/);
  assert.throws(() => IO.parsePGM(bytes('P2\n2 1\n15\n0 16\n')), /PGM/);
  assert.throws(() => IO.parsePGM(bytes('P2\n99999999 99999999\n255\n')), /PGM/);
  assert.throws(() => IO.parsePGM(bytes('P2\n1 1\n255\n0 2\n')), /PGM/);
});

test('map YAML handles quoted filenames, comments and block origin', () => {
  const result = IO.parseMapYaml(`image: "a # map.pgm" # external comment
resolution: 0.05
origin:
- -34.6
- -28.3
- 1.5707963267948966
negate: false
occupied_thresh: .65
free_thresh: 0.196
mode: trinary
`);
  assert.equal(result.image, 'a # map.pgm');
  assert.deepEqual(result.origin, [-34.6, -28.3, Math.PI / 2]);
  assert.equal(result.negate, 0);
  assert.equal(result.resolution, .05);
});

test('map YAML handles inline origin and escaped single quotes', () => {
  const result = IO.parseMapYaml("---\nimage: 'a''b.pgm'\nresolution: 0.1\norigin: [1, 2, -0.5] # yaw\nnegate: 1\n");
  assert.equal(result.image, "a'b.pgm");
  assert.deepEqual(result.origin, [1, 2, -.5]);
  assert.equal(result.mode, 'trinary');
  assert.equal(result.free_thresh, .196);
  assert.equal(IO.parseMapYaml("image: map's.pgm\nresolution: 1\norigin: [0,0,0]\n").image, "map's.pgm");
});

test('map YAML rejects unsupported modes and invalid/malicious metadata', () => {
  const source = 'image: map.pgm\nresolution: .05\norigin: [0,0,0]\n';
  for (const extra of ['mode: scale', 'mode: raw', 'resolution: 0', 'free_thresh: 0.9\noccupied_thresh: 0.1',
    'origin: !!python/object:call []', 'foo: bar']) {
    assert.throws(() => IO.parseMapYaml(source + extra), /YAML/);
  }
  assert.throws(() => IO.parseMapYaml('image: map.pgm\nresolution: -1\norigin: [0,0]\n'), /YAML/);
});

test('trajectory preserves exact nanoseconds and prefers vehicle columns', () => {
  const result = IO.parseTrajectory('timestamp_ns,timestamp,x,y,yaw,lidar_x,lidar_y,lidar_yaw,pose_frame\r\n' +
    '1793312142514555067,1793312142.514555067,1,2,0.2,9,8,0.7,base_link\r\n' +
    '1793312142514555068,1793312142.514555068,3,4,0.3,7,6,0.8,base_link\r\n');
  assert.equal(result.frame, 'base_link');
  assert.equal(result.poses[0].timestampNs, '1793312142514555067');
  assert.equal(result.poses[0].time, 0);
  assert.equal(result.poses[1].time, 1e-9);
  assert.equal(result.poses[0].x, 1);
  assert.equal(result.poses[0].yaw, .2);
});

test('trajectory supports exact decimal-second CSV and lidar-only coordinates', () => {
  const result = IO.parseTrajectory('timestamp,lidar_x,lidar_y,lidar_yaw\n1.000000001,1,2,0\n1.100000001,2,3,.1\n');
  assert.equal(result.poses[0].timestampNs, '1000000001');
  assert.equal(result.poses[1].time, .1);
  assert.equal(result.frame, 'laser');
});

test('trajectory rejects mixed frames, time reversals and nonfinite poses', () => {
  const header = 'timestamp_ns,x,y,yaw,pose_frame\n';
  for (const rows of ['2,1,2,0,base_link\n1,1,2,0,base_link',
    '1,1,2,0,base_link\n2,1,2,0,laser', '1,NaN,2,0,base_link', '1,1,2,0,base_link\n1,2,3,0,base_link']) {
    assert.throws(() => IO.parseTrajectory(header + rows), /CSV/);
  }
});
