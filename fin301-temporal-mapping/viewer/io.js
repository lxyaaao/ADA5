/* Offline map-file parsers. No network, dynamic code execution or dependencies. */
(function (root, factory) {
  'use strict';
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.MapIO = factory();
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';

  const MAX_BYTES = 128 * 1024 * 1024;
  const MAX_POINTS = 2000000;
  const MAX_PIXELS = 32000000;
  const MAX_COORDINATE = 10000000;
  const NUMBER = /^[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?$/;
  const SPACE = value => value === 9 || value === 10 || value === 11 || value === 12 || value === 13 || value === 32;

  function fail(kind, message) { throw new Error(kind + ': ' + message); }
  function asBytes(input, kind) {
    let bytes;
    if (input instanceof ArrayBuffer) bytes = new Uint8Array(input);
    else if (ArrayBuffer.isView(input)) bytes = new Uint8Array(input.buffer, input.byteOffset, input.byteLength);
    else fail(kind, 'Expected an ArrayBuffer or byte array.');
    if (bytes.byteLength > MAX_BYTES) fail(kind, 'The file exceeds 128 MiB. Crop or downsample it first.');
    return bytes;
  }
  function textInput(input, kind, limit) {
    if (typeof input !== 'string') fail(kind, 'Expected text content.');
    if (input.length > limit) fail(kind, 'The text is too large. Reduce the input data first.');
    return input.replace(/^\uFEFF/, '');
  }
  function integer(token, kind, label, min, max) {
    if (!/^\d+$/.test(String(token))) fail(kind, label + ' must be an integer.');
    const number = Number(token);
    if (!Number.isSafeInteger(number) || number < min || number > max) fail(kind, label + ' is outside the supported range.');
    return number;
  }
  function finiteNumber(token, kind, label) {
    const value = String(token).trim();
    if (!NUMBER.test(value) || !Number.isFinite(Number(value))) fail(kind, label + ' must be a finite number.');
    return Number(value);
  }
  function boundedCoordinate(value, kind) {
    if (Math.abs(value) > MAX_COORDINATE) fail(kind, 'A coordinate exceeds 10,000,000 metres. Check the units and coordinate frame.');
    return value;
  }

  function parsePCD(input) {
    const bytes = asBytes(input, 'PCD');
    const decoder = new TextDecoder('utf-8', {fatal: true});
    const header = Object.create(null);
    let offset = 0;
    let format = '';
    while (offset < bytes.length && offset < 65536) {
      let end = offset;
      while (end < bytes.length && bytes[end] !== 10) end++;
      if (end >= bytes.length || end >= 65536) fail('PCD', 'The header is too long or the newline after DATA is missing.');
      let line;
      try { line = decoder.decode(bytes.subarray(offset, end)).trim(); }
      catch (_) { fail('PCD', 'The header is not valid text.'); }
      offset = end + 1;
      if (!line || line.startsWith('#')) continue;
      if (line.includes('#')) line = line.slice(0, line.indexOf('#')).trim();
      const words = line.split(/\s+/);
      const key = words.shift().toUpperCase();
      if (header[key]) fail('PCD', 'Duplicate header field: ' + key + '.');
      header[key] = words;
      if (key === 'DATA') {
        if (words.length !== 1) fail('PCD', 'DATA must specify exactly one format.');
        format = words[0].toLowerCase();
        break;
      }
    }
    if (!format) fail('PCD', 'The DATA declaration is missing.');
    if (format === 'binary_compressed') fail('PCD', 'binary_compressed is not supported. Save the file as ASCII or uncompressed binary PCD.');
    if (format !== 'ascii' && format !== 'binary') fail('PCD', 'Unsupported data format: ' + format + '.');
    const fields = header.FIELDS || header.FIELD;
    if (!fields || fields.length < 3 || fields.length > 128) fail('PCD', 'A valid FIELDS declaration containing x, y and z is required.');
    const names = fields.map(name => name.toLowerCase());
    if (new Set(names).size !== names.length) fail('PCD', 'FIELDS must not contain duplicate names.');
    const xyzIndices = ['x', 'y', 'z'].map(name => names.indexOf(name));
    if (xyzIndices.some(index => index < 0)) fail('PCD', 'An x, y or z coordinate field is missing.');
    for (const key of ['SIZE', 'TYPE']) {
      if (!header[key] || header[key].length !== fields.length) fail('PCD', key + ' must have one entry for each field in FIELDS.');
    }
    const counts = header.COUNT || fields.map(() => '1');
    if (counts.length !== fields.length) fail('PCD', 'COUNT must have one entry for each field in FIELDS.');
    let stride = 0, columns = 0;
    const layout = fields.map((name, i) => {
      const size = integer(header.SIZE[i], 'PCD', 'SIZE', 1, 8);
      const count = integer(counts[i], 'PCD', 'COUNT', 1, 4096);
      const type = header.TYPE[i].toUpperCase();
      if (!['F', 'I', 'U'].includes(type) || ![1, 2, 4, 8].includes(size) || (type === 'F' && size !== 4 && size !== 8)) {
        fail('PCD', 'Unsupported TYPE/SIZE combination: ' + type + '/' + size + '.');
      }
      const field = {name, size, count, type, offset: stride, column: columns};
      stride += size * count;
      columns += count;
      return field;
    });
    if (stride > 65536 || columns > 16384) fail('PCD', 'Each point contains too many field values.');
    const axes = xyzIndices.map(index => layout[index]);
    if (axes.some(field => field.count !== 1)) fail('PCD', 'x, y and z must be scalar fields (COUNT 1).');
    function metadata(key, fallback) {
      if (!header[key]) return fallback;
      if (header[key].length !== 1) fail('PCD', key + ' must contain exactly one integer.');
      return integer(header[key][0], 'PCD', key, 0, MAX_POINTS);
    }
    let sourceCount = metadata('POINTS', null);
    const width = metadata('WIDTH', null), height = metadata('HEIGHT', 1);
    if (header.HEIGHT && width === null) fail('PCD', 'WIDTH is required when HEIGHT is declared.');
    if (height < 1) fail('PCD', 'HEIGHT must be greater than zero.');
    if (width !== null) {
      const dimensionalCount = width * height;
      if (!Number.isSafeInteger(dimensionalCount) || dimensionalCount > MAX_POINTS) fail('PCD', 'The point count exceeds 2,000,000. Downsample the cloud first.');
      if (sourceCount !== null && sourceCount !== dimensionalCount) fail('PCD', 'POINTS does not match WIDTH multiplied by HEIGHT.');
      sourceCount = dimensionalCount;
    }
    let capacity = sourceCount === null ? 1024 : sourceCount;
    let positions = new Float32Array(capacity * 3);
    let count = 0, seen = 0, filteredCount = 0;
    const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
    function append(point) {
      seen++;
      if (seen > MAX_POINTS || (sourceCount !== null && seen > sourceCount)) fail('PCD', 'The actual point count exceeds the header declaration or viewer limit.');
      if (point.some(value => !Number.isFinite(value))) { filteredCount++; return; }
      point.forEach(value => boundedCoordinate(value, 'PCD'));
      if (count === capacity) {
        capacity = Math.min(MAX_POINTS, Math.max(1, capacity * 2));
        const larger = new Float32Array(capacity * 3);
        larger.set(positions);
        positions = larger;
      }
      point.forEach((value, axis) => {
        positions[count * 3 + axis] = value;
        // Bounds describe the actual Float32 positions consumed by the renderer.
        const stored = positions[count * 3 + axis];
        min[axis] = Math.min(min[axis], stored);
        max[axis] = Math.max(max[axis], stored);
      });
      count++;
    }
    if (format === 'ascii') {
      let body;
      try { body = decoder.decode(bytes.subarray(offset)); }
      catch (_) { fail('PCD', 'The ASCII data contains invalid characters.'); }
      const tokens = /\S+/g;
      let match, column = 0;
      let point = [0, 0, 0];
      while ((match = tokens.exec(body)) !== null) {
        const token = match[0];
        let number;
        if (/^[+-]?(?:nan|inf(?:inity)?)$/i.test(token)) number = /^[-]/.test(token) ? -Infinity : Infinity;
        else number = finiteNumber(token, 'PCD', 'ASCII data');
        for (let axis = 0; axis < 3; axis++) if (column === axes[axis].column) point[axis] = number;
        column++;
        if (column === columns) { append(point); point = [0, 0, 0]; column = 0; }
      }
      if (column !== 0) fail('PCD', 'The ASCII data ends with an incomplete point record.');
    } else {
      const payloadLength = bytes.length - offset;
      if (payloadLength % stride !== 0) fail('PCD', 'The binary data length is not a multiple of the point record size.');
      const inferredCount = payloadLength / stride;
      if (inferredCount > MAX_POINTS || (sourceCount !== null && inferredCount !== sourceCount)) fail('PCD', 'The binary length does not match the declared point count, or the cloud is too large.');
      const view = new DataView(bytes.buffer, bytes.byteOffset + offset, payloadLength);
      function read(field, rowOffset) {
        const address = rowOffset + field.offset;
        if (field.type === 'F') return field.size === 4 ? view.getFloat32(address, true) : view.getFloat64(address, true);
        if (field.size === 8) return Number(field.type === 'I' ? view.getBigInt64(address, true) : view.getBigUint64(address, true));
        const method = 'get' + (field.type === 'I' ? 'Int' : 'Uint') + field.size * 8;
        return view[method](address, true);
      }
      for (let row = 0; row < inferredCount; row++) append(axes.map(field => read(field, row * stride)));
    }
    if (sourceCount !== null && seen !== sourceCount) fail('PCD', 'The actual point count does not match the header declaration.');
    if (count === 0) { min.fill(0); max.fill(0); }
    return {positions: positions.slice(0, count * 3), count, bounds: {min, max}, fields: fields.slice(), format,
      sourceCount: seen, filteredCount};
  }

  function stripYamlComment(line) {
    let quote = null;
    for (let i = 0; i < line.length; i++) {
      const char = line[i];
      if (quote === '"' && char === '\\') { i++; continue; }
      if (quote === "'" && char === "'" && line[i + 1] === "'") { i++; continue; }
      if (char === quote) { quote = null; continue; }
      if (!quote && (char === '"' || char === "'") && (i === 0 || /[\s:[,{]/.test(line[i - 1]))) { quote = char; continue; }
      if (!quote && char === '#' && (i === 0 || /\s/.test(line[i - 1]))) return line.slice(0, i).trimEnd();
    }
    if (quote) fail('YAML', 'A quoted string is not closed.');
    return line.trimEnd();
  }
  function yamlString(value) {
    if (value.startsWith('"')) {
      try {
        const parsed = JSON.parse(value);
        if (typeof parsed !== 'string') fail('YAML', 'Expected a string.');
        return parsed;
      } catch (_) { fail('YAML', 'Invalid double-quoted string.'); }
    }
    if (value.startsWith("'")) {
      if (!/^'(?:[^']|'')*'$/.test(value)) fail('YAML', 'Invalid single-quoted string.');
      return value.slice(1, -1).replace(/''/g, "'");
    }
    if (/[\[\]{}]|^[!&*]/.test(value)) fail('YAML', 'Tags, anchors and complex objects are not supported.');
    return value;
  }

  function parseMapYaml(input) {
    const text = textInput(input, 'YAML', 1024 * 1024);
    const values = Object.create(null);
    let blockOrigin = null;
    let documentEnded = false;
    for (const rawLine of text.split(/\r?\n/)) {
      const line = stripYamlComment(rawLine);
      if (!line.trim()) continue;
      if (documentEnded) fail('YAML', 'Only one map document is supported.');
      if (line.trim() === '---') {
        if (Object.keys(values).length) fail('YAML', 'Only one map document is supported.');
        continue;
      }
      if (line.trim() === '...') { documentEnded = true; continue; }
      const item = line.match(/^\s*-\s+(.+)$/);
      if (item) {
        if (!blockOrigin) fail('YAML', 'Only origin supports a list.');
        blockOrigin.push(item[1].trim());
        continue;
      }
      blockOrigin = null;
      const match = line.match(/^([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*)$/);
      if (!match) fail('YAML', 'Unsupported syntax. Use standard ROS map metadata.');
      const key = match[1], value = match[2].trim();
      if (!['image', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh', 'mode'].includes(key)) fail('YAML', 'Unsupported field: ' + key + '.');
      if (Object.prototype.hasOwnProperty.call(values, key)) fail('YAML', 'Duplicate field: ' + key + '.');
      if (key === 'origin' && value === '') { values.origin = []; blockOrigin = values.origin; }
      else if (key === 'origin') {
        if (!/^\[.*\]$/.test(value)) fail('YAML', 'origin must be [x, y, yaw] or a three-item block list.');
        values.origin = value.slice(1, -1).split(',').map(part => part.trim());
      } else {
        if (!value) fail('YAML', key + ' must not be empty.');
        values[key] = yamlString(value);
      }
    }
    for (const key of ['image', 'resolution', 'origin']) if (!(key in values)) fail('YAML', 'Missing field: ' + key + '.');
    if (!values.image || /[\r\n\0]/.test(values.image)) fail('YAML', 'The image filename is invalid.');
    const resolution = finiteNumber(values.resolution, 'YAML', 'resolution');
    if (resolution <= 0 || resolution > 1000000) fail('YAML', 'resolution must be positive and no greater than 1,000,000.');
    if (!Array.isArray(values.origin) || values.origin.length !== 3) fail('YAML', 'origin must contain exactly three numbers: x, y and yaw.');
    const origin = values.origin.map(value => finiteNumber(value, 'YAML', 'origin'));
    origin.slice(0, 2).forEach(value => boundedCoordinate(value, 'YAML'));
    if (Math.abs(origin[2]) > 1000000) fail('YAML', 'The origin yaw value is too large.');
    const mode = values.mode || 'trinary';
    if (mode !== 'trinary') fail('YAML', 'Only mode: trinary is supported; ' + mode + ' is not supported.');
    let negate = values.negate === undefined ? '0' : String(values.negate).toLowerCase();
    if (!['0', '1', 'false', 'true'].includes(negate)) fail('YAML', 'negate must be 0, 1, false or true.');
    negate = negate === '1' || negate === 'true' ? 1 : 0;
    const occupied_thresh = finiteNumber(values.occupied_thresh === undefined ? '.65' : values.occupied_thresh, 'YAML', 'occupied_thresh');
    const free_thresh = finiteNumber(values.free_thresh === undefined ? '.196' : values.free_thresh, 'YAML', 'free_thresh');
    if (free_thresh < 0 || occupied_thresh > 1 || free_thresh >= occupied_thresh) fail('YAML', 'Thresholds must satisfy 0 <= free_thresh < occupied_thresh <= 1.');
    return {image: values.image, resolution, origin, negate, occupied_thresh, free_thresh, mode};
  }

  function parsePGM(input) {
    const bytes = asBytes(input, 'PGM');
    let cursor = 0;
    function token() {
      while (cursor < bytes.length) {
        if (SPACE(bytes[cursor])) { cursor++; continue; }
        if (bytes[cursor] === 35) {
          while (cursor < bytes.length && bytes[cursor] !== 10 && bytes[cursor] !== 13) cursor++;
          continue;
        }
        break;
      }
      const start = cursor;
      while (cursor < bytes.length && !SPACE(bytes[cursor]) && bytes[cursor] !== 35) cursor++;
      if (cursor === start) return null;
      if (cursor - start > 32) fail('PGM', 'A header or pixel value is too long.');
      return String.fromCharCode(...bytes.subarray(start, cursor));
    }
    const magic = token();
    if (magic !== 'P2' && magic !== 'P5') fail('PGM', 'Only P2 (text) and P5 (binary) grayscale images are supported.');
    const width = integer(token(), 'PGM', 'width', 1, MAX_PIXELS);
    const height = integer(token(), 'PGM', 'height', 1, MAX_PIXELS);
    const count = width * height;
    if (!Number.isSafeInteger(count) || count > MAX_PIXELS) fail('PGM', 'The pixel count exceeds 32,000,000. Reduce the map dimensions first.');
    const maximum = integer(token(), 'PGM', 'maxval', 1, 65535);
    const data = new Uint8Array(count);
    if (magic === 'P2') {
      for (let i = 0; i < count; i++) data[i] = Math.round(integer(token(), 'PGM', 'Pixel value', 0, maximum) * 255 / maximum);
      if (token() !== null) fail('PGM', 'The actual pixel count exceeds width multiplied by height.');
    } else {
      // P5 has one required whitespace separator, not an arbitrary whitespace skip.
      // The very first raster byte can itself be LF, space, CR or '#'.
      if (!SPACE(bytes[cursor])) fail('PGM', 'The binary data separator after maxval is missing.');
      const delimiter = bytes[cursor++];
      const sampleBytes = maximum < 256 ? 1 : 2;
      // Resolve CR versus CRLF using the exact raster length; a lone CR may
      // legitimately be followed by a first pixel whose value is LF (10).
      if (delimiter === 13 && bytes[cursor] === 10 && bytes.length - cursor === count * sampleBytes + 1) cursor++;
      if (bytes.length - cursor !== count * sampleBytes) fail('PGM', 'The binary pixel length does not match the dimensions. Check for truncation or extra header characters.');
      for (let i = 0; i < count; i++) {
        const value = sampleBytes === 1 ? bytes[cursor++] : bytes[cursor++] * 256 + bytes[cursor++];
        if (value > maximum) fail('PGM', 'A pixel value exceeds maxval.');
        data[i] = Math.round(value * 255 / maximum);
      }
    }
    return {width, height, data};
  }

  function csvRows(input) {
    const text = textInput(input, 'CSV', 64 * 1024 * 1024);
    const rows = [];
    let row = [], cell = '', quoted = false, closed = false;
    function endCell() { row.push(cell.trim()); cell = ''; closed = false; }
    function endRow() {
      endCell();
      if (row.some(value => value !== '')) rows.push(row);
      if (rows.length > MAX_POINTS + 1) fail('CSV', 'The trajectory contains too many rows.');
      row = [];
    }
    for (let i = 0; i < text.length; i++) {
      const char = text[i];
      if (quoted) {
        if (char === '"' && text[i + 1] === '"') { cell += '"'; i++; }
        else if (char === '"') { quoted = false; closed = true; }
        else cell += char;
      } else if (char === ',' || char === '\n' || char === '\r') {
        if (char === ',') endCell();
        else { endRow(); if (char === '\r' && text[i + 1] === '\n') i++; }
      } else if (char === '"') {
        if (cell.trim() || closed) fail('CSV', 'Quotes must enclose an entire field.');
        cell = ''; quoted = true;
      } else {
        if (closed && !/\s/.test(char)) fail('CSV', 'Unexpected characters follow a closing quote.');
        cell += char;
      }
    }
    if (quoted) fail('CSV', 'A quoted string is not closed.');
    if (cell || row.length) endRow();
    return rows;
  }
  function secondsToNanoseconds(value) {
    const match = value.match(/^([+-]?)(\d*)(?:\.(\d*))?(?:[eE]([+-]?\d+))?$/);
    if (!match || !(match[2] || match[3])) fail('CSV', 'timestamp must be a valid number of seconds.');
    const exponent = Number(match[4] || '0');
    const fraction = match[3] || '';
    if (Math.abs(exponent) > 30 || match[2].length + fraction.length > 40) fail('CSV', 'The timestamp value is too large.');
    let number = BigInt((match[2] || '0') + fraction);
    const scale = 9 + exponent - fraction.length;
    if (scale >= 0) number *= 10n ** BigInt(scale);
    else {
      const divisor = 10n ** BigInt(-scale);
      number = (number + divisor / 2n) / divisor;
    }
    return (match[1] === '-' ? -number : number).toString();
  }
  function parseTrajectory(input) {
    const rows = csvRows(input);
    if (rows.length < 2) fail('CSV', 'A header and at least one trajectory row are required.');
    const header = rows.shift().map(name => name.toLowerCase());
    if (new Set(header).size !== header.length) fail('CSV', 'The header contains duplicate fields.');
    const indices = Object.fromEntries(header.map((name, index) => [name, index]));
    const hasMain = ['x', 'y', 'yaw'].every(name => name in indices);
    const hasLidar = ['lidar_x', 'lidar_y', 'lidar_yaw'].every(name => name in indices);
    if (!hasMain && !hasLidar) fail('CSV', 'A complete set of x,y,yaw or lidar_x,lidar_y,lidar_yaw columns is required.');
    const names = hasMain ? ['x', 'y', 'yaw'] : ['lidar_x', 'lidar_y', 'lidar_yaw'];
    const timeColumn = ['timestamp_ns', 'timestamp', 'time'].find(name => name in indices);
    if (!timeColumn) fail('CSV', 'A timestamp_ns, timestamp or time column is required.');
    const frameColumn = hasMain ? ['pose_frame', 'frame_id', 'frame'].find(name => name in indices) : null;
    let frame = null, firstStamp = null, previousStamp = null;
    const poses = rows.map((row, rowIndex) => {
      if (row.length !== header.length) fail('CSV', 'Row ' + (rowIndex + 2) + ' has a different column count from the header.');
      const stampText = row[indices[timeColumn]];
      if (timeColumn === 'timestamp_ns' && !/^-?\d{1,20}$/.test(stampText)) fail('CSV', 'timestamp_ns must be a full decimal integer, without a decimal point or scientific notation.');
      const timestampNs = timeColumn === 'timestamp_ns' ? stampText : secondsToNanoseconds(stampText);
      const stamp = BigInt(timestampNs);
      if (stamp < -(2n ** 63n) || stamp >= 2n ** 63n) fail('CSV', 'The timestamp is outside the signed 64-bit nanosecond range.');
      if (previousStamp !== null && stamp <= previousStamp) fail('CSV', 'Timestamps must increase strictly. A duplicate or out-of-order record was found.');
      if (firstStamp === null) firstStamp = stamp;
      previousStamp = stamp;
      const point = names.map(name => finiteNumber(row[indices[name]], 'CSV', name));
      point.slice(0, 2).forEach(value => boundedCoordinate(value, 'CSV'));
      if (Math.abs(point[2]) > 1000000) fail('CSV', 'The yaw value is too large.');
      const currentFrame = frameColumn ? row[indices[frameColumn]] || 'unknown' : (hasMain ? 'unknown' : 'laser');
      if (frame !== null && frame !== currentFrame) fail('CSV', 'The trajectory contains different pose_frame values and cannot be displayed as one path.');
      frame = currentFrame;
      return {timestampNs, time: Number(stamp - firstStamp) / 1e9, x: point[0], y: point[1], yaw: point[2]};
    });
    return {poses, frame, timeOriginNs: firstStamp.toString()};
  }

  return {parsePCD, parseMapYaml, parsePGM, parseTrajectory};
});
