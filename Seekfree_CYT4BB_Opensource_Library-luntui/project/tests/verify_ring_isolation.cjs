/* Compare with the working car's immutable 2026-11-07 snapshot. */
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const baseline = JSON.parse(fs.readFileSync(path.join(__dirname, 'ring_isolation_baseline_20261107.json'), 'utf8'));
const root = process.argv[2] ? path.resolve(process.argv[2]) : baseline.project;
const sha = file => crypto.createHash('sha256').update(fs.readFileSync(file)).digest('hex');
const failures = [], changed = [];
let checked = 0;
for (const item of baseline.files) {
  if (!item.protected && !baseline.allowedProductionChanges.includes(item.file)) continue;
  const file = path.join(root, item.file);
  if (!fs.existsSync(file)) { failures.push(`Missing: ${item.file}`); continue; }
  if (item.protected) {
    ++checked;
    if (sha(file) !== item.sha256) failures.push(`Protected input changed: ${item.file}`);
  } else if (sha(file) !== item.sha256) changed.push(item.file);
}
/* Also reject unreviewed extra production files. */
const expected = new Set(baseline.files.map(x => x.file.toLowerCase()));
const extensions = new Set(baseline.files.filter(x => x.protected).map(x => path.extname(x.file).toLowerCase()));
function walk(relative) {
  for (const item of fs.readdirSync(path.join(root, relative), { withFileTypes: true })) {
    const file = `${relative}/${item.name}`;
    if (item.isDirectory()) walk(file);
    else if (extensions.has(path.extname(file).toLowerCase()) && !expected.has(file.toLowerCase()))
      failures.push(`Unexpected production input: ${file}`);
  }
}
for (const folder of ['project/code', 'project/user', 'libraries']) walk(folder);
const main = fs.readFileSync(path.join(root, 'project/user/main_cm7_1.c')).toString('latin1');
const parameters = {};
for (const name of ['FWD_SPEED_SMALL', 'FWD_SPEED_LARGE', 'TURN_KP1', 'TURN_SAT', 'TURN_KD',
                    'TURN_ERR_ALPHA', 'TURN_SMOOTH_ALPHA', 'TURN_MAX_RPM']) {
  const match = main.match(new RegExp(`^#define\\s+${name}\\s+([^\\r\\n]+)`, 'm'));
  parameters[name] = match ? match[1].split('/*')[0].split('//')[0].trim() : 'MISSING';
}
if (failures.length) {
  console.error(failures.join('\n')); process.exitCode = 1;
} else {
  console.log(`PASS: ${checked} protected inputs byte-for-byte identical to working 2026-11-07 snapshot`);
  console.log(`Allowed ring files changed (${changed.length}): ${changed.join(', ') || 'none (pre-deployment)'}`);
  console.log(`Original parameters preserved: ${JSON.stringify(parameters)}`);
}
