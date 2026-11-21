const fs = require('fs'), path = require('path'), crypto = require('crypto');
const root = process.argv[2] || path.resolve(__dirname, '../..');
const b = require('./copy_ui_baseline_20261111.json');
const hash = s => crypto.createHash('sha256').update(s).digest('hex');
const read = rel => fs.readFileSync(path.join(root, rel)).toString('latin1');
const check = (actual, expected, message) => {
  if (actual !== expected) throw Error(message);
};
for (const [rel, expected] of Object.entries(b.files))
  check(hash(fs.readFileSync(path.join(root, rel))), expected, 'Unrelated input changed: ' + rel);

// Only the requested enable gate and operator UI wiring may change core1.
const stripUi = s => s.replace(/^[ \t]*\/\* OPERATOR_UI_BEGIN[\s\S]*?^[ \t]*\/\* OPERATOR_UI_END \*\/\r?\n/gm, '');
let main1 = stripUi(read('project/user/main_cm7_1.c'));
main1 = main1.replace('if (balance_run_enabled && FIXED_SPEED_ENABLE)', 'if (FIXED_SPEED_ENABLE)')
  .replace(/^        ring_runtime_set_operator\([^\r\n]+\r?\n/m, '');
check(hash(Buffer.from(main1, 'latin1')), b.main1,
  'Original core1 control, parameters or initialization changed beyond the requested run gate');

let isr1 = read('project/user/cm7_1_isr.c');
isr1 = isr1.replace(/    \/\* OPERATOR_UI_BEGIN: standby[^\r\n]*\r\n    if \(balance_run_enabled\) \{\r\n([\s\S]*?)    \}\r\n    \/\* OPERATOR_UI_END \*\/\r\n/, '$1');
isr1 = stripUi(isr1);
check(hash(Buffer.from(isr1, 'latin1')), b.isr1,
  'Existing IMU work or balance ISR calls changed beyond the requested scan/run gate');

const main0 = read('project/user/main_cm7_0.c');
const pipeline = main0.split('            ipm_warp(')[1].split('            /* ---- display')[0];
check(hash(Buffer.from(pipeline, 'latin1')), b.vision_pipeline,
  'Original image processing or steering publication pipeline changed');
console.log('PASS: ' + Object.keys(b.files).length + ' unrelated inputs byte-identical; original control and IMU work unchanged after removing requested UI/start wiring; original vision pipeline byte-identical');
