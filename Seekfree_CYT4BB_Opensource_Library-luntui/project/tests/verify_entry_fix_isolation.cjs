const fs = require('fs'), path = require('path'), crypto = require('crypto');
const root = process.argv[2] || path.resolve(__dirname, '../..');
const b = require('./entry_fix_baseline_20261112.json');
const hash = data => crypto.createHash('sha256').update(data).digest('hex');
for (const [rel, expected] of Object.entries(b.files)) {
  const actual = hash(fs.readFileSync(path.join(root, rel)));
  if (actual !== expected) throw Error('Protected v7 input changed: ' + rel);
}
console.log('PASS: ' + Object.keys(b.files).length + ' v7 production inputs byte-identical, including both mains, ISR, startup/buttons/display, ordinary tracking, camera/IPM, balance/PID, drivers and build configuration');
