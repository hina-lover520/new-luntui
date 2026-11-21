const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const assert = require('assert');
const root = path.resolve(__dirname, '../..');
const manifest = JSON.parse(fs.readFileSync(path.join(root,
    'project/_backup_before_original_restore/restore_manifest.json'), 'utf8'));
const original = process.argv[2] || manifest.original;
let compared = 0;
const verified = [];
const hash = buffer => crypto.createHash('sha256').update(buffer).digest('hex');

function expected(rel, buffer) {
    if (rel !== 'project/user/main_cm7_1.c') return buffer;
    let text = buffer.toString('latin1');
    for (const [name, value] of Object.entries(manifest.tuningPreserved)) {
        const pattern = new RegExp('(^#define[ \\t]+' + name + '[ \\t]+)\\([^\\r\\n]+?\\)', 'm');
        assert(pattern.test(text), 'Missing original tuning macro: ' + name);
        text = text.replace(pattern, '$1' + value);
    }
    return Buffer.from(text, 'latin1');
}

function compare(rel) {
    const a = fs.readFileSync(path.join(original, rel));
    const b = fs.readFileSync(path.join(root, rel));
    assert(expected(rel, a).equals(b), 'Unexpected difference from original: ' + rel);
    ++compared;
    verified.push({file: rel, sha256: hash(b)});
}

for (const entry of manifest.files) compare(entry.file);
function libraries(dir) {
    for (const entry of fs.readdirSync(path.join(original, dir), {withFileTypes: true})) {
        const rel = dir + '/' + entry.name;
        if (entry.isDirectory()) libraries(rel);
        else if (/\.(c|h|a|s|S|asm)$/.test(entry.name)) compare(rel);
    }
}
libraries('libraries');
// Audit the selected build, not only the formulas within a few functions.
for (const core of [0, 1]) {
    const ewp = fs.readFileSync(path.join(root,
        'project/iar/project_config/cyt4bb7_cm_7_' + core + '.ewp'), 'utf8');
    assert(!/\\(?:ring\.c|track_shared\.h|ring_control\.h)<\/name>/.test(ewp),
           'Experimental ring integration remains in the active project');
}
for (const file of ['main_cm7_0.c', 'main_cm7_1.c', 'cm7_1_isr.c']) {
    const text = fs.readFileSync(path.join(root, 'project/user', file), 'latin1');
    assert(!/\b(?:ring_update|track_publish|track_snapshot|track_timeout_enabled)\b/.test(text),
           'Experimental runtime path remains in ' + file);
}
const report = {original, comparedFiles: compared,
    tuningPreserved: manifest.tuningPreserved, activeRingIntegration: false, verified};
fs.writeFileSync(path.join(__dirname, 'original_baseline_verification.json'), JSON.stringify(report, null, 2) + '\n');
console.log('PASS: ' + compared + ' original source/library/project inputs match byte-for-byte,');
console.log('      allowing only preserved FWD_SPEED_SMALL=190 and FWD_SPEED_LARGE=180; active ring integration absent.');
