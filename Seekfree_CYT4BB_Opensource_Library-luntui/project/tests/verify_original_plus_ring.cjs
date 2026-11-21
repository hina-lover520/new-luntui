const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const assert = require('assert');
const root = path.resolve(__dirname, '../..');
const manifest = JSON.parse(fs.readFileSync(path.join(root,
    'project/_backup_before_original_restore/restore_manifest.json'), 'utf8'));
const original = process.argv[2] || manifest.original;
const runtimeFiles = ['project/user/main_cm7_0.c', 'project/user/main_cm7_1.c', 'project/user/cm7_1_isr.c'];
const projects = [0, 1].map(c => 'project/iar/project_config/cyt4bb7_cm_7_' + c + '.ewp');
const hookFiles = new Set([...runtimeFiles, ...projects]);
const read = rel => fs.readFileSync(path.join(root, rel));
const sha = b => crypto.createHash('sha256').update(b).digest('hex');
const stripHooks = s => s.replace(/^[ \t]*\/\* RING_ADDON_BEGIN \*\/\r?\n[\s\S]*?^[ \t]*\/\* RING_ADDON_END \*\/\r?\n/gm, '')
    .replace(/^[ \t]*<!-- RING_ADDON_BEGIN -->\r?\n[\s\S]*?^[ \t]*<!-- RING_ADDON_END -->\r?\n/gm, '');
const verified = [];
function expected(rel, bytes) {
    if (rel !== 'project/user/main_cm7_1.c') return bytes;
    let s = bytes.toString('latin1');
    for (const [name, value] of Object.entries(manifest.tuningPreserved)) {
        const re = new RegExp('(^#define[ \\t]+' + name + '[ \\t]+)\\([^\\r\\n]+?\\)', 'm');
        assert(re.test(s));
        s = s.replace(re, '$1' + value);
    }
    return Buffer.from(s, 'latin1');
}
function compare(rel) {
    const source = expected(rel, fs.readFileSync(path.join(original, rel)));
    const current = read(rel);
    const base = hookFiles.has(rel) ? Buffer.from(stripHooks(current.toString('latin1')), 'latin1') : current;
    assert(source.equals(base), 'Original code changed outside marked ring additions: ' + rel);
    verified.push({file: rel, currentSha256: sha(current), originalWithPreservedTuningSha256: sha(base)});
}
for (const {file} of manifest.files) compare(file);
function walk(dir) {
    for (const entry of fs.readdirSync(path.join(original, dir), {withFileTypes: true})) {
        const rel = dir + '/' + entry.name;
        if (entry.isDirectory()) walk(rel);
        else if (/\.(c|h|a|s|S|asm)$/.test(entry.name)) compare(rel);
    }
}
walk('libraries');
for (const rel of ['project/iar/cyt4bb7.eww', ...[0, 1].map(c => 'project/iar/project_config/cyt4bb7_cm_7_' + c + '.ewd')])
    assert(read(rel).equals(fs.readFileSync(path.join(original, rel))), 'Original debug configuration changed: ' + rel);
const main0 = read(runtimeFiles[0]).toString('latin1');
const main1 = read(runtimeFiles[1]).toString('latin1');
assert(main0.indexOf('ring_vision_after_frame(') > main0.indexOf('SCB_CleanInvalidateDCache_by_Addr(&m7_1_data_1'));
assert(main0.indexOf('ring_vision_select(') > main0.indexOf('m7_1_data_1[1] = track_state()'));
assert(main1.indexOf('ring_runtime_apply(') > main1.indexOf('balance_turn_out = func_limit_ab('));
assert(main1.indexOf('ring_runtime_init(') > main1.indexOf('    servo_balance_Init('));
assert(main1.indexOf('ring_runtime_init(') < main1.indexOf('    pit_ms_init('));
for (const rel of runtimeFiles) {
    const s = read(rel).toString('latin1');
    assert(!/\b(?:track_publish|track_snapshot|imu_calibration|m7_feedback)\b/.test(s), 'Old replacement protocol or blocking calibration: ' + rel);
}
for (const rel of projects) {
    const s = read(rel).toString('utf8');
    assert(s.includes('ring_link.c'));
    assert(!s.includes('track_shared.h'));
}
const report = {original, comparedInputs: verified.length, activeRingIntegration: true,
    tuningPreserved: manifest.tuningPreserved, method: 'Remove only marked additive hooks, then compare original bytes', verified};
fs.writeFileSync(path.join(__dirname, 'original_plus_ring_verification.json'), JSON.stringify(report, null, 2) + '\n');
console.log('PASS: ' + verified.length + ' original inputs preserved byte-for-byte outside isolated ring hooks;');
console.log('      original tracking, shared array, startup, ISR body and current 190/180 tuning retained.');
