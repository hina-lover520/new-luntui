// One-time recovery: read the user-provided original; write only this workspace.
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const root = path.resolve(__dirname, '../..');
const original = 'D:/CYT4BB7_Library-master/CYT4BB7_Library-master/Seekfree_CYT4BB_Opensource_Library-luntui/Seekfree_CYT4BB_Opensource_Library-luntui';
const snapshot = path.join(root, 'project/_backup_before_original_restore');
if (fs.existsSync(snapshot)) throw Error('Recovery backup already exists');
const hash = b => crypto.createHash('sha256').update(b).digest('hex');
const mainRelative = 'project/user/main_cm7_1.c';
const currentMain = fs.readFileSync(path.join(root, mainRelative)).toString('latin1');
const tuning = {};
for (const name of ['FWD_SPEED_SMALL', 'FWD_SPEED_LARGE']) {
    const match = currentMain.match(new RegExp('^#define[ \\t]+' + name + '[ \\t]+(\\([^\\r\\n]+?\\))', 'm'));
    if (!match) throw Error('Cannot read current tuning: ' + name);
    tuning[name] = match[1];
}
const files = [];
for (const dir of ['project/user', 'project/code', 'project/iar/icf']) {
    for (const item of fs.readdirSync(path.join(original, dir), {withFileTypes: true})) {
        if (item.isFile() && /\.(c|h|icf)$/.test(item.name)) files.push(dir + '/' + item.name);
    }
}
for (const core of [0, 1]) files.push('project/iar/project_config/cyt4bb7_cm_7_' + core + '.ewp');
const report = {original, snapshot, tuningPreserved: tuning, files: [], restored: []};
for (const rel of files) {
    const source = fs.readFileSync(path.join(original, rel));
    let desired = source;
    if (rel === mainRelative) {
        let text = source.toString('latin1');
        for (const [name, value] of Object.entries(tuning)) {
            const pattern = new RegExp('(^#define[ \\t]+' + name + '[ \\t]+)\\([^\\r\\n]+?\\)', 'm');
            if (!pattern.test(text)) throw Error('Original tuning anchor absent: ' + name);
            text = text.replace(pattern, '$1' + value);
        }
        desired = Buffer.from(text, 'latin1');
    }
    const destination = path.join(root, rel);
    const current = fs.readFileSync(destination);
    report.files.push({file: rel, originalSha256: hash(source), restoredSha256: hash(desired)});
    if (!desired.equals(current)) {
        const backup = path.join(snapshot, rel);
        fs.mkdirSync(path.dirname(backup), {recursive: true});
        fs.writeFileSync(backup, current);
        fs.writeFileSync(destination, desired);
        report.restored.push(rel);
    }
}
// Preserve the disconnected experimental modules, tests and last firmware too.
const extras = [
    'project/code/ring.c', 'project/code/ring.h', 'project/code/ring_control.h',
    'project/code/track_shared.h', 'project/tests/test_ring.c',
    'project/tests/test_ring_control.c', 'project/tests/run_ring_tests.ps1',
    'project/tests/verify_follow_compatibility.cjs'
];
for (const core of [0, 1]) for (const ext of ['hex', 'out'])
    extras.push('project/iar/project_config/Debug_m7_' + core + '/Exe/cyt4bb7_cm_7_' + core + '.' + ext);
for (const rel of extras) {
    if (!fs.existsSync(path.join(root, rel))) continue;
    const destination = path.join(snapshot, rel);
    fs.mkdirSync(path.dirname(destination), {recursive: true});
    fs.copyFileSync(path.join(root, rel), destination);
}
fs.mkdirSync(snapshot, {recursive: true});
fs.writeFileSync(path.join(snapshot, 'restore_manifest.json'), JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify({restored: report.restored, tuningPreserved: tuning, comparedFiles: files.length}, null, 2));
