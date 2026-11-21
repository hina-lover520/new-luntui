const fs = require('fs');
const path = require('path');
const assert = require('assert');
const root = path.resolve(__dirname, '../..');
const read = p => fs.readFileSync(path.join(root, p)).toString('latin1').replace(/\r\n/g, '\n');
const code = s => s.replace(/\/\*[\s\S]*?\*\/|\/\/[^\n]*/g, '').replace(/\s+/g, '');
function control(s) {
    const a = s.indexOf('        balance_steer_error =');
    const b = s.indexOf('\n', s.indexOf('        balance_turn_out = func_limit_ab', a));
    assert(a >= 0 && b > a);
    return code(s.slice(a, b).replaceAll('m7_1_data_1[', 'vision_in['));
}
const main = read('project/user/main_cm7_1.c');
const original = read('project/_backup_pre_ring/project/user/main_cm7_1.c');
const beforeFix = read('project/_backup_pre_follow_fix/project/user/main_cm7_1.c');
assert.strictEqual(control(main), control(original), 'Normal steering/speed/filter calculations changed');
function defines(s) { return s.split('\n').filter(l => /^#define\s/.test(l)).map(code); }
assert.deepStrictEqual(defines(main), defines(beforeFix), 'User tuning macros changed');
const initPattern = /(?:kaerman_b_Init|quaternion_Init|servo_balance_Init|balance_Init|balance_position_Init|balance_speed_Init|balance_turn_Init)\([^;]+\);/g;
const stripComments = s => s.replace(/\/\*[\s\S]*?\*\/|\/\/[^\n]*/g, '');
assert.deepStrictEqual(stripComments(main).match(initPattern).map(code),
                       stripComments(beforeFix).match(initPattern).map(code), 'Controller initialization gains changed');
assert(!/\bimu_calibration\s*\(/.test(stripComments(main)), 'Blocking global gyro calibration still enabled');
assert(main.indexOf('    servo_balance_Init(') < main.indexOf('    pit_ms_init('));
function func(s, name) {
    const start = s.indexOf('void ' + name + '(');
    assert(start >= 0, name);
    const open = s.indexOf('{', start);
    let depth = 1, end = open + 1;
    for (; depth && end < s.length; ++end) {
        if (s[end] === '{') ++depth;
        if (s[end] === '}') --depth;
    }
    return code(s.slice(start, end));
}
const vision = read('project/code/find.c'), oldVision = read('project/_backup_pre_ring/project/code/find.c');
for (const name of ['find_amax', 'find_amax_raw', 'find_mid_raw', 'find_LR', 'binarize'])
    assert.strictEqual(func(vision, name), func(oldVision, name), name + ' changed');
console.log('PASS: original normal controller and vision functions match; current tuning preserved; blocking calibration removed');
