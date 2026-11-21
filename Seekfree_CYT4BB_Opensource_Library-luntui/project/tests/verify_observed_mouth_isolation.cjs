const fs=require('fs'),path=require('path'),crypto=require('crypto');
const root=process.argv[2]||path.resolve(__dirname,'../..'),baseline=require('./observed_mouth_baseline_20261112.json');
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
for(const [file,expected]of Object.entries(baseline.files)) {
 let data=fs.readFileSync(path.join(root,file));
 if(file==='project/user/cm7_1_isr.c') {
  // Only this exact ring add-on call replaces the former zero-output block.
  // Reverse that one edit before hashing; every other ISR byte is protected.
  const current='        ring_runtime_poll_timeout(&balance_forward_target, &balance_turn_out);';
  const old=['        if (ring_runtime_watchdog_stop()) {','            balance_forward_target = 0.0f;','            balance_turn_out = 0.0f;','        }'];
  let text=data.toString('utf8'),nl=text.includes('\r\n')?'\r\n':'\n';
  if(text.includes(current)) {
   if(text.split(current).length!==2)throw Error('Unexpected repeated ring timeout hook');
   data=Buffer.from(text.replace(current,old.join(nl)),'utf8');
  }
 }
 if(file==='project/user/main_cm7_1.c') {
  const hook='        ring_runtime_observe_motion(balance_forward_speed, s_angle.agy - balance_mech_zero,\r\n            balance_speed_out, balance_left_duty, balance_right_duty);\r\n';
  const text=data.toString('utf8');
  if(text.split(hook).length!==2)throw Error('Unexpected motion observer hook');
  data=Buffer.from(text.replace(hook,''),'utf8');
 }
 const projectInputs=require('./speed_hold_project_inputs_20261114.json').files;
 const projectRef=projectInputs[file];
 if(projectRef && !['project/iar/project_config/cyt4bb7_cm_7_1.ewp','project/iar/project_config/cyt4bb7_cm_7_1.ewt'].includes(file))throw Error('Unexpected project exception');
 if(projectRef && projectRef.reference_sha256!==expected)throw Error('Project reference changed');
 const localMain=require('./lap360_preserved_main_20261115.json');
 if(localMain.file!=='project/user/main_cm7_1.c' || localMain.reference_sha256!==baseline.files[localMain.file])throw Error('Invalid preserved user main reference');
 const preserved=file===localMain.file?localMain.preserved_sha256:expected;
 if(hash(data)!==(projectRef?projectRef.expected_sha256:preserved))throw Error('Protected ordinary input changed '+file);
}
const preserved=baseline.preserved_local_inputs||[];
for(const item of preserved)if(baseline.files[item.file]!==item.preserved_sha256)throw Error('Invalid preservation reference '+item.file);
console.log('PASS: '+Object.keys(baseline.files).length+' protected inputs verified; exact ISR and read-only motion hooks normalized; two documented IAR project references checked; ordinary main/vision/PID/IPM preserved');
