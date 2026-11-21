const fs=require('fs'),path=require('path'),crypto=require('crypto');
const root=process.argv[2]||path.resolve(__dirname,'../..');
const baseline=require(process.argv[3]||'./v29_preserved_inputs.json');
for(const [file,expected] of Object.entries(baseline.files)) {
 const hash=crypto.createHash('sha256').update(fs.readFileSync(path.join(root,file))).digest('hex');
 if(hash!==expected)throw Error('Ordinary controller/input changed: '+file);
}
console.log('PASS: '+Object.keys(baseline.files).length+' user/main/PID/balance/vision/IPM inputs byte-identical to selected baseline');

