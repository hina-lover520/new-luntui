const fs=require('fs'),path=require('path'),crypto=require('crypto');
const b=require('./ring_branch_baseline_20261107.json');
const root=process.argv[2]||b.project;
const hash=data=>crypto.createHash('sha256').update(data).digest('hex');
let count=0;
for(const f of b.files.filter(x=>x.protected)) {
 if(hash(fs.readFileSync(path.join(root,f.file)))!==f.sha256)
  throw Error('Ordinary production input changed: '+f.file);
 ++count;
}
console.log('PASS: '+count+' production inputs byte-for-byte unchanged from the working 115200 baseline; both mains, find.c, balance/PID, speed curve, startup, drivers, ISR and build configuration protected');
const main=fs.readFileSync(path.join(root,'project/user/main_cm7_1.c'),'latin1');
console.log(main.match(/^#define\s+FWD_SPEED_(?:SMALL|LARGE)[^\r\n]+/gm).map(x=>x.split('//')[0].trim()).join('; '));
