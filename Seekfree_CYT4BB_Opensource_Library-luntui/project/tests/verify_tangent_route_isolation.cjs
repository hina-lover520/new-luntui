const fs=require('fs'),path=require('path'),crypto=require('crypto');
const root=process.argv[2]||path.resolve(__dirname,'../..'),baseline=require('./tangent_route_baseline_20261112.json');
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
for(const [file,expected]of Object.entries(baseline.files))if(hash(fs.readFileSync(path.join(root,file)))!==expected)throw Error('Protected ordinary input changed '+file);
console.log('PASS: '+Object.keys(baseline.files).length+' v11 production inputs byte-identical; ordinary vision, main, ISR, controls and IPM unchanged');
