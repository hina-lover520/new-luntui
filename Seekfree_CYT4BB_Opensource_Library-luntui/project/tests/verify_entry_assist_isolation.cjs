const fs=require('fs'),path=require('path'),crypto=require('crypto');
const root=process.argv[2]||path.resolve(__dirname,'../..');
const baseline=require('./entry_assist_baseline_20261112.json');
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
for(const [rel,expected]of Object.entries(baseline.files))
  if(hash(fs.readFileSync(path.join(root,rel)))!==expected)throw Error('Protected v8 input changed: '+rel);
function functionText(source,name){
  const match=new RegExp('(?:static\\s+)?(?:void|int|island_t)\\s+'+name+'\\s*\\(').exec(source);
  if(!match)throw Error('Missing recognition primitive '+name);
  const start=match.index,open=source.indexOf('{',start);let depth=0,state='code';
  for(let i=open;i<source.length;++i){const c=source[i],n=source[i+1];
    if(state==='line'){if(c==='\n')state='code';continue;}
    if(state==='comment'){if(c==='*'&&n==='/'){state='code';++i;}continue;}
    if(state==='string'||state==='char'){if(c==='\\'){++i;continue;}if(c===(state==='string'?'"':"'"))state='code';continue;}
    if(c==='/'&&n==='/'){state='line';++i;continue;}if(c==='/'&&n==='*'){state='comment';++i;continue;}
    if(c==='"'){state='string';continue;}if(c==="'"){state='char';continue;}
    if(c==='{')++depth;else if(c==='}'&&--depth===0)return source.slice(start,i+1);
  }throw Error('Unterminated recognition primitive '+name);
}
const previous=require('./branch_follow_baseline_20261112.json');
const source=fs.readFileSync(path.join(root,'project/code/ring.c'),'utf8');
for(const [name,expected]of Object.entries(previous.acquisition).filter(([name])=>name!=='scan_geometry'))
  if(hash(functionText(source,name))!==expected)throw Error('Existing recognition primitive changed: '+name);
const header=fs.readFileSync(path.join(root,'project/code/ring.h'),'utf8');
const macros=Object.fromEntries([...header.matchAll(/^#define\s+(RING_\w+)\s+(.+)$/gm)].map(m=>[m[1],m[2].trim()]));
for(const [name,value]of Object.entries(previous.macros))
  if(macros[name]!==value)throw Error('Existing constant changed: '+name);
console.log('PASS: '+Object.keys(baseline.files).length+' v10 production inputs byte-identical; 6 acquisition primitives and all previous ring constants unchanged');
