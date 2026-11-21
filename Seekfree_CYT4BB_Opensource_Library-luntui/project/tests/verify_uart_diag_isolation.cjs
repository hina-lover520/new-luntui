const fs=require('fs'),path=require('path'),crypto=require('crypto');
const b=require('./uart_diagnostics_baseline_20261107.json');
const edits=require('./uart_diagnostics_allowed_main_edits.json');
const root=process.argv[2]||b.project;
const hash=data=>crypto.createHash('sha256').update(data).digest('hex');
let count=0;
for(const f of b.files.filter(x=>x.protected)){
  if(hash(fs.readFileSync(path.join(root,f.file)))!==f.sha256)throw Error('Protected production input changed: '+f.file);
  ++count;
}
const name='project/user/main_cm7_1.c';
let main=fs.readFileSync(path.join(root,name)).toString('latin1');
if(main.includes('#include "ring_uart_diag.h"'))main=main
  .replace(edits.include+'#include "ring_uart_diag.h"\r\n',edits.include)
  .replace(edits.newLine,edits.oldLine);
if (edits.oldUartInit && edits.newUartInit) {
  const oldCount=main.split(edits.oldUartInit).length-1;
  const newCount=main.split(edits.newUartInit).length-1;
  if(oldCount+newCount!==1)throw Error('Expected exactly one authorized UART1 initialization');
  if(newCount===1)main=main.replace(edits.newUartInit,edits.oldUartInit);
}
if(hash(Buffer.from(main,'latin1'))!==b.files.find(x=>x.file===name).sha256)
  throw Error('Main changed beyond the diagnostic include/call and authorized UART1 baud');
console.log('PASS: '+count+' production files byte-for-byte unchanged; ordinary main restores exactly after removing diagnostic include/call and restoring only authorized UART1 baud');
console.log(main.match(/^#define\s+FWD_SPEED_(?:SMALL|LARGE)[^\r\n]+/gm).map(x=>x.split('//')[0].trim()).join('; '));
