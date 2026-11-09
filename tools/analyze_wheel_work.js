#!/usr/bin/env node
'use strict';
// Uses exactly the same decoder and integral as the browser.
const fs=require('fs'), path=require('path');
const work=require('../site/wheel-work.js');
const [input,outDir,sign,inertia]=process.argv.slice(2);
if(!input||!outDir){console.error('node tools/analyze_wheel_work.js INPUT.rwlog OUTPUT_DIR [sign=-1|1] [inertia_kg_m2]');process.exit(2);}
const options={};if(sign!==undefined)options.sign=Number(sign);if(inertia!==undefined)options.inertia=Number(inertia);
const report=work.fromBytes(new Uint8Array(fs.readFileSync(input)),options);
fs.mkdirSync(outDir,{recursive:true});
fs.writeFileSync(path.join(outDir,'wheel_work.json'),JSON.stringify(report,null,2));
for(const kind of ['pulses','intervals','kinematics'])fs.writeFileSync(path.join(outDir,`wheel_work_${kind}.csv`),'\ufeff'+work.csv(report[kind],report));
console.log(work.describe(report));
