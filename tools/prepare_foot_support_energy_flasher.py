"""Prepare the separately identified v14 energy-domain walking trial installer."""
import json
import shutil
from pathlib import Path
root=Path(__file__).resolve().parents[1]
build=root/'.pio/build/atoms3cam-foot-support-energy'
identity='foot-support-energy-v14'
firmware=build/'firmware.bin'
if not firmware.is_file() or identity.encode() not in firmware.read_bytes():
    raise SystemExit('Build atoms3cam-foot-support-energy successfully first.')
files={name:build/name for name in ['bootloader.bin','partitions.bin','firmware.bin']}
files['boot_app0.bin']=Path.home()/'.platformio/packages/framework-arduinoespressif32@3.20016.0/tools/partitions/boot_app0.bin'
if any(not p.is_file() for p in files.values()):raise SystemExit('Missing binary part')
dest=root/'flasher/foot-support-energy'
(dest/'firmware').mkdir(parents=True,exist_ok=True)
for name,path in files.items():shutil.copyfile(path,dest/'firmware'/name)
for name in ['flasher-completion.js','soft-boot.js']:shutil.copyfile(root/'flasher'/name,dest/name)
page=(root/'flasher/index.html').read_text(encoding='utf8').replace('angle-early-weak-zero-v2',identity)
(dest/'index.html').write_text(page,encoding='utf8')
manifest=json.loads((root/'flasher/manifest.json').read_text(encoding='utf8'))
manifest['name']='RW Walker energy-domain walking trial'
manifest['version']=identity
(dest/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
print('Prepared http://127.0.0.1:8767/foot-support-energy/')
