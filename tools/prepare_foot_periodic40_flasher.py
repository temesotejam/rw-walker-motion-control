"""Build the dedicated v38 installer from the compiled application."""
import json
import prepare_foot_fast_flasher as base

PROFILE = dict(environment='atoms3cam-foot-periodic40',identity='walking-foot-periodic40-v38',
               suffix='foot-periodic40',title='歩行器 v38 足角度40 Hz版',name='RW Walker v38 periodic foot state',
               description='<p>本体は25 ms周期で足角度を推定し、画面は約2秒ごとに更新します。書き込みにはUSBのみで給電してください。</p>')

if __name__=='__main__':
    print(json.dumps(base.prepare(profile=PROFILE), indent=2))
