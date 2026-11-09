#!/usr/bin/env python3
"""Keep optional audit storage in PSRAM, allocated once before any run."""
from pathlib import Path
R=Path(__file__).resolve().parents[1]
def rd(p):return (R/p).read_text()
def wr(p,s):(R/p).write_text(s)
def once(s,a,b):
    assert s.count(a)==1,(a[:100],s.count(a))
    return s.replace(a,b,1)
h=rd('src/psram_logger.h')
h=once(h,'void addSolverAuditEvent(const solver_audit::Record& event) { solver_audit_.push(event); }','void addSolverAuditEvent(const solver_audit::Record& event) { if (solver_audit_) solver_audit_->push(event); }')
h=once(h,'solver_audit::Buffer<128> solver_audit_;','solver_audit::Buffer<128>* solver_audit_ = nullptr;')
wr('src/psram_logger.h',h)
s=rd('src/psram_logger.cpp')
s=once(s,'#include <string.h>','#include <string.h>\n#include <new>')
s=once(s,'  clear();\n  ready_ = true;', '''  // Optional diagnostics must not consume internal task/driver RAM. No
  // allocation occurs during a run. Failure is explicit in the export and
  // never changes the existing controller, IMU guards, or actuator authority.
  if (!solver_audit_) {
    void* audit_storage = ps_malloc(sizeof(solver_audit::Buffer<128>));
    if (audit_storage) solver_audit_ = new (audit_storage) solver_audit::Buffer<128>();
  }
  clear();
  ready_ = true;''')
assert s.count('  solver_audit_.clear();')==2
s=s.replace('  solver_audit_.clear();','  if (solver_audit_) solver_audit_->clear();')
s=once(s,'  solver_audit_.appendJson(json);','''  if (solver_audit_) solver_audit_->appendJson(json);
  else json += "{\\"schema_version\\":1,\\"available\\":false,\\"reason\\":\\"audit_psram_allocation_failed\\"}";''')
wr('src/psram_logger.cpp',s)
p='src/solver_audit.h';s=rd(p)
s=once(s,'{\\"schema_version\\":1,\\"solver_revision\\"','{\\"schema_version\\":1,\\"available\\":true,\\"solver_revision\\"')
wr(p,s)
p='tools/replay_v46s_solver_audit.py';s=rd(p)
s=once(s,"    if not audit:raise ValueError('no V46s audit in this file; earlier firmware cannot supply missing diagnostics')", "    if not audit:raise ValueError('no V46s audit in this file; earlier firmware cannot supply missing diagnostics')\n    if audit.get('available') is False:raise ValueError('audit unavailable: '+str(audit.get('reason', 'unknown')))")
wr(p,s)
p='tools/test_v46s_solver_audit.py';s=rd(p)
s=s.replace("logger.count('solver_audit_.clear();')", "logger.count('solver_audit_->clear();')")
s=s.replace("'solver_audit_.appendJson(json)'", "'solver_audit_->appendJson(json)'")
s=once(s,"    assert 'solver_audit_->appendJson(json)' in logger", "    assert 'solver_audit_->appendJson(json)' in logger\n    assert 'ps_malloc(sizeof(solver_audit::Buffer<128>))' in logger\n    assert 'solver_audit::Buffer<128>* solver_audit_ = nullptr;' in (ROOT/'src/psram_logger.h').read_text()\n    assert 'audit_psram_allocation_failed' in logger")
s=once(s,"        print('PASS: wrong width, lost records, failed decisions, empty runs and old logs cannot report a full pass')", "        try:analyze({'v46s_solver_audit':{'available':False,'reason':'audit_psram_allocation_failed'}},binary)\n        except ValueError:pass\n        else:raise AssertionError('unavailable audit incorrectly accepted')\n        print('PASS: wrong width, lost records, failed decisions, empty runs, allocation failure and old logs cannot report a full pass')")
wr(p,s)
p='docs/V46S_SOLVER_AUDIT.md';s=rd(p)
s+='\n## 保存領域\n\n監査リング約21 KBは起動時にPSRAMへ1回だけ確保します。内部RAMの固定使用量を増やしません（ポインタ等の少量を除く）。制御中の再確保はありません。確保できない場合は既存の制御動作を変えず、メタデータに`available: false`と`audit_psram_allocation_failed`を出します。そのログを比較成功とは扱いません。新しい記録の実機負荷は未確認です。\n'
wr(p,s)
print('V46s audit buffer moved to startup-allocated PSRAM; no runtime allocation')
