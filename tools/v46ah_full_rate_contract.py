"""Reverse only the V46ah revision label for retained historical hash gates.

The changed rate-baseline behavior is exercised directly by the production
helper test. No old checksum or V46ag integration delta is changed here.
"""

def normalize_v46ah(text: str, path: str) -> str:
    from v46ai_rate_only_contract import normalize_v46ai
    text = normalize_v46ai(text, path)
    if path != 'src/config.h':
        return text
    old = 'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46ag_rate_baseline_20260919";'
    new = 'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46ah_full_rate_baseline_20260919";'
    if text.count(new) == 1:
        return text.replace(new, old, 1)
    if text.count(old) != 1:
        raise ValueError('V46ah revision delta changed')
    return text
