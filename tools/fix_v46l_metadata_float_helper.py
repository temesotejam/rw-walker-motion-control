from pathlib import Path

path = Path("src/psram_logger.cpp")
text = path.read_text(encoding="utf-8")

helper = '''String jsonFloatOrNull(float value, unsigned int decimals) {
  return isfinite(value) ? String(value, decimals) : String("null");
}
'''

if helper not in text:
    marker = "namespace {\n"
    if text.count(marker) != 1:
        raise RuntimeError(f"psram_logger.cpp: expected one anonymous namespace marker, got {text.count(marker)}")
    text = text.replace(marker, marker + helper, 1)

path.write_text(text, encoding="utf-8")
print("Added V46l metadata float/null helper")
