#!/usr/bin/env python3
"""Reject flash-resident functions on the new timer ISR notification path.
Task/IRQ core observations remain an on-device check; an ELF is not timing proof.
"""
import argparse
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('elf')
parser.add_argument('--objdump', required=True)
args = parser.parse_args()
symbols = subprocess.check_output([args.objdump, '-t', '-C', args.elf], text=True)
for name in ('ImuManager::timerCallback(void*)', 'timer_isr_default',
             'esp_timer_get_time', 'vTaskGenericNotifyGiveFromISR'):
    rows = [line for line in symbols.splitlines() if line.endswith(' ' + name) or line.endswith('\t' + name)]
    assert rows and all('.iram0.text' in line for line in rows), 'Timer ISR path must be linked in IRAM: ' + name
for name in ('timer_isr_callback_add', 'timer_init', 'timer_start'):
    assert any(line.endswith(' ' + name) or line.endswith('\t' + name) for line in symbols.splitlines()), name
print('ELF: hardware timer callback, IDF ISR, clock and ISR notification are in IRAM; timer driver linked')
