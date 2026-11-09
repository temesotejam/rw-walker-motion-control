#!/usr/bin/env python3
"""Gate that the interrupt transport and IDF completion path survived linking.
Runtime bus ownership/probe/error behavior is covered separately by host mocks;
an ELF cannot establish electrical communication or achieved hardware latency.
"""
import argparse, subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('elf',type=Path);p.add_argument('--nm',required=True)
a=p.parse_args()
symbols=subprocess.check_output([a.nm,'-C',str(a.elf)],text=True)
for name in ['imu_i2c::begin(', 'imu_i2c::read(', 'i2c_master_write_read_device',
             'i2c_master_cmd_begin', 'i2c_isr_handler_default', 'xQueueReceive',
             'bmi270_timing::transport()::fn', 'ExperimentRunner::finishDeferredComparison()']:
    assert name in symbols, 'Missing actual linked transport/comparison symbol: '+name
print('ELF interrupt I2C transport, completion ISR/queue, shared BMI transport slot and deferred comparison present')
