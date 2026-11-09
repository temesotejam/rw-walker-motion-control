#pragma once

#include <Arduino.h>
#include <stddef.h>

void rwlogDownloadDiagBegin(uint32_t expected_bytes, uint32_t metadata_bytes,
                            uint32_t sample_bytes);
void rwlogDownloadDiagSetPhase(const char* phase);
void rwlogDownloadDiagRequest(size_t requested_bytes, size_t chunk_limit);
void rwlogDownloadDiagProgress(size_t written_bytes);
void rwlogDownloadDiagRetry(int error_code);
void rwlogDownloadDiagShrink(size_t new_chunk_limit);
void rwlogDownloadDiagFail(const char* reason, int error_code);
void rwlogDownloadDiagFinish(bool ok);
String rwlogDownloadDiagnosticsJson();
