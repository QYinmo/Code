#ifndef GIMBAL_STREAM_PARSER_H
#define GIMBAL_STREAM_PARSER_H

#include "gimbal_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t input_bytes;
    uint64_t valid_frames;
    uint64_t candidate_errors;
    uint64_t crc_errors;
    uint64_t discarded_bytes;
    uint64_t sequence_jumps;
} GimbalStreamParserStats;

typedef void (*GimbalStreamFrameCallback)(
    const uint8_t frame[GIMBAL_PACKET_LENGTH],
    const GimbalDecodedPacket *packet, void *context);

typedef struct {
    uint8_t buffer[GIMBAL_PACKET_LENGTH];
    size_t length;
    bool has_last_sequence;
    uint8_t last_sequence;
    GimbalStreamParserStats stats;
} GimbalStreamParser;

void GimbalStreamParser_Init(GimbalStreamParser *parser);
void GimbalStreamParser_Reset(GimbalStreamParser *parser);
size_t GimbalStreamParser_Push(
    GimbalStreamParser *parser, const uint8_t *data, size_t length,
    GimbalStreamFrameCallback callback, void *context);
size_t GimbalStreamParser_PushByte(
    GimbalStreamParser *parser, uint8_t byte,
    GimbalStreamFrameCallback callback, void *context);
const GimbalStreamParserStats *GimbalStreamParser_GetStats(
    const GimbalStreamParser *parser);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_STREAM_PARSER_H */
