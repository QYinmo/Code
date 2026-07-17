#include "gimbal_stream_parser.h"

#include <string.h>

static void GimbalStreamParser_DiscardFirst(GimbalStreamParser *parser)
{
    if (parser->length == 0U) {
        return;
    }
    if (parser->length > 1U) {
        (void)memmove(&parser->buffer[0], &parser->buffer[1],
                      parser->length - 1U);
    }
    --parser->length;
    ++parser->stats.discarded_bytes;
}

static void GimbalStreamParser_AlignHeader(GimbalStreamParser *parser)
{
    while (parser->length > 0U) {
        if (parser->buffer[0] != 0x5AU) {
            GimbalStreamParser_DiscardFirst(parser);
            continue;
        }
        if (parser->length >= 2U && parser->buffer[1] != 0xA5U) {
            GimbalStreamParser_DiscardFirst(parser);
            continue;
        }
        break;
    }
}

static bool GimbalStreamParser_HasCrcError(
    const uint8_t frame[GIMBAL_PACKET_LENGTH])
{
    uint16_t received_crc;
    if (frame[3] > (uint8_t)GIMBAL_MODE_FAULT) {
        return false;
    }
    received_crc = GimbalProtocol_ReadU16Le(&frame[8]);
    return received_crc != GimbalProtocol_Crc16Modbus(frame, 8U);
}

void GimbalStreamParser_Init(GimbalStreamParser *parser)
{
    if (parser == NULL) {
        return;
    }
    (void)memset(parser, 0, sizeof(*parser));
}

void GimbalStreamParser_Reset(GimbalStreamParser *parser)
{
    GimbalStreamParser_Init(parser);
}

size_t GimbalStreamParser_Push(
    GimbalStreamParser *parser, const uint8_t *data, size_t length,
    GimbalStreamFrameCallback callback, void *context)
{
    size_t index;
    size_t produced = 0U;
    if (parser == NULL || (data == NULL && length != 0U)) {
        return 0U;
    }
    for (index = 0U; index < length; ++index) {
        GimbalDecodedPacket decoded;
        ++parser->stats.input_bytes;
        if (parser->length >= GIMBAL_PACKET_LENGTH) {
            GimbalStreamParser_DiscardFirst(parser);
        }
        parser->buffer[parser->length] = data[index];
        ++parser->length;
        GimbalStreamParser_AlignHeader(parser);

        if (parser->length != GIMBAL_PACKET_LENGTH) {
            continue;
        }
        if (GimbalProtocol_Parse(parser->buffer, &decoded)) {
            const uint8_t expected =
                (uint8_t)(parser->last_sequence + 1U);
            ++parser->stats.valid_frames;
            if (parser->has_last_sequence && decoded.sequence != expected) {
                ++parser->stats.sequence_jumps;
            }
            parser->has_last_sequence = true;
            parser->last_sequence = decoded.sequence;
            if (callback != NULL) {
                callback(parser->buffer, &decoded, context);
            }
            parser->length = 0U;
            ++produced;
            continue;
        }

        ++parser->stats.candidate_errors;
        if (GimbalStreamParser_HasCrcError(parser->buffer)) {
            ++parser->stats.crc_errors;
        }
        /*
         * 候选帧无效时只前移一个字节，再重新滑动搜索 5A A5。
         * 这样可以保留候选帧内部或紧随其后的潜在新包头。
         */
        GimbalStreamParser_DiscardFirst(parser);
        GimbalStreamParser_AlignHeader(parser);
    }
    return produced;
}

size_t GimbalStreamParser_PushByte(
    GimbalStreamParser *parser, uint8_t byte,
    GimbalStreamFrameCallback callback, void *context)
{
    return GimbalStreamParser_Push(parser, &byte, 1U, callback, context);
}

const GimbalStreamParserStats *GimbalStreamParser_GetStats(
    const GimbalStreamParser *parser)
{
    return parser != NULL ? &parser->stats : NULL;
}
