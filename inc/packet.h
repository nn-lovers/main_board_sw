#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 4)
typedef struct {
  uint8_t led_pattern;
  bool emergency_stop;
} DownlinkPacket;

typedef struct {
  uint16_t abs_enc;
  uint8_t limit_sw;
} UplinkPacket;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif