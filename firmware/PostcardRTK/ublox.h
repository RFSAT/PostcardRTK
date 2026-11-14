// ublox.h - the few UBX (u-blox binary protocol) pieces the F9R firmware needs: frames, CFG-VALSET to configure the
// ZED-F9R at start-up, MON-VER for its firmware version, and a parser that picks UBX frames (ACK/NAK, MON-VER) out
// of a stream mixed with NMEA. Portable (no Arduino calls), unit-tested on the PC.
// Reference: u-blox ZED-F9R Interface Description (configuration keys CFG-xxx).
#pragma once
#include <cstddef>
#include <cstdint>

// One configuration item: key id and value. The key's size is in bits 28-30 of the id (1 = bit, 2 = 1 byte,
// 3 = 2 bytes, 4 = 4 bytes, 5 = 8 bytes).
struct UbxKey
{
    uint32_t id;
    uint32_t value;
    const char *name; // for the event log
};

// Whole UBX frame (sync, class, id, length, payload, checksum) into out; returns its length or 0 if it does not fit
size_t ubxFrame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len, uint8_t *out, size_t outSize);
// UBX-CFG-VALSET of one key, applied to RAM and battery-backed RAM (layers 0x03; not flash, so the receiver's
// saved configuration stays as it was). One key per message: a key the receiver does not know only NAKs itself.
size_t ubxValset(const UbxKey &k, uint8_t *out, size_t outSize);
// UBX-MON-VER poll (receiver answers with its software / firmware version)
size_t ubxMonVerPoll(uint8_t *out, size_t outSize);

// The configuration the firmware applies at every start-up (see ublox.cpp for the list)
extern const UbxKey UBX_SETUP[];
extern const int UBX_SETUP_COUNT;
// Key ids of the UART1 / UART2 baud rates (sent separately, last, because the link speed changes with them)
static const uint32_t UBX_KEY_UART1_BAUD = 0x40520001;
static const uint32_t UBX_KEY_UART2_BAUD = 0x40530001;

enum UbxEvent : uint8_t
{
    UBX_NONE = 0,
    UBX_ACK,    // CFG message accepted (ackedMsg() tells which)
    UBX_NAK,    // CFG message rejected
    UBX_MONVER, // version text in version()
    UBX_OTHER,
};

class UbxParser
{
  public:
    // Feed one byte; returns an event when a complete valid frame ends with it
    UbxEvent feed(uint8_t b);
    uint8_t ackedClass() const { return _ackCls; }
    uint8_t ackedId() const { return _ackId; }
    // "HPS 1.30 / PROTVER=33.30 / ZED-F9R" style summary of the last MON-VER (empty until one arrived)
    const char *version() const { return _ver; }
    uint32_t frames() const { return _frames; }
    uint32_t errors() const { return _errors; }

  private:
    void finish(UbxEvent &ev);
    int _state = 0;
    uint8_t _cls = 0, _id = 0;
    uint16_t _len = 0, _pos = 0;
    uint8_t _ckA = 0, _ckB = 0, _rxA = 0;
    uint8_t _buf[260]; // longer payloads (e.g. MON-VER with many extensions) are checked but only partly kept
    uint8_t _ackCls = 0, _ackId = 0;
    char _ver[120] = "";
    uint32_t _frames = 0, _errors = 0;
};
