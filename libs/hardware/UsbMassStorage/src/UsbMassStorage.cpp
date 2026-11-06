#include "UsbMassStorage.h"

#if FREEINK_CAP_USB_MSC

#include <Arduino.h>
#include <USB.h>
#include <USBMSC.h>

#include <algorithm>
#include <atomic>
#include <cstring>

extern "C" bool tud_mounted(void);
extern "C" bool tud_suspended(void);
extern "C" bool tud_disconnect(void);
extern "C" bool tud_suspended(void);

namespace freeink {
namespace {

USBMSC gMsc;
std::atomic<UsbMassStorage*> gOwner{nullptr};
std::atomic<FsBlockDeviceInterface*> gDev{nullptr};
constexpr uint16_t kBlockSize = 512;
uint8_t gSectorScratch[kBlockSize];
// Optional read-ahead window (see UsbMassStorage::begin). Only touched from the
// TinyUSB task (read/write callbacks) while a session is active; begin()/end() run
// with the callbacks quiesced.
uint8_t* gReadAhead = nullptr;
uint32_t gReadAheadSectors = 0;
uint32_t gReadAheadFirst = 0;  // first cached LBA
uint32_t gReadAheadValid = 0;  // cached sector count, 0 = empty
uint32_t gNextLba = 0;         // LBA that would continue the previous read

// Serve an aligned read from the window, refilling it from the card on a miss.
// Returns false when the request is not eligible (caller reads directly).
bool readViaWindow(FsBlockDeviceInterface* dev, const uint32_t lba, uint8_t* out, const uint32_t sectors) {
  if (!gReadAhead || sectors == 0 || sectors > gReadAheadSectors) return false;
  const bool sequential = lba == gNextLba;
  gNextLba = lba + sectors;
  if (!(gReadAheadValid != 0 && lba >= gReadAheadFirst && lba + sectors <= gReadAheadFirst + gReadAheadValid)) {
    // Random reads (FAT/directory lookups) should not pay for a 16 KB fill.
    if (!sequential) return false;
    const uint64_t total = dev->sectorCount();
    uint64_t fill = gReadAheadSectors;
    if (lba + fill > total) fill = total > lba ? total - lba : 0;
    gReadAheadValid = 0;
    if (fill < sectors || !dev->readSectors(lba, gReadAhead, static_cast<size_t>(fill))) return false;
    gReadAheadFirst = lba;
    gReadAheadValid = static_cast<uint32_t>(fill);
  }
  memcpy(out, gReadAhead + static_cast<size_t>(lba - gReadAheadFirst) * kBlockSize, sectors * kBlockSize);
  return true;
}

bool isRangeValid(FsBlockDeviceInterface* dev, const uint32_t lba, const uint32_t offset, const uint32_t bufsize) {
  if (!dev || bufsize == 0) return false;
  const uint64_t totalBytes = static_cast<uint64_t>(dev->sectorCount()) * kBlockSize;
  const uint64_t start = static_cast<uint64_t>(lba) * kBlockSize + offset;
  const uint64_t end = start + bufsize;
  return end >= start && end <= totalBytes;
}

int32_t mscRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  auto* const dev = gDev.load();
  auto* const owner = gOwner.load();
  if (!buffer || !isRangeValid(dev, lba, offset, bufsize)) {
    if (owner) owner->markIoError();
    return -1;
  }

  const uint64_t start = static_cast<uint64_t>(lba) * kBlockSize + offset;
  auto* output = static_cast<uint8_t*>(buffer);
  uint32_t remaining = bufsize;
  uint64_t cursor = start;

  if ((offset % kBlockSize) == 0 && (bufsize % kBlockSize) == 0) {
    const size_t sectors = bufsize / kBlockSize;
    const auto firstSector = static_cast<Sector_t>(start / kBlockSize);
    if (readViaWindow(dev, firstSector, output, static_cast<uint32_t>(sectors))) {
      // served from the read-ahead window
    } else if (!dev->readSectors(firstSector, output, sectors)) {
      if (owner) owner->markIoError();
      return -1;
    }
  } else {
    while (remaining > 0) {
      const auto sector = static_cast<Sector_t>(cursor / kBlockSize);
      const size_t within = static_cast<size_t>(cursor % kBlockSize);
      const size_t count = std::min<size_t>(kBlockSize - within, remaining);
      if (!dev->readSector(sector, gSectorScratch)) {
        if (owner) owner->markIoError();
        return -1;
      }
      memcpy(output, gSectorScratch + within, count);
      output += count;
      cursor += count;
      remaining -= count;
    }
  }

  if (owner) owner->markAccessed();
  return static_cast<int32_t>(bufsize);
}

int32_t mscWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
  auto* const dev = gDev.load();
  auto* const owner = gOwner.load();
  if (!buffer || !isRangeValid(dev, lba, offset, bufsize)) {
    if (owner) owner->markIoError();
    return -1;
  }

  gReadAheadValid = 0;  // any write may overlap the window
  const uint64_t start = static_cast<uint64_t>(lba) * kBlockSize + offset;
  auto* input = buffer;
  uint32_t remaining = bufsize;
  uint64_t cursor = start;

  if ((offset % kBlockSize) == 0 && (bufsize % kBlockSize) == 0) {
    const size_t sectors = bufsize / kBlockSize;
    if (!dev->writeSectors(static_cast<Sector_t>(start / kBlockSize), input, sectors)) {
      if (owner) owner->markIoError();
      return -1;
    }
  } else {
    while (remaining > 0) {
      const auto sector = static_cast<Sector_t>(cursor / kBlockSize);
      const size_t within = static_cast<size_t>(cursor % kBlockSize);
      const size_t count = std::min<size_t>(kBlockSize - within, remaining);
      if (!dev->readSector(sector, gSectorScratch)) {
        if (owner) owner->markIoError();
        return -1;
      }
      memcpy(gSectorScratch + within, input, count);
      if (!dev->writeSector(sector, gSectorScratch)) {
        if (owner) owner->markIoError();
        return -1;
      }
      input += count;
      cursor += count;
      remaining -= count;
    }
  }

  if (owner) owner->markAccessed();
  return static_cast<int32_t>(bufsize);
}

bool mscStartStop(uint8_t /*power_condition*/, bool start, bool load_eject) {
  if (load_eject && !start) {
    if (auto* const owner = gOwner.load()) owner->markEjected();
  }
  return true;
}

}  // namespace

bool UsbMassStorage::begin(FsBlockDeviceInterface* dev, uint8_t* readAhead, size_t readAheadSectors) {
  if (_active || !dev || dev->sectorCount() == 0) return false;
  gReadAhead = readAheadSectors > 1 ? readAhead : nullptr;
  gReadAheadSectors = gReadAhead ? static_cast<uint32_t>(readAheadSectors) : 0;
  gReadAheadValid = 0;

  gDev.store(dev);
  gOwner.store(this);
  _state.store(UsbMassStorageState::WaitingForHost);
  _hostSeen.store(false);
  gMsc.vendorID("FreeInk");
  gMsc.productID("SD Card");
  gMsc.productRevision("1.0");
  gMsc.mediaPresent(true);
  gMsc.isWritable(true);
  gMsc.onStartStop(mscStartStop);
  gMsc.onRead(mscRead);
  gMsc.onWrite(mscWrite);
  if (!gMsc.begin(dev->sectorCount(), kBlockSize)) {
    gMsc.end();
    gOwner.store(nullptr);
    gDev.store(nullptr);
    gReadAhead = nullptr;
    _state.store(UsbMassStorageState::Idle);
    return false;
  }
  if (!USB.begin()) {
    gMsc.end();
    gOwner.store(nullptr);
    gDev.store(nullptr);
    gReadAhead = nullptr;
    _state.store(UsbMassStorageState::Idle);
    return false;
  }
  _active = true;
  return true;
}

void UsbMassStorage::end() {
  if (!_active) return;
  // Signal the host to release the device BEFORE tearing down callback state.
  // tud_disconnect() disables the D+/D- pull-up so the host ejects the media;
  // we then wait for tud_mounted() to go false so no in-flight MSC callback
  // can fire after gMsc.end() zeroes the callback pointers or gDev/gOwner clear.
  // On the dual-core S3 the TinyUSB task runs on the other core — without this
  // drain, a callback can dereference null gDev/gOwner (or the zeroed msc_luns
  // function pointers) and hard-fault during the subsequent ESP.restart().
  tud_disconnect();
  for (uint32_t i = 0; i < 50; i++) {  // ~500ms timeout at 10ms ticks
    if (!tud_mounted()) break;
    delay(10);
  }
  // Mark inactive first so any callback already dispatched bails out instead
  // of dereferencing the globals that we clear below.
  _active = false;
  gOwner.store(nullptr);
  gDev.store(nullptr);
  gMsc.end();
  gReadAhead = nullptr;
  gReadAheadValid = 0;
  _state.store(UsbMassStorageState::Idle);
  _hostSeen.store(false);
}

UsbMassStorageState UsbMassStorage::state() const {
  if (!_active) return UsbMassStorageState::Idle;
  const auto current = _state.load();
  if (current == UsbMassStorageState::Ejected) return current;

  if (!tud_mounted()) {
    return _hostSeen.load() ? UsbMassStorageState::Disconnected : UsbMassStorageState::WaitingForHost;
  }

  // Keep the error visible while the host is mounted, but report a later cable
  // removal so the application can safely reclaim its raw storage session.
  if (current == UsbMassStorageState::IoError) return current;

  _hostSeen.store(true);
  auto expected = UsbMassStorageState::WaitingForHost;
  _state.compare_exchange_strong(expected, UsbMassStorageState::Connected);
  return _state.load();
}

bool UsbMassStorage::hostConnected() const {
  const auto current = state();
  return current == UsbMassStorageState::Connected || current == UsbMassStorageState::Accessed;
}

bool UsbMassStorage::disconnectHost() const { return _active && tud_disconnect(); }

bool UsbMassStorage::hostSuspended() const { return _active && tud_mounted() && tud_suspended(); }

void UsbMassStorage::markAccessed() const {
  auto current = _state.load();
  while (current != UsbMassStorageState::Ejected && current != UsbMassStorageState::IoError) {
    if (_state.compare_exchange_weak(current, UsbMassStorageState::Accessed)) return;
  }
}

void UsbMassStorage::markEjected() const { _state.store(UsbMassStorageState::Ejected); }

void UsbMassStorage::markIoError() const {
  _hostSeen.store(true);
  _state.store(UsbMassStorageState::IoError);
}

}  // namespace freeink

#endif  // FREEINK_CAP_USB_MSC
