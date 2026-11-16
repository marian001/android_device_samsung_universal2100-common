//
// SPDX-FileCopyrightText: The LineageOS Project
// SPDX-License-Identifier: Apache-2.0
//

// Restores the Galaxy S21 ("tsds2") SIM2 <-> eSIM slot switch in the
// A556 RIL we ship.
//
// On tsds2 devices the second SIM tray and the embedded eUICC share one
// SIM interface. One UI (TelephonyUI SlotMappingManager) switches between
// them with the plain AOSP API:
//
//   eSIM: setSimSlotMapping([{port 0, phys 0, log 0}, {port 0, phys 2, log 1}])
//   pSIM: setSimSlotMapping([{port 0, phys 0, log 0}, {port 0, phys 1, log 1}])
//
// The stock S21 libsec-ril (G991BXXSJHZC2) catches these two requests in
// SimManager::DoSetSlotMapping, stores the target (1 = pSIM, 2 = eSIM) at
// SimManager+0x1668 and starts SimManager::ExecuteSlotSwitch(req, 0x21).
// OnSimLowLevelControlDone then walks the modem through
// 0x21 -> 0x10 (eSIM) / 0x11 (pSIM) -> 0x20 and answers the original
// request. The A556 RIL still contains ExecuteSlotSwitch and the whole
// OnSimLowLevelControlDone state machine, but its DoSetSlotMapping only
// knows the A35/A55 MEP layout and forwards physical slot 2 to the modem as
// a normal slot mapping, which the S21 modem rejects (INTERNAL_ERR).
//
// This library is linked with -z global (DF_1_GLOBAL) and loaded as a
// DT_NEEDED of the libsec-ril shim, i.e. before libsec-ril-impl.so is
// dlopen()ed. The impl calls SimManager::DoSetSlotMapping through its PLT,
// so the definition below takes precedence; everything that is not a tsds2
// switch request is handed to the real implementation unchanged.

#include <dlfcn.h>
#include <stddef.h>
#include <sys/system_properties.h>

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include <log/log.h>

#undef LOG_TAG
#define LOG_TAG "sec-ril-slotswitch"

namespace {

constexpr char kRealLib[] = "/vendor/lib64/libsec-ril-impl.so";
constexpr char kDoSetSlotMapping[] = "_ZN10SimManager16DoSetSlotMappingEP7Request";
constexpr char kExecuteSlotSwitch[] = "_ZN10SimManager17ExecuteSlotSwitchEP7Requesti";
// static SimManager* SimManager::mInstance[2] (16 bytes, one per RIL slot).
constexpr char kSimManagerInstances[] = "_ZN10SimManager9mInstanceE";
constexpr int kEsimSlot = 1;  // tsds2: the eUICC sits behind the second SIM interface

// Offsets taken from both RILs (G991B HZC2 stock and A556EXXSABYI4).
constexpr size_t kRequestDataOffset = 0x38;      // Request::mData
constexpr size_t kSlotMapCountOffset = 0x0c;     // slot map: int count
constexpr size_t kSlotMapEntriesOffset = 0x10;   // slot map: {int phy, int port}[]
constexpr size_t kSimManagerSlotOffset = 0x14;   // SimManager: RIL socket/slot id
constexpr size_t kSimManagerTargetOffset = 0x1668;  // SimManager: switch target

constexpr int kTargetPsim = 1;
constexpr int kTargetEsim = 2;
constexpr int kActionStartSwitch = 0x21;

using DoSetSlotMappingFn = int (*)(void*, void*);
using ExecuteSlotSwitchFn = void (*)(void*, void*, int);

DoSetSlotMappingFn gRealDoSetSlotMapping = nullptr;
ExecuteSlotSwitchFn gExecuteSlotSwitch = nullptr;
void** gSimManagers = nullptr;

bool resolveReal() {
    if (gRealDoSetSlotMapping != nullptr && gExecuteSlotSwitch != nullptr) {
        return true;
    }

    // The impl is already loaded (it is the caller). dlsym() on its handle
    // searches the impl first, so this returns its own definitions and not
    // the interposer below.
    void* handle = dlopen(kRealLib, RTLD_NOW | RTLD_NOLOAD);
    if (handle == nullptr) {
        ALOGE("dlopen(%s, RTLD_NOLOAD) failed: %s", kRealLib, dlerror());
        return false;
    }

    gRealDoSetSlotMapping =
            reinterpret_cast<DoSetSlotMappingFn>(dlsym(handle, kDoSetSlotMapping));
    gExecuteSlotSwitch = reinterpret_cast<ExecuteSlotSwitchFn>(dlsym(handle, kExecuteSlotSwitch));
    gSimManagers = reinterpret_cast<void**>(dlsym(handle, kSimManagerInstances));
    if (gRealDoSetSlotMapping == nullptr || gExecuteSlotSwitch == nullptr) {
        ALOGE("dlsym failed (DoSetSlotMapping=%p ExecuteSlotSwitch=%p)",
              reinterpret_cast<void*>(gRealDoSetSlotMapping),
              reinterpret_cast<void*>(gExecuteSlotSwitch));
        return false;
    }

    return true;
}

int getPropInt(const char* name, int def) {
    char value[PROP_VALUE_MAX] = {};
    if (__system_property_get(name, value) <= 0) {
        return def;
    }
    char* end = nullptr;
    long parsed = strtol(value, &end, 10);
    return end == value ? def : static_cast<int>(parsed);
}

bool isTsds2() {
    char value[PROP_VALUE_MAX] = {};
    if (__system_property_get("persist.radio.esim.slotswitch", value) <= 0) {
        return false;
    }
    return strcmp(value, "tsds2") == 0;
}

template <typename T>
T* at(void* base, size_t offset) {
    return reinterpret_cast<T*>(static_cast<uint8_t*>(base) + offset);
}

// Returns kTargetPsim/kTargetEsim when the request is a One UI style tsds2
// switch request, 0 otherwise. Mirrors the stock G991B logic.
int getSwitchTarget(void* slotMap) {
    const int count = *at<int>(slotMap, kSlotMapCountOffset);
    const int* map = at<int>(slotMap, kSlotMapEntriesOffset);

    ALOGI("DoSetSlotMapping: count %d", count);
    for (int i = 0; i < count && i < 4; i++) {
        ALOGI("  logical %d -> phy %d port %d", i, map[2 * i], map[2 * i + 1]);
    }

    if (count != 2 || map[0] != 0 || map[1] != 0 || map[3] != 0) {
        return 0;
    }

    const bool esimActive =
            getPropInt("ril.simslottype1", 0) == 1 || getPropInt("ril.simslottype2", 0) == 1;
    if (!esimActive && map[2] == 2) {
        return kTargetEsim;
    }
    if (esimActive && map[2] == 1) {
        return kTargetPsim;
    }
    return 0;
}


// ---- Synthetic slot status (GET_SLOT_STATUS) --------------------------------
// The A556 RIL answers GET_SLOT_STATUS from SimManager::mSlotStatusList, which is only
// filled by the modem's unsolicited slot status message. On the S21+ (t2s) that list stays
// empty after boot, DoGetSlotStatus answers INTERNAL_ERR, Android sees no slot mapping at
// all ("0 logical slots") and neither OpenEUICC nor Settings can offer the eSIM. In tsds2
// mode we build the list the stock S21 modem would report: three physical slots
// (0 = SIM1 tray, 1 = SIM2 tray, 2 = soldered eUICC) with the SIM2 tray mapped to
// logical slot 1. The first real notification replaces it as usual. All calls go through
// the impl's own exported constructors, so the layouts always match the blob.
constexpr char kDoGetSlotStatus[] = "_ZN10SimManager15DoGetSlotStatusEP7Request";
constexpr char kSlotStatusListCtor[] = "_ZN14SlotStatusListC1Ev";
constexpr char kSlotStatusListAdd[] = "_ZN14SlotStatusList13addSlotStatusEP10SlotStatus";
constexpr char kSlotStatusCtor[] = "_ZN10SlotStatusC1E13SlotCardState7MepModeP6SimAtrP6SimEid";
constexpr char kSlotStatusAddPort[] = "_ZN10SlotStatus11addPortInfoEP8PortInfo";
constexpr char kPortInfoCtor[] = "_ZN8PortInfoC1Ei9PortStateiP8SimICCID";
constexpr char kSimAtrCtor[] = "_ZN6SimAtrC1EPhi";
constexpr char kSimEidCtor[] = "_ZN6SimEidC1EPhi";
constexpr char kSimIccidCtor[] = "_ZN8SimICCIDC1Ev";
constexpr char kSlotStatusListGlobal[] = "_ZN10SimManager15mSlotStatusListE";
constexpr char kSlotStatusLockGlobal[] = "_ZN10SimManager15mLockSlotStatusE";

// Object sizes (operator new() sizes seen in IpcRxSlotStatus of the same blob).
constexpr size_t kSizeSlotStatusList = 0x18;
constexpr size_t kSizeSlotStatus = 0x38;
constexpr size_t kSizePortInfo = 0x20;
constexpr size_t kSizeSimBlob = 0x110;  // SimAtr, SimEid, SimICCID
constexpr size_t kListInnerOffset = 0x10;   // SlotStatusList: List*
constexpr size_t kListCountOffset = 0x18;   // List: int count

constexpr int kCardAbsent = 0;
constexpr int kCardPresent = 1;
constexpr int kEidFileBytes = 16;

using DoGetSlotStatusFn = int (*)(void*, void*);
using ListCtorFn = void (*)(void*);
using ListAddFn = int (*)(void*, void*);
using SlotCtorFn = void (*)(void*, int, int, void*, void*);
using SlotAddPortFn = int (*)(void*, void*);
using PortCtorFn = void (*)(void*, int, int, int, void*);
using SimBytesCtorFn = void (*)(void*, unsigned char*, int);
using SimIccidCtorFn = void (*)(void*);

DoGetSlotStatusFn gRealDoGetSlotStatus = nullptr;
ListCtorFn gListCtor = nullptr;
ListAddFn gListAdd = nullptr;
SlotCtorFn gSlotCtor = nullptr;
SlotAddPortFn gSlotAddPort = nullptr;
PortCtorFn gPortCtor = nullptr;
SimBytesCtorFn gAtrCtor = nullptr;
SimBytesCtorFn gEidCtor = nullptr;
SimIccidCtorFn gIccidCtor = nullptr;
void** gSlotListGlobal = nullptr;
pthread_mutex_t* gSlotLock = nullptr;
bool gSynthResolved = false;
bool gSynthOk = false;

bool resolveSynth() {
    if (gSynthResolved) return gSynthOk;
    gSynthResolved = true;
    void* h = dlopen(kRealLib, RTLD_NOW | RTLD_NOLOAD);
    if (h == nullptr) {
        ALOGE("synth: dlopen(%s, RTLD_NOLOAD) failed: %s", kRealLib, dlerror());
        return false;
    }
    gRealDoGetSlotStatus = reinterpret_cast<DoGetSlotStatusFn>(dlsym(h, kDoGetSlotStatus));
    gListCtor = reinterpret_cast<ListCtorFn>(dlsym(h, kSlotStatusListCtor));
    gListAdd = reinterpret_cast<ListAddFn>(dlsym(h, kSlotStatusListAdd));
    gSlotCtor = reinterpret_cast<SlotCtorFn>(dlsym(h, kSlotStatusCtor));
    gSlotAddPort = reinterpret_cast<SlotAddPortFn>(dlsym(h, kSlotStatusAddPort));
    gPortCtor = reinterpret_cast<PortCtorFn>(dlsym(h, kPortInfoCtor));
    gAtrCtor = reinterpret_cast<SimBytesCtorFn>(dlsym(h, kSimAtrCtor));
    gEidCtor = reinterpret_cast<SimBytesCtorFn>(dlsym(h, kSimEidCtor));
    gIccidCtor = reinterpret_cast<SimIccidCtorFn>(dlsym(h, kSimIccidCtor));
    gSlotListGlobal = reinterpret_cast<void**>(dlsym(h, kSlotStatusListGlobal));
    gSlotLock = reinterpret_cast<pthread_mutex_t*>(dlsym(h, kSlotStatusLockGlobal));
    gSynthOk = gRealDoGetSlotStatus && gListCtor && gListAdd && gSlotCtor && gSlotAddPort &&
               gPortCtor && gAtrCtor && gEidCtor && gIccidCtor && gSlotListGlobal && gSlotLock;
    if (!gSynthOk) {
        ALOGE("synth: dlsym failed (get=%p list=%p add=%p slot=%p port=%p port2=%p atr=%p eid=%p "
              "iccid=%p glob=%p lock=%p)",
              (void*)gRealDoGetSlotStatus, (void*)gListCtor, (void*)gListAdd, (void*)gSlotCtor,
              (void*)gSlotAddPort, (void*)gPortCtor, (void*)gAtrCtor, (void*)gEidCtor,
              (void*)gIccidCtor, (void*)gSlotListGlobal, (void*)gSlotLock);
    }
    return gSynthOk;
}

bool slotListEmpty() {
    void* list = *gSlotListGlobal;
    if (list == nullptr) return true;
    void* inner = *at<void*>(list, kListInnerOffset);
    if (inner == nullptr) return true;
    return *at<int>(inner, kListCountOffset) <= 0;
}

// EID persisted by the RIL itself (SimManager::WriteEidFile) after the first eSIM selection.
void loadEid(unsigned char out[kEidFileBytes]) {
    memset(out, 0, kEidFileBytes);
    // Prefix of the NXP eUICC in the S21 family; a placeholder until the real EID is known.
    static const unsigned char kPrefix[4] = {0x89, 0x04, 0x30, 0x51};
    memcpy(out, kPrefix, sizeof(kPrefix));
    FILE* f = fopen("/efs/FactoryApp/eID", "r");
    if (f == nullptr) return;
    char hex[64] = {};
    size_t n = fread(hex, 1, sizeof(hex) - 1, f);
    fclose(f);
    if (n < 2 * kEidFileBytes) return;
    unsigned char tmp[kEidFileBytes];
    for (int i = 0; i < kEidFileBytes; i++) {
        unsigned int v;
        char pair[3] = {hex[2 * i], hex[2 * i + 1], 0};
        char* end = nullptr;
        v = static_cast<unsigned int>(strtoul(pair, &end, 16));
        if (end == pair || *end != 0) return;
        tmp[i] = static_cast<unsigned char>(v);
    }
    memcpy(out, tmp, kEidFileBytes);
}

using OperatorNewFn = void* (*)(size_t);
OperatorNewFn gOperatorNew = nullptr;

// Objects are later destroyed by the RIL with operator delete, so allocate with the same
// operator new the RIL uses (resolved from the libc++ the impl is linked against).
void* newObj(size_t size) {
    if (gOperatorNew == nullptr) {
        gOperatorNew = reinterpret_cast<OperatorNewFn>(dlsym(RTLD_DEFAULT, "_Znwm"));
    }
    void* p = gOperatorNew != nullptr ? gOperatorNew(size) : malloc(size);
    memset(p, 0, size);
    return p;
}

// Builds {SIM1 tray, SIM2 tray, eUICC} as the stock S21 modem reports them.
void* buildSyntheticSlotList() {
    unsigned char eid[kEidFileBytes];
    loadEid(eid);
    unsigned char noAtr[1] = {0};

    void* list = newObj(kSizeSlotStatusList);
    gListCtor(list);

    for (int phys = 0; phys < 3; phys++) {
        void* atr = newObj(kSizeSimBlob);
        gAtrCtor(atr, noAtr, 0);
        void* eidObj = newObj(kSizeSimBlob);
        if (phys == 2) {
            gEidCtor(eidObj, eid, kEidFileBytes);
        } else {
            gEidCtor(eidObj, noAtr, 0);
        }
        void* slot = newObj(kSizeSlotStatus);
        // The eUICC is soldered: always present. Tray state is unknown until the real
        // notification arrives, report them absent (mapping below is what matters).
        gSlotCtor(slot, phys == 2 ? kCardPresent : kCardAbsent, 0, atr, eidObj);

        // Mapping: logical 0 = SIM1 tray, logical 1 = SIM2 tray; the eUICC starts unmapped.
        // (If the eSIM were already selected the modem has reported a real list.)
        const bool esimActive = getPropInt("ril.simslottype1", 0) == 1 ||
                                getPropInt("ril.simslottype2", 0) == 1;
        const int logical = phys == 0 ? 0 : (phys == (esimActive ? 2 : 1) ? 1 : -1);
        void* iccid = newObj(kSizeSimBlob);
        gIccidCtor(iccid);
        void* port = newObj(kSizePortInfo);
        gPortCtor(port, 0, logical >= 0 ? 1 : 0, logical, iccid);  // (port, state, logical, iccid)
        gSlotAddPort(slot, port);
        gListAdd(list, slot);
    }
    return list;
}

void ensureSlotList() {
    // Disabled by default. Build #5 showed the 3-slot model is wrong for Android: the framework
    // only has two UiccSlots (the eUICC is reported in physical slot 1 once selected, as on
    // o1s), drops the third entry and marks logical slot 1 inactive -> no subscription, no
    // data. Kept for experiments only (setprop persist.vendor.radio.synth_slot_status 1).
    if (getPropInt("persist.vendor.radio.synth_slot_status", 0) != 1) return;
    if (!resolveSynth() || !isTsds2()) return;
    pthread_mutex_lock(gSlotLock);
    if (slotListEmpty()) {
        void* list = buildSyntheticSlotList();
        *gSlotListGlobal = list;
        ALOGI("GET_SLOT_STATUS: modem reported no slots, using synthetic tsds2 list (SIM1, SIM2, eUICC)");
    }
    pthread_mutex_unlock(gSlotLock);
}

}  // namespace

// int SimManager::DoSetSlotMapping(Request*)
extern "C" __attribute__((visibility("default"))) int _ZN10SimManager16DoSetSlotMappingEP7Request(
        void* self, void* request) {
    if (!resolveReal()) {
        abort();  // Cannot forward; rild restarts and the interposer is logged.
    }

    void* slotMap = request != nullptr ? *at<void*>(request, kRequestDataOffset) : nullptr;
    if (slotMap == nullptr || !isTsds2()) {
        return gRealDoSetSlotMapping(self, request);
    }

    const int target = getSwitchTarget(slotMap);
    if (target == 0) {
        return gRealDoSetSlotMapping(self, request);
    }

    ALOGI("tsds2 slot switch to %s (SimManager slot %d, simslottype %d/%d)",
          target == kTargetEsim ? "eSIM" : "pSIM", *at<int>(self, kSimManagerSlotOffset),
          getPropInt("ril.simslottype1", 0), getPropInt("ril.simslottype2", 0));

    // Same as stock: remember the target, start the low-level control
    // sequence; OnSimLowLevelControlDone answers the request when done.
    // The modem applies the switch to the SIM interface of the RIL instance the
    // command is sent on, so it must run on the second slot's SimManager.
    void* mgr = (gSimManagers != nullptr) ? gSimManagers[kEsimSlot] : nullptr;
    if (mgr == nullptr) {
        ALOGE("SimManager instance for slot %d not found (table %p); refusing switch",
              kEsimSlot, reinterpret_cast<void*>(gSimManagers));
        return gRealDoSetSlotMapping(self, request);
    }
    ALOGI("using SimManager %p (slot field %d) instead of caller %p (slot field %d)", mgr,
          *at<int>(mgr, kSimManagerSlotOffset), self, *at<int>(self, kSimManagerSlotOffset));
    *at<int>(mgr, kSimManagerTargetOffset) = target;
    gExecuteSlotSwitch(mgr, request, kActionStartSwitch);
    return 0;
}

// int SimManager::DoGetSlotStatus(Request*)
extern "C" __attribute__((visibility("default"))) int _ZN10SimManager15DoGetSlotStatusEP7Request(
        void* self, void* request) {
    ensureSlotList();
    if (gRealDoGetSlotStatus == nullptr) {
        void* h = dlopen(kRealLib, RTLD_NOW | RTLD_NOLOAD);
        if (h != nullptr) {
            gRealDoGetSlotStatus = reinterpret_cast<DoGetSlotStatusFn>(dlsym(h, kDoGetSlotStatus));
        }
        if (gRealDoGetSlotStatus == nullptr) {
            abort();  // Cannot forward; rild restarts and the interposer is logged.
        }
    }
    return gRealDoGetSlotStatus(self, request);
}
