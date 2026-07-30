/*
 * vwifi — tlv_mem.cpp
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Overloaded operator new/delete for the WDI TLV generator/parser
 * library.
 *
 * This file exists because the library internally uses C++ new/delete
 * and requires consumers to supply the operators when linking. Per
 * Microsoft's "WDI TLV generator/parser memory interface": "The parser
 * and generator internally use C++ with new/delete. This simplifies
 * several implementation details. This means that consumers of the
 * library must provide overloaded operator implementations of these
 * APIs when linking to the library. This is the only C++ dependency
 * that your code must take."
 *
 * The AllocationContext field of TLV_CONTEXT is passed through to the
 * new operator unmodified, letting us customise allocation per call
 * site. We don't currently need that, so it's ignored — but the
 * parameter must exist for the library to link.
 *
 * NonPagedPoolNx is required: TLV parse/generate happens at DISPATCH
 * level in our DPC paths (the ctrl-rsp drain), so the memory must
 * never be paged, and NX because it's data.
 */

extern "C" {
#include <ntddk.h>
}

#define VWIFI_TLV_POOL_TAG  (ULONG)'vlTv'   /* "vTlv" in WinDbg */

void * __cdecl operator new(size_t Size, ULONG_PTR AllocationContext)
{
    UNREFERENCED_PARAMETER(AllocationContext);

    PVOID p = ExAllocatePool2(POOL_FLAG_NON_PAGED, Size, VWIFI_TLV_POOL_TAG);
    /* ExAllocatePool2 zeroes by default; older kits without it should
     * use ExAllocatePoolWithTag(NonPagedPoolNx, ...) + RtlZeroMemory.
     * The library relies on zero-initialised memory either way. */
    return p;
}

void __cdecl operator delete(void *p)
{
    if (p != nullptr) {
        ExFreePoolWithTag(p, VWIFI_TLV_POOL_TAG);
    }
}

/* Sized delete — C++14 emits calls to this form. Without it the link
 * fails with an unresolved external on newer toolchains. */
void __cdecl operator delete(void *p, size_t Size)
{
    UNREFERENCED_PARAMETER(Size);
    if (p != nullptr) {
        ExFreePoolWithTag(p, VWIFI_TLV_POOL_TAG);
    }
}

/* Array forms, for completeness — the library shouldn't need them,
 * but an unresolved external here is a confusing link error to debug. */
void * __cdecl operator new[](size_t Size, ULONG_PTR AllocationContext)
{
    UNREFERENCED_PARAMETER(AllocationContext);
    return ExAllocatePool2(POOL_FLAG_NON_PAGED, Size, VWIFI_TLV_POOL_TAG);
}

void __cdecl operator delete[](void *p)
{
    if (p != nullptr) {
        ExFreePoolWithTag(p, VWIFI_TLV_POOL_TAG);
    }
}

void __cdecl operator delete[](void *p, size_t Size)
{
    UNREFERENCED_PARAMETER(Size);
    if (p != nullptr) {
        ExFreePoolWithTag(p, VWIFI_TLV_POOL_TAG);
    }
}
