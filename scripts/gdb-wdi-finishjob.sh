#!/usr/bin/env bash
#
# Break on wdiwifi!CScanJob::FinishJob from outside the guest and print
# the three values that decide whether the good-scan time is recorded.
#
# WHY THIS EXISTS
#
# A Windows guest cannot break on itself. Local kernel debugging (kd
# -kl) reads memory and disassembles happily, which is how the whole
# connect failure was traced, but it cannot stop the machine -- and the
# last question is a runtime one:
#
#   CScanJob::FinishJob writes WfcPortPropertyGoodScanStartTime only if
#     1. its status argument is 0
#     2. the job's m_bCancelled byte (+0x78A) is 0
#     3. the port property cache for the job's port id (+0x34) resolves
#
#   Without that property the connect job discards its candidate list
#   and OID_WDI_TASK_CONNECT is never sent. FinishJob is known to run --
#   it clears m_ScanInProgress before any of the gates -- so exactly one
#   of the three is rejecting the write, and reading which needs a stop.
#
# QEMU's gdbstub can stop it from the host. At FinishJob's first
# instruction the Microsoft x64 arguments are still in registers:
#
#   rcx = the CScanJob
#   edx = the status                <- gate 1
#   [rcx+0x78A] = m_bCancelled      <- gate 2
#   [rcx+0x34]  = the job's port id <- gate 3 input
#
# so a single hit answers all three.
#
# USAGE
#
#   1. Start the guest with a gdbstub. Add to the QEMU command line:
#
#        -gdb tcp::1234
#
#      (-s is the same thing on port 1234. Do NOT add -S; that halts the
#      guest at reset and you want it booted.)
#
#   2. In the guest, run dump-wdi-state.cmd. It prints
#
#        wdiwifi!CScanJob::FinishJob = 0x<addr>
#
#      That address is valid for THIS BOOT ONLY -- the driver is
#      relocated on every boot, so re-read it after every reboot.
#
#   3. On the host:
#
#        scripts/gdb-wdi-finishjob.sh 0x<addr> [port]
#
#   4. When gdb says it is waiting, trigger a scan in the guest -- open
#      the Wi-Fi flyout, or run `netsh wlan show networks`. The guest
#      freezes at the breakpoint; this prints the three values and lets
#      it go.
#
# The guest is stopped while the breakpoint is being serviced. That is
# normal and it resumes on its own.
set -euo pipefail

ADDR="${1:-}"
PORT="${2:-1234}"

if [[ -z "$ADDR" ]]; then
    echo "usage: $0 <address-of-wdiwifi!CScanJob::FinishJob> [gdb-port]" >&2
    echo "" >&2
    echo "Get the address from dump-wdi-state.cmd in the guest. It is" >&2
    echo "valid for one boot only." >&2
    exit 2
fi

# Normalise: kd prints kernel addresses as fffff804`505d02bc.
ADDR="${ADDR//\`/}"
[[ "$ADDR" == 0x* ]] || ADDR="0x$ADDR"

GDB="${GDB:-gdb}"
command -v "$GDB" >/dev/null 2>&1 || {
    echo "error: $GDB not found. Install gdb, or set GDB=/path/to/gdb." >&2
    exit 1
}

echo "Breaking on wdiwifi!CScanJob::FinishJob at $ADDR (gdbstub :$PORT)"
echo "Trigger a scan in the guest once gdb reports it is waiting."
echo

# hbreak, not break. A software breakpoint would write 0xCC into
# wdiwifi's code, which PatchGuard treats as exactly the kind of kernel
# modification it bugchecks for. A hardware breakpoint uses the debug
# registers and leaves the image alone.
#
# The reads are laid out to be unambiguous in the output rather than
# terse: this is meant to be pasted back verbatim.
exec "$GDB" -q -nx -batch \
    -ex "set pagination off" \
    -ex "set confirm off" \
    -ex "target remote :$PORT" \
    -ex "hbreak *$ADDR" \
    -ex "printf \"\\n=== waiting for CScanJob::FinishJob -- trigger a scan in the guest ===\\n\"" \
    -ex "continue" \
    -ex "printf \"\\n=== CScanJob::FinishJob hit ===\\n\"" \
    -ex "printf \"gate 1  status        = 0x%x\\n\", (unsigned int)\$rdx" \
    -ex "printf \"gate 2  m_bCancelled  = 0x%x\\n\", *(unsigned char *)(\$rcx + 0x78A)" \
    -ex "printf \"gate 3  port id       = 0x%x\\n\", *(unsigned short *)(\$rcx + 0x34)" \
    -ex "printf \"        cache source  = 0x%lx\\n\", *(unsigned long *)(\$rcx + 0x1F0)" \
    -ex "printf \"        job           = 0x%lx\\n\", (unsigned long)\$rcx" \
    -ex "printf \"        start time    = 0x%lx\\n\", *(unsigned long *)(\$rcx + 0x790)" \
    -ex "printf \"\\nAll three gates must read 0 / 0 / a resolvable port for the\\n\"" \
    -ex "printf \"good-scan time to be written. Whichever is not is the bug.\\n\\n\"" \
    -ex "detach"
