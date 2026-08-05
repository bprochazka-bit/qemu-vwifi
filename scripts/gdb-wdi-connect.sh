#!/usr/bin/env bash
#
# Trace wdiwifi's connect decision from outside the guest, through
# QEMU's gdbstub.
#
# WHY THIS EXISTS
#
# A Windows guest cannot break on itself. Local kernel debugging (kd
# -kl) reads memory and disassembles happily, which is how this whole
# failure was traced, but it cannot stop the machine -- and everything
# still open is a runtime fact.
#
# The chain, read end to end out of the binary:
#
#   CScanJob::FinishJob writes WfcPortPropertyGoodScanStartTime only if
#     1. its status argument is 0
#     2. the job's m_bCancelled byte (+0x78A) is 0
#     3. the port property cache for the job's port id (+0x34) resolves
#
#   CConnectJob::CheckAndUpdateCandidates reads that property, and if it
#   comes back STATUS_INVALID_DEVICE_STATE it zeroes the job's candidate
#   count at +0x26C. CheckAndStartConnectProcess compares that count
#   against zero and, finding zero, skips StartConnectRoamTask -- which
#   is the only thing that ever sends OID_WDI_TASK_CONNECT.
#
# WHY IT REPLACED THE SINGLE-SHOT VERSION
#
# The first version of this script broke on FinishJob, printed one hit,
# and detached. It reported
#
#   gate 1  status = 0x40230001    (STATUS_NDIS_INDICATION_REQUIRED)
#
# which was read as "our task status fails gate 1, always". One sample
# cannot say "always". FinishJob is called more than once per job, and
# the call that matters -- the one driven by our SCAN_COMPLETE
# indication -- was never in the sample. Changing the driver on the
# strength of it cost a build and broke the scan job's lifetime.
#
# So this one stays attached and prints EVERY hit, at four points:
#
#   1. CScanJob::FinishJob                        entry
#   2. CConnectJob::CheckAndUpdateCandidates      +0x317, the store
#                                                 that zeroes the count
#   3. CConnectJob::CheckAndStartConnectProcess   +0x1a3, the compare
#                                                 that reads it
#   4. CConnectJob::StartConnectRoamTask          entry
#
# Four is the limit -- x86 has four debug registers, and these must be
# hardware breakpoints. A software breakpoint writes 0xCC into
# wdiwifi's code, which PatchGuard treats as exactly the kind of kernel
# modification it bugchecks for.
#
# WHAT THE OUTPUT MEANS
#
#   [1] once, status non-zero, and never again
#       the M3 does not drive FinishJob at all. Neither
#       INDICATION_REQUIRED nor SUCCESS can be right, and the task OID
#       should return NDIS_STATUS_PENDING and be completed later with
#       NdisMOidRequestComplete.
#   [1] twice, the second with status 0
#       the good-scan property is being written and is not the problem.
#       Whatever discards the candidates is somewhere else.
#   [2] hit
#       the property read failed and the candidate list was discarded.
#       That is the good-scan chain, confirmed at runtime.
#   [3] reporting 0 without [2] having been hit
#       the count was never filled in. PickCandidates is then the place
#       to look, not the property.
#   [4] hit
#       the connect task went out and the failure is downstream of
#       everything above.
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
#   2. In the guest, run dump-wdi-state.cmd. It prints a ready-made
#      command line for this script with every address filled in. Those
#      addresses are valid for THIS BOOT ONLY -- wdiwifi is relocated on
#      every boot, so re-read them after every reboot.
#
#   3. On the host, run that command line.
#
#   4. When gdb says it is waiting, attempt the connection in the guest.
#      The guest freezes briefly at each hit and resumes on its own.
#
#   5. When the attempt has failed, press Ctrl-C, then type
#
#        detach
#        quit
#
#      Detaching matters: quitting while the guest is stopped can leave
#      it frozen.
#
# Any address may be omitted; only the breakpoints you give are set.
set -euo pipefail

PORT=1234
FINISH_JOB=
UPDATE_CANDIDATES=
START_CONNECT=
ROAM_TASK=

# Offsets into the two functions whose interesting point is not their
# entry. Both come from disassembling this guest's wdiwifi.sys, so a
# Windows update that changes the binary changes them. Each of those two
# breakpoints prints the instruction it landed on for exactly that
# reason: if it is not the `mov` or the `cmp` named here, the offsets
# are stale and the numbers beside them mean nothing.
OFF_ZERO_COUNT=0x317      # mov dword ptr [rbx+26Ch],r13d
OFF_DECIDE=0x1a3          # cmp dword ptr [rbx+26Ch],r13d
OFF_CANDIDATE_COUNT=0x26C # CConnectJob's candidate count

usage() {
    sed -n '3,/^set -euo/p' "$0" | sed 's/^#\{0,1\} \{0,1\}//;$d'
    exit "${1:-2}"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --finish-job)        FINISH_JOB="${2:-}";        shift 2 ;;
        --update-candidates) UPDATE_CANDIDATES="${2:-}"; shift 2 ;;
        --start-connect)     START_CONNECT="${2:-}";     shift 2 ;;
        --roam-task)         ROAM_TASK="${2:-}";         shift 2 ;;
        --port)              PORT="${2:-1234}";          shift 2 ;;
        -h|--help)           usage 0 ;;
        *) echo "unknown argument: $1" >&2; usage 2 ;;
    esac
done

# kd prints kernel addresses as fffff804`505d02bc. Strip the tick and
# make sure gdb sees hex.
norm() {
    local a="${1//\`/}"
    if [[ -n "$a" && "$a" != 0x* ]]; then a="0x$a"; fi
    printf '%s' "$a"
}

FINISH_JOB=$(norm "$FINISH_JOB")
UPDATE_CANDIDATES=$(norm "$UPDATE_CANDIDATES")
START_CONNECT=$(norm "$START_CONNECT")
ROAM_TASK=$(norm "$ROAM_TASK")

if [[ -z "$FINISH_JOB$UPDATE_CANDIDATES$START_CONNECT$ROAM_TASK" ]]; then
    echo "error: no addresses given -- nothing to break on." >&2
    echo "" >&2
    echo "Run dump-wdi-state.cmd in the guest; it prints the whole" >&2
    echo "command line for this script, addresses included." >&2
    exit 2
fi

GDB="${GDB:-gdb}"
command -v "$GDB" >/dev/null 2>&1 || {
    echo "error: $GDB not found. Install gdb, or set GDB=/path/to/gdb." >&2
    exit 1
}

CMDS=$(mktemp -t gdb-wdi-connect.XXXXXX)
trap 'rm -f "$CMDS"' EXIT

{
    echo 'set pagination off'
    echo 'set confirm off'
    echo 'set height 0'
    echo "target remote :$PORT"

    if [[ -n "$FINISH_JOB" ]]; then
        # Microsoft x64: rcx = this (the CScanJob), edx = the status.
        # Every gate input is still in a register or reachable from rcx
        # at the first instruction, so one hit answers all three.
        cat <<EOF
hbreak *$FINISH_JOB
commands
  silent
  printf "\n[1] CScanJob::FinishJob  status=0x%x  cancelled=0x%x  portid=0x%x\n", (unsigned int)\$rdx, *(unsigned char *)(\$rcx + 0x78A), *(unsigned short *)(\$rcx + 0x34)
  printf "    job=0x%lx  cache=0x%lx  starttime=0x%lx\n", (unsigned long)\$rcx, *(unsigned long *)(\$rcx + 0x1F0), *(unsigned long *)(\$rcx + 0x790)
  continue
end
EOF
    fi

    if [[ -n "$UPDATE_CANDIDATES" ]]; then
        # rbx is the CConnectJob here, not rcx: this is mid-function,
        # well past the prologue that moved it.
        cat <<EOF
hbreak *($UPDATE_CANDIDATES + $OFF_ZERO_COUNT)
commands
  silent
  printf "\n[2] CheckAndUpdateCandidates+$OFF_ZERO_COUNT  DISCARDING CANDIDATES  count=%u -> 0\n", *(unsigned int *)(\$rbx + $OFF_CANDIDATE_COUNT)
  printf "    good-scan property read failed. landed on: "
  x/1i \$pc
  continue
end
EOF
    fi

    if [[ -n "$START_CONNECT" ]]; then
        cat <<EOF
hbreak *($START_CONNECT + $OFF_DECIDE)
commands
  silent
  printf "\n[3] CheckAndStartConnectProcess+$OFF_DECIDE  candidates=%u\n", *(unsigned int *)(\$rbx + $OFF_CANDIDATE_COUNT)
  printf "    zero here means no connect task. landed on: "
  x/1i \$pc
  continue
end
EOF
    fi

    if [[ -n "$ROAM_TASK" ]]; then
        cat <<EOF
hbreak *$ROAM_TASK
commands
  silent
  printf "\n[4] StartConnectRoamTask -- CONNECT TASK GOING OUT  job=0x%lx\n", (unsigned long)\$rcx
  continue
end
EOF
    fi

    cat <<'EOF'
printf "\n=== attached. Attempt the connection in the guest now.    ===\n"
printf "=== Ctrl-C, then 'detach' and 'quit', once it has failed. ===\n\n"
continue
EOF
} > "$CMDS"

# `if`, not `[[ ... ]] && echo`. Under `set -e` a trailing test that
# comes out false is the script's exit status, so omitting the last
# breakpoint would have made this quit without ever starting gdb.
echo "Breakpoints (gdbstub :$PORT):"
if [[ -n "$FINISH_JOB"        ]]; then echo "  [1] CScanJob::FinishJob                        $FINISH_JOB"; fi
if [[ -n "$UPDATE_CANDIDATES" ]]; then echo "  [2] CheckAndUpdateCandidates+$OFF_ZERO_COUNT         $UPDATE_CANDIDATES"; fi
if [[ -n "$START_CONNECT"     ]]; then echo "  [3] CheckAndStartConnectProcess+$OFF_DECIDE       $START_CONNECT"; fi
if [[ -n "$ROAM_TASK"         ]]; then echo "  [4] StartConnectRoamTask                       $ROAM_TASK"; fi
echo

exec "$GDB" -q -nx -x "$CMDS"
