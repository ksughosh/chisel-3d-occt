#!/usr/bin/env bash
# SPDX-License-Identifier: LGPL-2.1-only
# KERN-0 host-side smoke for libchisel_occt.so: LOAD align, NEEDED has no libTK*, only the two factory symbols exported, size.
# Usage: tools/occt/check_occt_lib.sh [path/to/libchisel_occt.so] [max_bytes]   (default max 13,631,488 B = KERN-5 budget, measured on the stripped file)
set -euo pipefail
SO="${1:-build/kern0-android/occt/libchisel_occt.so}"; MAX="${2:-20733020}"   # KERN-6 (U38): measured stripped 20,031,904 B (STEP writer, 18 TK archives) + 3.5 % headroom; was the 20,971,520 Architect ceiling (spec 17 8.1c) and 13631488 for KERN-5. KERN-5 (U35/ADR-027): measured 13,158,720 B (chamfer + draft, bend/STEP declared only) + 3.6 % headroom; user ceiling about 13.2 MB for the linked ops
SDK="${ANDROID_HOME:-$HOME/Library/Android/sdk}"; NB="$SDK/ndk/28.2.13676358/toolchains/llvm/prebuilt/darwin-x86_64/bin"
T="$(mktemp -d)"; "$NB/llvm-strip" --strip-all -o "$T/s.so" "$SO"; fail=0
size=$(stat -f%z "$T/s.so"); echo "stripped size: $size B"; [[ $size -le $MAX ]] || { echo "FAIL size > $MAX"; fail=1; }
al=$("$NB/llvm-readelf" -lW "$SO" | awk '$1=="LOAD"{print $NF}' | sort -u | tr '\n' ' '); echo "LOAD align: $al"; [[ "$al" == "0x4000 " ]] || { echo "FAIL LOAD align"; fail=1; }
"$NB/llvm-readelf" -d "$SO" | grep NEEDED | sed 's/.*\[\(.*\)\]/\1/' | tr '\n' ' '; echo
if "$NB/llvm-readelf" -d "$SO" | grep NEEDED | grep -q 'libTK'; then echo "FAIL NEEDED has libTK*"; fail=1; fi
ex=$("$NB/llvm-nm" -D --defined-only "$SO" | awk '{print $3}' | sed 's/@.*//' | sort | tr '\n' ' '); echo "exports: $ex"
[[ "$ex" == "chisel_occt_abi chisel_occt_make " ]] || { echo "FAIL exports differ from {chisel_occt_abi, chisel_occt_make}"; fail=1; }
if "$NB/llvm-nm" -D --undefined-only "$SO" | grep -qE ' _Z.*(Standard_|TopoDS|BRep|gp_)'; then echo "FAIL undefined OCCT symbol (archive missing from link)"; fail=1; fi
[[ $fail == 0 ]] && echo "OK libchisel_occt.so" ; exit $fail
