#!/bin/bash
# Compare two identity dumps ignoring the sign of zero (PX4 builds with
# -fno-signed-zeros: -0 vs +0 is not a behavior difference at the pins)
sed 's/-0x0p+0/0x0p+0/g' "$1" > /tmp/idcmp_a.txt
sed 's/-0x0p+0/0x0p+0/g' "$2" > /tmp/idcmp_b.txt
if cmp -s /tmp/idcmp_a.txt /tmp/idcmp_b.txt; then echo "IDENTICAL (modulo signed zero): $(wc -l < /tmp/idcmp_a.txt) lines"; else echo "DIFFERS:"; diff /tmp/idcmp_a.txt /tmp/idcmp_b.txt | head -8; exit 1; fi
