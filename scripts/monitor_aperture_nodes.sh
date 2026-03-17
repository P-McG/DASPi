#!/bin/bash

NODES=(
aperturecomputemodule000
aperturecomputemodule001
#aperturecomputemodule002
#aperturecomputemodule003
)

while true; do
    clear
    echo "Aperture Network Monitor"
    echo "Time: $(date)"
    echo "----------------------------------------"

    for node in "${NODES[@]}"; do
        ping -c1 -W1 $node > /tmp/ping_$node 2>/dev/null

        if grep -q "time=" /tmp/ping_$node; then
            LAT=$(grep "time=" /tmp/ping_$node | sed -E 's/.*time=([0-9.]+).*/\1/')
            printf "%-30s %8s ms\n" "$node" "$LAT"
        else
            printf "%-30s %8s\n" "$node" "DROP"
        fi
    done

    sleep 1
done
