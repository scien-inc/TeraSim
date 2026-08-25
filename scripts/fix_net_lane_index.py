#!/usr/bin/env python3
"""Fix incorrect lane index attributes in a SUMO .net.xml file.

Within each <edge>, the <lane> children should have index="0", index="1", ...
incrementing by their order. This script detects and fixes any mismatches,
then saves the corrected file.

Usage:
    python fix_net_lane_index.py --input map.net.xml --output map_fixed.net.xml
"""

import argparse
import xml.etree.ElementTree as ET


def fix_lane_indices(input_path, output_path):
    tree = ET.parse(input_path)
    root = tree.getroot()

    fixed_count = 0

    for edge in root.iter("edge"):
        lanes = edge.findall("lane")
        if not lanes:
            continue
        for expected_idx, lane in enumerate(lanes):
            actual_idx = int(lane.get("index", -1))
            if actual_idx != expected_idx:
                lane_id = lane.get("id")
                edge_id = edge.get("id")
                print(
                    f"Fix: edge={edge_id}, lane={lane_id}, "
                    f"index {actual_idx} -> {expected_idx}"
                )
                lane.set("index", str(expected_idx))
                fixed_count += 1

    print(f"\nTotal fixes: {fixed_count}")

    if fixed_count > 0:
        tree.write(output_path, encoding="UTF-8", xml_declaration=True)
        print(f"Saved to: {output_path}")
    else:
        print("No fixes needed. Output file not written.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Fix lane index attributes in a SUMO .net.xml file."
    )
    parser.add_argument("--input", type=str, required=True, help="Path to the input .net.xml file.")
    parser.add_argument("--output", type=str, required=True, help="Path to save the fixed .net.xml file.")
    args = parser.parse_args()

    fix_lane_indices(args.input, args.output)
