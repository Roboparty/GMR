#!/usr/bin/env python3
import os
import sys
import xml.etree.ElementTree as ET

script_dir = os.path.dirname(__file__)
input_path = sys.argv[1] if len(sys.argv) > 1 else os.path.abspath(os.path.join(script_dir, '..', 'generated_data.txt'))
output_path = sys.argv[2] if len(sys.argv) > 2 else os.path.abspath(os.path.join(script_dir, '..', 'generated_data.xml'))

root = ET.Element('document')

with open(input_path, 'r', encoding='utf-8') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        if ':' not in line:
            continue
        key, val = [s.strip() for s in line.split(':', 1)]
        if ',' in val:
            parent = ET.SubElement(root, key)
            for item in [s.strip() for s in val.split(',') if s.strip()]:
                ET.SubElement(parent, 'item').text = item
        else:
            child = ET.SubElement(root, key)
            child.text = val

tree = ET.ElementTree(root)
tree.write(output_path, encoding='utf-8', xml_declaration=True)
print(f'Wrote {output_path}')
