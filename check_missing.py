#!/usr/bin/env python3
import os
import re

# Hardcoded paths relative to this script.
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
XACRO_FILE = os.path.join(SCRIPT_DIR, "src", "robco_description", "urdf", "robco_modules.xacro")
BASE_PATH  = os.path.join(SCRIPT_DIR, "src", "robco_description", "meshes")

# List of attribute keys to look for.
ATTR_KEYS = ["mesh_filename", "proximal_mesh_filename", "distal_mesh_filename"]

def main():
    try:
        with open(XACRO_FILE, "r", encoding="utf-8") as f:
            content = f.read()
    except Exception as e:
        print(f"Error reading {XACRO_FILE}: {e}")
        return

    # Find all macro blocks. We assume each macro block is delimited by <xacro:macro ...> and </xacro:macro>
    macro_pattern = re.compile(r'(<xacro:macro\s+.*?</xacro:macro>)', re.DOTALL)
    macros = macro_pattern.findall(content)
    
    if not macros:
        print("No macros found.")
        return

    for macro in macros:
        # Extract the macro name from the opening tag.
        name_match = re.search(r'<xacro:macro\s+[^>]*name="([^"]+)"', macro)
        macro_name = name_match.group(1) if name_match else "unknown"

        missing = []  # List of (attr, value, full_path) tuples.
        # For each attribute key, find all occurrences in this macro block.
        for key in ATTR_KEYS:
            # Look for attribute key="value"
            pattern = re.compile(r'%s="([^"]+)"' % re.escape(key))
            for match in pattern.finditer(macro):
                file_rel = match.group(1)
                full_path = os.path.join(BASE_PATH, file_rel)
                if not os.path.exists(full_path):
                    missing.append((key, file_rel, full_path))
        if missing:
            for key, file_rel, full_path in missing:
                print(f"'{macro_name}': {file_rel}")

if __name__ == "__main__":
    main()
