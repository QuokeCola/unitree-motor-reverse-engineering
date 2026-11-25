#!/usr/bin/env python3
"""!
@file doxygen_matlab_filter.py
@brief Doxygen input filter that turns a MATLAB file into C++-like text.

Doxygen cannot read MATLAB. This filter keeps the `%>` comment lines as `///`
comments and turns `classdef` and `function` lines into declarations that
Doxygen understands; everything else is dropped.

Usage (from the Doxyfile): FILTER_PATTERNS = *.m="python3 this_file.py"
"""
import re
import sys

## Matches a classdef line: class name and optional base classes.
_CLASSDEF = re.compile(r"^\s*classdef\s+(\w+)(?:\s*<\s*([\w.&\s]+))?")
## Matches a function line: outputs, name and parameters.
_FUNCTION = re.compile(
    r"^\s*function\s+(?:\[?([\w,\s~]*)\]?\s*=\s*)?([\w.]+)\s*(?:\(([^)]*)\))?")


def convert(lines: list[str]) -> list[str]:
    """!
    @brief Converts the lines of one MATLAB file.
    @param lines Source lines.
    @return Lines of C++-like text for Doxygen.
    """
    out = []
    class_name = None
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("%>"):
            out.append("///" + stripped[2:])
            continue
        match = _CLASSDEF.match(line)
        if match:
            class_name = match.group(1)
            base = (match.group(2) or "").strip()
            inherit = f" : public {base}" if base else ""
            out.append(f"class {class_name}{inherit} {{")
            out.append("public:")
            continue
        match = _FUNCTION.match(line)
        if match:
            returns, name, params = match.groups()
            names = [p.strip() for p in (params or "").split(",") if p.strip()]
            if class_name:
                names = names[1:] if names and name != class_name else names
            args = ", ".join(f"in {p}" for p in names if p != "~")
            if class_name and name == class_name:
                out.append(f"{name}({args});")
            elif class_name and name == "delete":
                out.append(f"~{class_name}();")
            else:
                result = (returns or "").strip() or "void"
                out.append(f"{result.replace(',', ' ')} {name}({args});")
            continue
        out.append("")
    if class_name:
        out.append("};")
    return out


def main() -> int:
    """!
    @brief Filters the file named on the command line to standard output.
    @return Exit code.
    """
    with open(sys.argv[1], encoding="utf-8") as source:
        print("\n".join(convert(source.read().splitlines())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
