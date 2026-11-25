#!/usr/bin/env python3
"""!
@file doxygen_matlab_filter.py
@brief Doxygen input filter that turns a MATLAB file into C++-like text.

Doxygen cannot read MATLAB. This filter keeps the `%>` comment lines as `///`
comments and turns `classdef`, `properties`, `methods` and `function` lines
into declarations that Doxygen understands; everything else is dropped.

Usage (from the Doxyfile): FILTER_PATTERNS = *.m="python3 this_file.py"
"""
import re
import sys

## Matches a classdef line: class name and optional base classes.
_CLASSDEF = re.compile(r"^\s*classdef\s+(\w+)(?:\s*<\s*([\w.&\s]+))?")
## Matches the start of a properties or methods block and its attributes.
_BLOCK = re.compile(r"^\s*(properties|methods)\b\s*(?:\(([^)]*)\))?\s*$")
## Matches a property line: its name.
_PROPERTY = re.compile(r"^\s*(\w+)\s*(?:=.*)?$")
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
    in_properties = False
    static = False
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
            continue
        match = _BLOCK.match(line) if class_name else None
        if match:
            kind, attributes = match.group(1), match.group(2) or ""
            in_properties = kind == "properties"
            static = "Static" in attributes
            out.append("private:" if "private" in attributes else "public:")
            continue
        if in_properties:
            if stripped == "end":
                in_properties = False
                out.append("")
                continue
            match = _PROPERTY.match(line)
            out.append(f"var {match.group(1)};" if match and stripped else "")
            continue
        match = _FUNCTION.match(line)
        if match:
            returns, name, params = match.groups()
            names = [p.strip() for p in (params or "").split(",") if p.strip()]
            is_constructor = class_name and name == class_name
            if class_name and not static and not is_constructor:
                names = names[1:]  # Drop the object argument.
            args = ", ".join(f"in {p}" for p in names if p != "~")
            if is_constructor:
                out.append(f"{name}({args});")
            elif class_name and name == "delete":
                out.append(f"~{class_name}();")
            else:
                result = (returns or "").strip() or "void"
                prefix = "static " if class_name and static else ""
                out.append(f"{prefix}{result.replace(',', ' ')} "
                           f"{name}({args});")
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
