# auto_make_tls.py
# Usage: python auto_make_tls.py
# Reads simple_intersection.net.xml, finds traffic-light junction(s), determines required phase string length
# and writes tls.add.xml with phases that match the expected length. Backs up existing tls.add.xml to tls.add.xml.bak

import xml.etree.ElementTree as ET
from pathlib import Path
import sys

NETFILE = Path("simple_intersection.net.xml")
OUTFILE = Path("tls.add.xml")

if not NETFILE.exists():
    print("ERROR: simple_intersection.net.xml not found in current folder.")
    sys.exit(1)

tree = ET.parse(NETFILE)
root = tree.getroot()

# collect junctions that are traffic lights
tls_junctions = []
for j in root.findall(".//junction"):
    t = j.get("type", "").lower()
    if "traffic_light" in t or t == "traffic_light":
        tls_junctions.append(j)
# additional fallback: treat junctions with 'intLanes' and multiple internal lanes as candidates
if not tls_junctions:
    for j in root.findall(".//junction"):
        if j.get("intLanes"):
            tls_junctions.append(j)

if not tls_junctions:
    print("No junctions with traffic_light type or intLanes found in net. Nothing to do.")
    sys.exit(0)

additional = ET.Element("additional")

for j in tls_junctions:
    jid = j.get("id")
    intlanes = j.get("intLanes", "").strip()
    if intlanes:
        groups = [tok for tok in intlanes.split() if tok.strip()]
        n_groups = len(groups)
    else:
        # fallback: try incLanes count
        inc = j.get("incLanes", "").strip()
        groups = [tok for tok in inc.split() if tok.strip()]
        n_groups = max(1, len(groups))

    # ensure n_groups >= 1
    n_groups = max(1, n_groups)

    print(f"Junction {jid}: detected {n_groups} signal groups.")

    # create tlLogic element
    tl_attrib = {"id": jid, "type": "static", "programID": "main", "offset": "0", "currentPhase": "0"}
    tl = ET.SubElement(additional, "tlLogic", tl_attrib)

    # build phases: we'll create up to n_groups phases where each phase gives green to one or more groups
    # Strategy:
    # - If n_groups <= 4: make 4-phase cycle alternating halves.
    # - Else: create n_groups phases rotating a single-green per index, plus short yellows.
    if n_groups <= 4:
        # simple 4-phase pattern: group 0..n-1 get green in sensible pairs
        # form states of length n_groups
        # phase A - first half green
        stateA = "".join("G" if i < (n_groups+1)//2 else "r" for i in range(n_groups))
        stateAy = "".join("y" if s=="G" else "r" for s in stateA)
        stateB = "".join("G" if i >= (n_groups+1)//2 else "r" for i in range(n_groups))
        stateBy = "".join("y" if s=="G" else "r" for s in stateB)
        ET.SubElement(tl, "phase", {"duration":"20", "state": stateA})
        ET.SubElement(tl, "phase", {"duration":"4",  "state": stateAy})
        ET.SubElement(tl, "phase", {"duration":"18", "state": stateB})
        ET.SubElement(tl, "phase", {"duration":"4",  "state": stateBy})
    else:
        # rotational single-green per index with yellow after each
        for i in range(n_groups):
            stateG = "".join("G" if idx==i else "r" for idx in range(n_groups))
            stateY = "".join("y" if idx==i else "r" for idx in range(n_groups))
            ET.SubElement(tl, "phase", {"duration":"10", "state": stateG})
            ET.SubElement(tl, "phase", {"duration":"3",  "state": stateY})

# backup existing tls.add.xml if exists
if OUTFILE.exists():
    OUTFILE.rename(OUTFILE.with_suffix(".add.xml.bak"))

# write pretty-ish XML
ET.ElementTree(additional).write(OUTFILE, encoding="utf-8", xml_declaration=True)
print(f"Wrote {OUTFILE}. If previous tls.add.xml existed it was backed up to tls.add.xml.bak")
print("Now run SUMO (sumo-gui -c simple_intersection.sumocfg) or run your python controller.")
