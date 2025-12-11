# make_tls_from_net.py
# Run in the same folder as simple_intersection.net.xml
import xml.etree.ElementTree as ET
from pathlib import Path

netfile = Path("simple_intersection.net.xml")
if not netfile.exists():
    print("ERROR: simple_intersection.net.xml not found in current folder.")
    raise SystemExit(1)

tree = ET.parse(netfile)
root = tree.getroot()

# Find junctions that are traffic lights
tls_junctions = []
for j in root.findall(".//junction"):
    typ = j.get("type","")
    if typ and typ.lower().startswith("traffic_light"):
        tls_junctions.append(j)

# fallback: some net files list <trafficLightLogic> or <tlLogic> - try finding any 'trafficLight' tags
if not tls_junctions:
    for j in root.findall(".//junction"):
        if "tl" in ET.tostring(j, encoding='unicode'):
            tls_junctions.append(j)

if not tls_junctions:
    print("No traffic-light junctions found in net. Nothing to do.")
    raise SystemExit(0)

# For each traffic-light junction, attempt to determine number of signal groups.
# We'll look for child <connection> elements that have the 'tl' attribute; each unique index forms a group.
tls_info = []
for j in tls_junctions:
    jid = j.get("id")
    # Count outgoing connections that mention a traffic light (heuristic)
    conns = j.findall(".//connection")
    # fallback: count lanes on the junction
    if conns:
        # determine unique 'tl' positions by collecting 'from'+'to' pairs (approx)
        groups = set()
        for c in conns:
            key = (c.get("from"), c.get("to"))
            groups.add(key)
        n_groups = max(1, len(groups))
    else:
        # fallback to counting lanes defined on nearby edges
        lanes = j.findall(".//lane")
        n_groups = max(1, len(lanes))
    tls_info.append((jid, n_groups))

# create tls.add.xml
out = ET.Element("additional")
for jid, n_groups in tls_info:
    tl = ET.SubElement(out, "tlLogic", attrib={"id": jid, "type": "static", "programID": "main", "offset":"0", "currentPhase":"0"})
    # create 2 simple phases with matching-length state strings
    # phase 1: first group green, others red: 'G' + 'r'*(n-1)
    state1 = "G" + "r"*(n_groups-1)
    # phase 2: first group yellow, others red
    state2 = "y" + "r"*(n_groups-1)
    # phase 3: rotate green to group 2 (if exists) else reuse
    if n_groups >= 2:
        # green for group 2 at pos 1 (0-index)
        state3 = "r"*1 + "G" + "r"*(n_groups-2)
        state4 = "r"*1 + "y" + "r"*(n_groups-2)
    else:
        state3 = state1
        state4 = state2
    ET.SubElement(tl, "phase", attrib={"duration":"20", "state":state1})
    ET.SubElement(tl, "phase", attrib={"duration":"5",  "state":state2})
    ET.SubElement(tl, "phase", attrib={"duration":"12", "state":state3})
    ET.SubElement(tl, "phase", attrib={"duration":"5",  "state":state4})

# write file (backup existing)
out_file = Path("tls.add.xml")
if out_file.exists():
    out_file.rename(out_file.with_suffix(".add.xml.bak"))
ET.ElementTree(out).write(out_file, encoding="utf-8", xml_declaration=True)
print(f"Wrote {out_file}. Backup created if original existed.")
print("Please re-run SUMO (sumo-gui -c simple_intersection.sumocfg) now.")
