"""Check units against effective native fields, never against source extensions."""

from lxml import etree


def opendrive_speed_consistency(path):
    root = etree.parse(str(path)).getroot()
    records = []
    factors = {"m/s": 1, "km/h": 1 / 3.6, "mph": 0.44704}
    for road in root.findall("road"):
        sections = road.findall("lanes/laneSection")
        for signal in road.findall("signals/signal"):
            if signal.get("type") != "274" or signal.get("value") is None:
                continue
            signal_s = float(signal.get("s"))
            validity = signal.findall("validity")
            for i, section in enumerate(sections):
                start = float(section.get("s"))
                end = (
                    float(sections[i + 1].get("s"))
                    if i + 1 < len(sections)
                    else float(road.get("length"))
                )
                if not start <= signal_s < end:
                    continue
                for lane in section.findall("*/lane"):
                    lid = int(lane.get("id"))
                    if (
                        lid == 0
                        or validity
                        and not any(
                            int(v.get("fromLane")) <= lid <= int(v.get("toLane")) for v in validity
                        )
                    ):
                        continue
                    limits = [
                        l
                        for l in lane.findall("speed")
                        if float(l.get("sOffset")) <= signal_s - start
                    ]
                    if not limits:
                        continue
                    limit = max(limits, key=lambda l: float(l.get("sOffset")))
                    lane_speed = float(limit.get("max")) * factors[limit.get("unit", "m/s")]
                    sign_speed = float(signal.get("value")) * factors[signal.get("unit", "m/s")]
                    records.append(
                        {
                            "road": road.get("id"),
                            "lane": str(lid),
                            "signal": signal.get("id"),
                            "lane_speed_mps": lane_speed,
                            "signal_speed_mps": sign_speed,
                            "passed": abs(lane_speed - sign_speed) <= 0.01,
                        }
                    )
    return {
        "status": "FAIL" if any(not r["passed"] for r in records) else "PASS",
        "scope": "Native signal validity, effective lane section and speed record at signal station",
        "records": records,
    }
