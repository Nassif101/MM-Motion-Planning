"""Site selection shared by the navigation, global planning and manipulation launches.

Reads only the YAML files of a share directory: the top-level `site` of each
config/scenarios*.yaml (default construction_site) and the keys of its `scenarios` mapping.
"""
from pathlib import Path

import yaml

DEFAULT_SITE = "construction_site"


def scenario_site(share: Path, scenario: str) -> str:
    for path in sorted((Path(share) / "config").glob("scenarios*.yaml")):
        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if scenario in (document.get("scenarios") or {}):
            return document.get("site", DEFAULT_SITE)
    raise RuntimeError(f"Unknown scenario '{scenario}' in config/scenarios*.yaml")


def launch_site(share: Path, site: str, scenario: str) -> str:
    if not scenario:
        return site or DEFAULT_SITE
    scenario_s = scenario_site(share, scenario)
    if site and site != scenario_s:
        raise RuntimeError(f"site {site} differs from scenario {scenario}'s site {scenario_s}")
    return scenario_s


def site_map_yaml(share: Path, site: str) -> str:
    path = Path(share) / "maps" / f"{site}.yaml"
    if not path.is_file():
        known = sorted(p.name[:-len(".yaml")] for p in (Path(share) / "maps").glob("*.yaml")
                       if not p.name.endswith(".boxes.yaml"))
        raise RuntimeError(f"Unknown site '{site}'; expected one of {known}")
    return str(path)
