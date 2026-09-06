#!/usr/bin/env python3
"""Check recorded physical acceptance criteria; action success alone is insufficient."""
import argparse
import csv
import gzip
import json
from pathlib import Path


def analyze(path):
    opener=gzip.open if path.suffix=='.gz' else open
    with opener(path,'rt') as stream:
        rows=list(csv.DictReader(stream))
    stem=path.name.removesuffix('.gz').removesuffix('.csv')
    action=json.loads(path.with_name(stem+'.json').read_text())
    def values(key): return [float(r[key]) for r in rows]
    ticks=[r for r in rows if r['joint']==rows[0]['joint']]
    checks={
        'action_succeeded':action['status']==4 and action['error_code']==0,
        'path_error_below_0p15_rad':max(map(abs,values('position_error')))<.15,
        'hold_error_below_0p06_rad':max(action['hold_max_error'])<.06,
        'no_fault_or_watchdog':all(r['state'] in ['HOLD','EXTERNAL_CONTROL'] for r in rows),
        'no_detected_panel_penetration':max(values('panel_penetration_m'))==0,
        'collision_observer_not_overflowed':not any(values('overlap_buffer_full')),
        'panel_ground_clearance_over_0p15m':min(values('panel_bottom'))>.15,
        'base_tilt_below_3_degrees':max(values('base_tilt_degrees'))<3,
        'each_drive_saturation_below_2_percent':all(
            sum(float(r['estimated_saturated']) for r in rows if r['joint']==name)/sum(r['joint']==name for r in rows)<.02
            for name in {r['joint'] for r in rows}),
    }
    result={'seconds':float(rows[-1]['sim_time'])-float(rows[0]['sim_time']),
            'max_path_error_rad':max(map(abs,values('position_error'))),
            'max_hold_error_rad':max(action['hold_max_error']),
            'min_panel_bottom_m':min(values('panel_bottom')),
            'max_panel_penetration_m':max(values('panel_penetration_m')),
            'max_base_tilt_degrees':max(values('base_tilt_degrees')),
            'max_base_speed_mps':max(map(abs,values('base_velocity'))),
            'max_base_yaw_rate_radps':max(map(abs,values('base_yaw_rate'))),
            'saturation_fraction':sum(values('estimated_saturated'))/len(rows),
            'max_command_age_seconds':max(values('command_age')),
            'max_wall_gap_between_physics_samples_s':max(
                (float(b['wall_time'])-float(a['wall_time']) for a,b in zip(ticks,ticks[1:])),default=0),
            'checks':checks}
    if action.get('disturbance')=='gate':
        throat=[r for r in rows if float(r['robot_min_z'])< -7.025 and float(r['robot_max_z'])> -7.425]
        margin=min((min(float(r['robot_min_x'])-7.2,8.25-float(r['robot_max_x'])) for r in throat),default=-1)
        result['gate_min_lateral_margin_m']=margin
        checks['crossed_gate']=bool(throat) and float(rows[0]['robot_min_z'])> -7.025 and float(rows[-1]['robot_max_z'])< -7.425
        checks['whole_robot_gate_margin_over_0p10m']=margin>.10
    result['passed']=all(checks.values())
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--prefix',default='final-')
    args=parser.parse_args()
    reports={p.name:analyze(p) for p in sorted(args.directory.glob(args.prefix+'*.csv*'))}
    if not reports: raise RuntimeError('No qualification recordings found')
    (args.directory/'acceptance.json').write_text(json.dumps(reports,indent=2)+'\n')
    print(json.dumps(reports,indent=2))
    if not all(r['passed'] for r in reports.values()): raise SystemExit(1)


if __name__=='__main__': main()
