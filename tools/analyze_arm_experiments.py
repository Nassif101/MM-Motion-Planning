"""Summarize arm commissioning CSV(.gz) without conflating drive force with net joint effort."""
import argparse
import collections
import csv
import gzip
import json
import math
from pathlib import Path


def analyze(path):
    opener=gzip.open if path.suffix=='.gz' else open
    joints=collections.defaultdict(list)
    with opener(path,'rt') as stream:
        for row in csv.DictReader(stream):
            if None in row or any(v is None for v in row.values()):
                raise ValueError('Truncated CSV: '+str(path))
            joints[row['joint']].append(row)
    report={'joints':{}}
    for name,rows in joints.items():
        def values(key): return [float(r[key]) for r in rows]
        t=values('sim_time'); wall=values('wall_time'); error=values('position_error')
        velocity=values('velocity_error'); effort=values('drive_effort')
        tail=[r for r in rows if float(r['sim_time'])>=t[-1]-2]
        q=values('actual_position'); desired=values('desired_position')
        moving=[i for i,r in enumerate(rows) if abs(float(r['desired_velocity']))>1e-6]
        settled=None
        if moving:
            final=moving[-1]+1
            bad=[i for i in range(final,len(rows)) if abs(error[i])>.04 or abs(float(rows[i]['actual_velocity']))>.05]
            index=max([final]+[i+1 for i in bad])
            if index<len(rows) and t[-1]-t[index]>=2:
                settled=t[index]-t[final]
        report['joints'][name]={
            'samples':len(rows),'simulation_seconds':t[-1]-t[0],
            'physics_samples_per_sim_second':(len(rows)-1)/(t[-1]-t[0]),
            'physics_samples_per_wall_second':(len(rows)-1)/(wall[-1]-wall[0]),
            'max_position_error_rad':max(map(abs,error)),
            'rms_position_error_rad':math.sqrt(sum(e*e for e in error)/len(error)),
            'max_velocity_error_rad_s':max(map(abs,velocity)),
            'mean_last_2s_error_rad':sum(float(r['position_error']) for r in tail)/len(tail),
            'max_drive_effort_Nm':max(map(abs,effort)),
            'near_force_limit_percent':100*sum(int(r['estimated_saturated']) for r in rows)/len(rows),
            'command_envelope_overshoot_rad':max(0,max(q)-max(desired),min(desired)-min(q)),
            'settling_after_final_target_seconds':settled,
            'states':dict(collections.Counter(r['state'] for r in rows))}
    rows=next(iter(joints.values()))
    report['base']={key:max(abs(float(r[key])) for r in rows) for key in
                    ['base_velocity','base_yaw_rate','base_acceleration','base_yaw_acceleration']}
    ages=[float(r['command_age']) for r in rows if float(r['command_age'])>=0]
    report['max_command_age_seconds']=max(ages) if ages else None
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    paths=sorted(list(args.directory.glob('*.csv'))+list(args.directory.glob('*.csv.gz')))
    result={p.name:analyze(p) for p in paths}
    (args.directory/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    for name,report in result.items():
        print(name, 'max error',round(max(j['max_position_error_rad'] for j in report['joints'].values()),5))

if __name__=='__main__': main()
