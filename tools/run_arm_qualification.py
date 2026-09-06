#!/usr/bin/env python3
"""Repeat bounded arm/payload integration tests in the already-running Unity/ROS simulation.

No planner is used. Requires the construction-site scene in Play and an active arm
controller. The gate suite teleports the stopped articulation to a declared fixture
start; the gate traversal itself uses physical wheel drives. Never use on hardware.
"""
import argparse
import json
import math
import subprocess
from pathlib import Path
from analyze_arm_qualification import analyze

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'docs/experiments/arm-controller/qualification'
VERTICAL = [math.pi/2, 0, 0, 0, math.pi/2, 0]
LEVEL = [0, math.pi/2, 0, 0, -math.pi/2, 0]
HOME = [0.0]*6


def unity(command, *parameters):
    result = subprocess.run(['unity','command',command,*parameters,'--format','json'],
                            text=True,capture_output=True,timeout=30)
    result.check_returncode()
    envelope=json.loads(result.stdout)
    if not envelope['success']:
        raise RuntimeError(result.stdout)
    return envelope['data']['result']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--container',required=True)
    parser.add_argument('--suite',choices=['manipulation','gate','timing'],default='manipulation')
    parser.add_argument('--prefix',default='final')
    parser.add_argument('--start-at',help='Resume at a named case after diagnosing an interrupted run')
    args=parser.parse_args()
    if not args.prefix.replace('-','').isalnum():
        parser.error('Prefix must contain only letters, numbers and hyphens')
    OUTPUT.mkdir(parents=True,exist_ok=True)
    readiness=unity('arm_test_snapshot')
    if not readiness['playing'] or readiness['state']!='HOLD' or not 0<=readiness['age']<.5 or readiness['speed']>.05:
        raise RuntimeError('Requires fresh controlled HOLD and a stationary base: '+str(readiness))
    if args.suite=='gate':
        # Gate inner faces are x=7.20 and 8.25, z=-7.225. Start outside the throat.
        unity('arm_test_place_gate')
        cases=[('gate-crossing',VERTICAL,2,21,'gate')]
    elif args.suite=='timing':
        cases=[('frame-hold',VERTICAL,2,20,'none')]
    else:
        unity('arm_test_place_open')
        cases=[('vertical-transition',VERTICAL,8,10,'none'),
               ('vertical-base',VERTICAL,2,24,'compact'),
               ('home-return',HOME,8,5,'none'),
               ('level-extension',LEVEL,8,60,'none'),
               ('level-base',LEVEL,2,14,'extended'),
               ('home-return-2',HOME,8,5,'none'),
               ('vertical-repeat',VERTICAL,8,10,'none')]
    if args.start_at:
        names=[case[0] for case in cases]
        if args.start_at not in names: parser.error('Unknown start case')
        cases=cases[names.index(args.start_at):]
    for name,q,duration,hold,disturbance in cases:
        stem=args.prefix+'-'+name
        csv_path=OUTPUT/(stem+'.csv')
        if csv_path.exists() or csv_path.with_suffix('.json').exists():
            raise RuntimeError('Refusing to overwrite evidence: '+str(csv_path))
        unity('arm_test_record','--name',stem)
        print('START '+stem,flush=True)
        cmd=['docker','exec',args.container,'bash','-lc',
             'source "$ROS_WS/install/setup.bash" && exec python3 "$ROS_WS/src/mobile_manipulator_control/scripts/arm_experiment.py" "$@"',
             'arm-qualification','--positions',*[str(x) for x in q],
             '--duration',str(duration),'--hold-seconds',str(hold),'--disturbance',disturbance,
             '--output','/workspaces/mm-motion-planning/docs/experiments/arm-controller/qualification/'+stem+'.json']
        try:
            result=subprocess.run(cmd,text=True,capture_output=True,timeout=240)
            print(result.stdout,flush=True)
            if result.returncode:
                raise RuntimeError(result.stderr+'\n'+result.stdout)
        finally:
            unity('arm_test_end')
        report=json.loads(csv_path.with_suffix('.json').read_text())
        if report['status']!=4 or report['error_code']!=0 or max(report['hold_max_error'])>.06:
            raise RuntimeError('Action/hold acceptance failed: '+str(report))
        physical=analyze(csv_path)
        if not physical['passed']:
            raise RuntimeError('Physical acceptance failed: '+str(physical))
        print('PASS '+stem,flush=True)


if __name__=='__main__':
    main()
