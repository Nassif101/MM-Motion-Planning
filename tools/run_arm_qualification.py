#!/usr/bin/env python3
"""Repeat bounded arm/payload integration tests in the already-running Unity/ROS simulation.

The manipulation, gate and timing suites use explicit trajectories (no planner). The moveit
suite qualifies MoveIt-planned B3 transitions through the ReconfigurePanel action and needs
manipulation.launch.py running as well. Requires the construction-site scene in Play and an
active arm controller. The gate suite teleports the stopped articulation to a declared fixture
start; the gate traversal itself uses physical wheel drives. Never use on hardware.
"""
import argparse
import json
import math
import subprocess
import time
from pathlib import Path
from analyze_arm_qualification import analyze

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'docs/experiments/arm-controller/qualification'
MOVEIT_OUTPUT = ROOT / 'docs/experiments/moveit-arm/qualification'
# Panel centre in base_footprint for vertical carry (panel_pose --pose vertical_carry).
VERTICAL_PANEL = ['-0.08', '0.225', '1.32', '-1.5707963267948966', '1.5707963267948966', '0']
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
    parser.add_argument('--suite',choices=['manipulation','gate','timing','moveit'],default='manipulation')
    parser.add_argument('--repeats',type=int,default=5,help='moveit suite: runs of each transition')
    parser.add_argument('--prefix',default='final')
    parser.add_argument('--start-at',help='Resume at a named case after diagnosing an interrupted run')
    args=parser.parse_args()
    if not args.prefix.replace('-','').isalnum():
        parser.error('Prefix must contain only letters, numbers and hyphens')
    readiness=unity('arm_test_snapshot')
    if not readiness['playing'] or readiness['state']!='HOLD' or not 0<=readiness['age']<.5 or readiness['speed']>.05:
        raise RuntimeError('Requires fresh controlled HOLD and a stationary base: '+str(readiness))
    if args.suite=='moveit':
        return moveit_suite(args)
    OUTPUT.mkdir(parents=True,exist_ok=True)
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
             'source "$ROS_WS/install/setup.bash" && exec ros2 run mobile_manipulator_control arm_experiment "$@"',
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


def moveit_suite(args):
    """B3 transitions planned by MoveIt, recorded and judged like the explicit trajectories."""
    MOVEIT_OUTPUT.mkdir(parents=True,exist_ok=True)
    unity('arm_test_place_open')
    transitions=[('home-to-vertical',['--named','vertical_carry','--profile','vertical_carry']),
                 ('vertical-to-home',['--named','home','--profile','home']),
                 ('home-to-vertical-panel',['--panel-pose',*VERTICAL_PANEL,
                                            '--position-tolerance','0.01','0.01','0.01',
                                            '--orientation-tolerance','0.01','0.01','0.01',
                                            '--profile','vertical_carry']),
                 ('vertical-to-home-2',['--named','home','--profile','home'])]
    failures=[]
    for repeat in range(1,args.repeats+1):
        for name,goal in transitions:
            stem=f'{args.prefix}-{name}-{repeat}'
            csv_path=MOVEIT_OUTPUT/(stem+'.csv')
            if csv_path.exists() or csv_path.with_suffix('.json').exists():
                raise RuntimeError('Refusing to overwrite evidence: '+str(csv_path))
            # The Unity recorder writes under arm-controller/qualification; moved below.
            unity('arm_test_record','--name',stem)
            print('START '+stem,flush=True)
            container_result='/workspaces/mm-motion-planning/docs/experiments/moveit-arm/qualification/'+stem+'.reconfigure.json'
            cmd=['docker','exec',args.container,'bash','-lc',
                 'source "$ROS_WS/install/setup.bash" && exec ros2 run mobile_manipulator_manipulation reconfigure_panel "$@"',
                 'reconfigure_panel',*goal,'--output',container_result]
            try:
                # The hold after the action shows settling under the 3 kg panel.
                result=subprocess.run(cmd,text=True,capture_output=True,timeout=120)
                time.sleep(2.0)
            finally:
                unity('arm_test_end')
                (OUTPUT/(stem+'.csv')).replace(csv_path)
            reconfigure=json.loads((MOVEIT_OUTPUT/(stem+'.reconfigure.json')).read_text())
            # The analyzer's action record: status 4 (succeeded) and the measured hold error.
            csv_path.with_suffix('.json').write_text(json.dumps({
                'status':4 if reconfigure['error_code']=='SUCCESS' else 6,
                'error_code':0 if reconfigure['error_code']=='SUCCESS' else 1,
                'hold_max_error':[reconfigure['hold_error_rad']],
                'disturbance':'none','source':'reconfigure_panel'},indent=2)+'\n')
            physical=analyze(csv_path)
            print(json.dumps({'reconfigure':reconfigure['error_code'],'message':reconfigure['message'],
                              'trajectory_duration_s':reconfigure['trajectory_duration_s'],
                              'planning_time_s':reconfigure['planning_time_s'],
                              **{k:physical[k] for k in ('max_path_error_rad','max_hold_error_rad',
                                 'min_panel_bottom_m','max_base_tilt_degrees','passed')}}),flush=True)
            if result.returncode or not physical['passed']:
                failures.append(stem)
                print('FAIL '+stem+' '+json.dumps(physical['checks']),flush=True)
            else:
                print('PASS '+stem,flush=True)
    if failures:
        raise SystemExit('MoveIt transitions failed acceptance: '+', '.join(failures))


if __name__=='__main__':
    main()
