"""Reproduce conservative actuator starting estimates from the authoritative robot model."""
import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def calculate():
    robot=ET.parse(ROOT/'ros2_ws/src/mobile_manipulator_description/urdf/mobile_manipulator.urdf').getroot()
    joints=[j for j in robot.findall('joint') if j.get('type')=='revolute']
    z=[]; at=0
    for joint in joints:
        at+=float(joint.find('origin').get('xyz').split()[2]); z.append(at)
    result={'classification':'Model masses, inertias and limits are simulation assumptions. Bounds use neutral serial distances, not an exact coupled dynamics model.',
            'payload':{'mass_kg':3,'com_tool_unity_m':[0,.035,0],'inertia_kg_m2':[.3604,.72,.3604]},'joints':[]}
    for i,joint in enumerate(joints):
        mass_distance=0; inertia=0
        for k in range(i,len(joints)):
            link=robot.find("link[@name='%s']"%joints[k].find('child').get('link'))
            inertial=link.find('inertial')
            mass=float(inertial.find('mass').get('value'))
            distance=z[k]-z[i]+float(inertial.find('origin').get('xyz').split()[2])
            mass_distance+=mass*distance
            inertia+=mass*distance**2+max(float(inertial.find('inertia').get(a)) for a in ['ixx','iyy','izz'])
        distance=z[-1]+.09+.035-z[i]
        inertial=robot.find("link[@name='%s']/inertial"%joint.find('child').get('link'))
        kp=[2000,6000,3500,500,650,300][i]; kd=[240,360,180,40,35,28][i]
        result['joints'].append({'name':joint.get('name'),'unity_body':joint.find('child').get('link'),
            'axis_ros':joint.find('axis').get('xyz'),'sign':1,'limits':dict(joint.find('limit').attrib),
            'link_mass_kg':float(inertial.find('mass').get('value')),'com_ros_m':inertial.find('origin').get('xyz'),
            'inertia_ros':dict(inertial.find('inertia').attrib),
            'unloaded_lever_bound_Nm':9.81*mass_distance,'loaded_lever_bound_Nm':9.81*(mass_distance+3*distance),
            'effective_inertia_bound_kg_m2':inertia+3*distance**2+.72,'stiffness':kp,'damping':kd,
            'loaded_static_error_bound_rad':9.81*(mass_distance+3*distance)/kp})
    return result

if __name__=='__main__':
    output=ROOT/'docs/experiments/arm-controller/model-calculations.json'
    output.write_text(json.dumps(calculate(),indent=2)+'\n')
    print(output.relative_to(ROOT))
