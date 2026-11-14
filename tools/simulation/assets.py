"""Startup byte binding for explicit, movable MJCF dependency closures."""
import hashlib
import importlib.metadata
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def closure(model):
    """Attest a deliberately restricted, unambiguous MJCF file-resolution subset.

    Single-file models support root meshdir/texturedir. Includes support only
    same-directory XML with no compiler path settings anywhere. Global compiler
    inheritance, assetdir/strippath, nested models and alternate file attributes
    are rejected, never approximated with a different dependency's bytes.
    """
    model=Path(model).resolve()
    root=model.parent
    xmls={}
    references=[]
    compiler=None
    def contained(path):
        path=path.resolve()
        if not path.is_relative_to(root):
            raise ValueError('asset dependency escapes model directory')
        return path
    def unsupported(reason):
        raise ValueError('unsupported asset resolution: '+reason)
    def inspect(path):
        nonlocal compiler
        path=contained(path)
        if path in xmls:return
        if path.parent!=root:unsupported('includes must share main XML directory')
        tree=ET.parse(path)
        xmls[path]=tree
        for node in tree.iter():
            if node.tag=='compiler':
                if path!=model or node not in list(tree.getroot()):
                    unsupported('included or nested compiler settings')
                if compiler is not None:unsupported('multiple compiler elements')
                compiler=node
                if 'assetdir' in node.attrib or 'strippath' in node.attrib:
                    unsupported('compiler assetdir/strippath')
            file_attributes=[key for key in node.attrib if key.startswith('file')]
            if not file_attributes:continue
            if file_attributes!=['file'] or node.tag not in ('include','mesh','texture'):
                unsupported('file attributes on '+node.tag)
            if node.tag=='include':
                inspect(root/node.get('file'))
            else:
                references.append((node.tag,node.get('file')))
    inspect(model)
    if len(xmls)>1 and compiler is not None and any(
            key in compiler.attrib for key in ('meshdir','texturedir')):
        unsupported('compiler directory inheritance across includes')
    meshdir=compiler.get('meshdir','') if compiler is not None else ''
    texturedir=compiler.get('texturedir','') if compiler is not None else ''
    paths=set(xmls)
    for kind,name in references:
        paths.add(contained(root/(meshdir if kind=='mesh' else texturedir)/name))
    files=dict(sorted((path.relative_to(root).as_posix(),sha256(path)) for path in paths))
    digest=hashlib.sha256(json.dumps(files,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    return dict(digest=digest,files=files)


def engine_identity():
    versions={name:importlib.metadata.version(name) for name in ('mujoco','mujoco-warp','warp-lang')}
    expected={'mujoco':'3.9.0','mujoco-warp':'3.9.0','warp-lang':'1.15.0'}
    if versions!=expected:raise ValueError('unvalidated simulator versions: '+str(versions))
    binding=dict(versions=versions,adapter_sha256=sha256(Path(__file__).with_name('mujoco_warp_engine.py')))
    return hashlib.sha256(json.dumps(binding,sort_keys=True).encode()).hexdigest(),binding


def admit_initial_geometry(config,manifest):
    """Only the converter's admitted flat-floor stance is supported initially.

    A finite arbitrary base override is not geometric admission. A different
    stance needs explicit preparation/kinematic admission, never a hidden lift.
    """
    expected=manifest.get('initial_geometry')
    keys=('initial_base_position','initial_base_quaternion','initial_joint_position')
    if not expected or any(config.get(k)!=expected.get(k) for k in keys):
        raise ValueError('initial geometry differs from admitted prepared stance; re-prepare explicitly')


def verify_binding(normalized):
    engine_digest,engine=engine_identity()
    if normalized['engine_digest']!=engine_digest:raise ValueError('engine digest mismatch')
    model=Path(normalized['engine_config']['model_path'])
    manifest=json.loads(model.with_suffix('.manifest.json').read_text())
    actual=closure(model)
    if 'initial_base_position' in normalized['engine_config']:
        admit_initial_geometry(normalized['engine_config'],manifest)
    if actual!=manifest['prepared_closure'] or actual['digest']!=normalized['asset_digest']:
        raise ValueError('asset closure digest mismatch')
    if sha256(normalized['policy_path'])!=manifest['policy_sha256']:
        raise ValueError('policy digest mismatch')
    return dict(engine=engine,asset=manifest,policy_sha256=manifest['policy_sha256'])


def binary_stl(data):
    """Preserve triangle coordinates/normals; convert ASCII STL to binary storage."""
    import re
    import struct
    if len(data)>=84 and 84+50*struct.unpack_from('<I',data,80)[0]==len(data):return data
    text=data.decode('ascii')
    triangles=re.findall(r'facet\s+normal\s+([^\n]+)\s+outer\s+loop\s+(.*?)endloop\s+endfacet',text,re.S)
    if not triangles:raise ValueError('unsupported STL encoding')
    output=bytearray(b'IRMV explicit ASCII to binary STL'.ljust(80,b'\0'))
    output.extend(struct.pack('<I',len(triangles)))
    for normal,vertices in triangles:
        values=[float(v) for v in normal.split()]
        for row in re.findall(r'vertex\s+([^\n]+)',vertices):values.extend(float(v) for v in row.split())
        if len(values)!=12:raise ValueError('invalid STL triangle')
        output.extend(struct.pack('<12fH',*values,0))
    return bytes(output)


def stl_obj(data):
    """Lossless binary32 vertex conversion; avoids MuJoCo's 200k STL face cap."""
    import struct
    data=binary_stl(data)
    vertices={};faces=[]
    for start in range(84,len(data),50):
        values=struct.unpack_from('<12fH',data,start)
        face=[]
        for offset in (3,6,9):
            vertex=values[offset:offset+3]
            if vertex not in vertices:vertices[vertex]=len(vertices)+1
            face.append(vertices[vertex])
        faces.append(face)
    lines=['v '+' '.join(format(x,'.9g') for x in v) for v in vertices]
    lines.extend('f '+' '.join(map(str,f)) for f in faces)
    return '\n'.join(lines)+'\n'
