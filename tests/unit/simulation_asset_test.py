"""Dependency closure is byte-attested, never inferred from the URDF alone."""
from pathlib import Path
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
class AssetTest(unittest.TestCase):
    def test_dependency_bytes_and_paths(self):
        from simulation.assets import closure
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            (p/'model.xml').write_text('<mujoco><include file="part.xml"/></mujoco>')
            (p/'part.xml').write_text('<mujoco><asset><mesh file="a.stl"/></asset></mujoco>')
            (p/'a.stl').write_bytes(b'first')
            first=closure(p/'model.xml')
            self.assertEqual(len(first['files']),3)
            (p/'a.stl').write_bytes(b'second')
            self.assertNotEqual(first['digest'],closure(p/'model.xml')['digest'])
            (p/'part.xml').write_text('<mujoco><include file="../outside.xml"/></mujoco>')
            with self.assertRaises(ValueError):closure(p/'model.xml')
    def test_prepared_geometry_admission_refuses_manual_pose_override(self):
        from simulation.assets import admit_initial_geometry
        expected=dict(initial_base_position=[0,0,.8],initial_base_quaternion=[1,0,0,0],
                      initial_joint_position=[-.3,.6,-.3])
        manifest=dict(initial_geometry=expected)
        admit_initial_geometry(expected,manifest)
        for key in expected:
            changed={k:list(v) for k,v in expected.items()}
            changed[key][0]+=.1
            with self.assertRaisesRegex(ValueError,'initial geometry'):
                admit_initial_geometry(changed,manifest)
        with self.assertRaisesRegex(ValueError,'initial geometry'):
            admit_initial_geometry(expected,{})

    def test_ascii_mesh_conversion_preserves_vertices(self):
        from simulation.assets import binary_stl
        import struct
        text=b'solid test\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid test\n'
        data=binary_stl(text)
        self.assertEqual(len(data),134)
        self.assertEqual(struct.unpack_from('<I',data,80)[0],1)
        self.assertEqual(struct.unpack_from('<12fH',data,84)[3:12],(0,0,0,1,0,0,0,1,0))
    def test_large_stl_to_obj_preserves_triangle(self):
        from simulation.assets import stl_obj
        import struct
        data=b'0'*80+struct.pack('<I',1)+struct.pack('<12fH',0,0,1,0,0,0,1,0,0,0,1,0,0)
        self.assertEqual(stl_obj(data).splitlines(),['v 0 0 0','v 1 0 0','v 0 1 0','f 1 2 3'])
    def test_binding_rejects_changed_asset_policy_and_engine(self):
        import json
        from unittest.mock import patch
        from simulation.assets import closure,sha256,verify_binding
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);model=p/'robot.xml';policy=p/'policy.onnx'
            model.write_text('<mujoco/>');policy.write_bytes(b'policy bytes')
            manifest=dict(prepared_closure=closure(model),policy_sha256=sha256(policy))
            model.with_suffix('.manifest.json').write_text(json.dumps(manifest))
            n=dict(engine_digest='e'*64,engine_config=dict(model_path=str(model)),
                   asset_digest=manifest['prepared_closure']['digest'],policy_path=str(policy))
            with patch('simulation.assets.engine_identity',return_value=('e'*64,{})):
                verify_binding(n)
                n['engine_digest']='a'*64
                with self.assertRaisesRegex(ValueError,'engine digest'):verify_binding(n)
                n['engine_digest']='e'*64;policy.write_bytes(b'changed policy')
                with self.assertRaisesRegex(ValueError,'policy digest'):verify_binding(n)
                policy.write_bytes(b'policy bytes');model.write_text('<mujoco model="changed"/>')
                with self.assertRaisesRegex(ValueError,'asset closure'):verify_binding(n)
    def test_assetdir_decoy_never_attested_even_after_loaded_mesh_mutation(self):
        import hashlib,json
        from unittest.mock import patch
        from simulation.assets import closure,sha256,verify_binding
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'assets').mkdir()
            model=p/'robot.xml';policy=p/'policy.onnx'
            model.write_text('<mujoco><compiler assetdir="assets"/><asset><mesh file="shape.obj"/></asset></mujoco>')
            (p/'shape.obj').write_bytes(b'decoy');actual=p/'assets/shape.obj'
            actual.write_bytes(b'actual loaded mesh');policy.write_bytes(b'policy')
            # Exact vulnerable legacy attestation; do not call fixed closure to build it.
            files={name:sha256(p/name) for name in ('robot.xml','shape.obj')}
            digest=hashlib.sha256(json.dumps(files,sort_keys=True,separators=(',',':')).encode()).hexdigest()
            model.with_suffix('.manifest.json').write_text(json.dumps(dict(
                prepared_closure=dict(files=files,digest=digest),policy_sha256=sha256(policy))))
            n=dict(engine_digest='e'*64,engine_config=dict(model_path=str(model)),
                   asset_digest=digest,policy_path=str(policy))
            for changed in (False,True):
                if changed:actual.write_bytes(b'CHANGED actual loaded mesh')
                with self.subTest(changed=changed,entry='closure'):
                    with self.assertRaisesRegex(ValueError,'unsupported.*resolution'):closure(model)
                with self.subTest(changed=changed,entry='verify_binding'):
                    with patch('simulation.assets.engine_identity',return_value=('e'*64,{})):
                        with self.assertRaisesRegex(ValueError,'unsupported.*resolution'):verify_binding(n)

    def test_include_compiler_resolution_is_explicitly_fail_closed(self):
        from simulation.assets import closure
        cases=[
            ('<compiler meshdir="assets"/>',''),
            ('<compiler texturedir="assets"/>',''),
            ('','<compiler meshdir="assets"/>'),
            ('','<compiler texturedir="assets"/>'),
            ('','<compiler assetdir="assets"/>'),
            ('<compiler strippath="true"/>',''),
        ]
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'assets').mkdir()
            (p/'shape.obj').write_bytes(b'decoy');(p/'assets/shape.obj').write_bytes(b'actual')
            for top,part in cases:
                (p/'robot.xml').write_text('<mujoco>'+top+'<include file="part.xml"/></mujoco>')
                (p/'part.xml').write_text('<mujoco>'+part+'<asset><mesh file="shape.obj"/></asset></mujoco>')
                with self.subTest(top=top,part=part):
                    with self.assertRaisesRegex(ValueError,'unsupported.*resolution'):closure(p/'robot.xml')
            (p/'sub').mkdir();(p/'sub/part.xml').write_text('<mujoco><asset><mesh file="shape.obj"/></asset></mujoco>')
            (p/'sub/shape.obj').write_bytes(b'another decoy')
            (p/'robot.xml').write_text('<mujoco><include file="sub/part.xml"/></mujoco>')
            with self.assertRaisesRegex(ValueError,'unsupported.*resolution'):closure(p/'robot.xml')

    def test_other_dependency_resolution_not_silently_ignored(self):
        from simulation.assets import closure
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            (p/'child.xml').write_text('<mujoco/>');(p/'face.png').write_bytes(b'face')
            for asset in ('<model file="child.xml"/>','<texture fileleft="face.png"/>'):
                (p/'robot.xml').write_text('<mujoco><asset>'+asset+'</asset></mujoco>')
                with self.subTest(asset=asset):
                    with self.assertRaisesRegex(ValueError,'unsupported.*resolution'):closure(p/'robot.xml')

    def test_single_file_meshdir_still_hashes_loaded_mesh(self):
        from simulation.assets import closure
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'assets').mkdir()
            (p/'shape.obj').write_bytes(b'decoy');actual=p/'assets/shape.obj';actual.write_bytes(b'first')
            (p/'robot.xml').write_text('<mujoco><compiler meshdir="assets"/><asset><mesh file="shape.obj"/></asset></mujoco>')
            first=closure(p/'robot.xml')
            self.assertEqual(set(first['files']),{'robot.xml','assets/shape.obj'})
            actual.write_bytes(b'changed')
            self.assertNotEqual(first['digest'],closure(p/'robot.xml')['digest'])

if __name__=='__main__':unittest.main()
