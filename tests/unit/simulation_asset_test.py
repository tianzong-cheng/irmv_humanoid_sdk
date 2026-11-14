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
if __name__=='__main__':unittest.main()
