import argparse
import json
import shutil
import struct
import sys
from pathlib import Path

import bpy


DEFAULT_RATIOS = {
    'body_link.STL': 0.16,
    'car_link.STL': 0.18,
    'head_yaw_link.STL': 0.22,
    'llink2.STL': 0.20,
    'rlink2.STL': 0.20,
    'llink4.STL': 0.22,
    'rlink4.STL': 0.22,
    'llink1.STL': 0.22,
    'rlink1.STL': 0.22,
    'llink3.STL': 0.25,
    'rlink3.STL': 0.25,
    'llink5.STL': 0.25,
    'rlink5.STL': 0.25,
    'llink6.STL': 0.28,
    'rlink6.STL': 0.28,
    'base_link.STL': 0.30,
    'lidar_link.STL': 0.35,
    'head_pitch_link.STL': 0.35,
    'ankle_link.STL': 0.35,
    'knee_link.STL': 0.35,
    'llink7.STL': 0.45,
    'rlink7.STL': 0.45,
    'left_wheel_link.STL': 0.75,
    'right_wheel_link.STL': 0.75,
    'head_yaw.STL': 0.80,
    'hip_pitch_link.STL': 0.90,
}


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-dir', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--report-json', default='')
    parser.add_argument('--uniform-ratio', type=float, default=0.0)
    parser.add_argument('--ratio-overrides-json', default='')
    parser.add_argument('--component-face-thresholds-json', default='')
    return parser.parse_args(argv)


def stl_triangle_count(path):
    with path.open('rb') as handle:
        header = handle.read(84)
    if len(header) != 84:
        raise RuntimeError(f'Unexpected STL header length for {path}')
    return struct.unpack('<I', header[80:84])[0]


def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for data_collection in (
        bpy.data.meshes,
        bpy.data.materials,
        bpy.data.images,
        bpy.data.objects,
        bpy.data.collections,
    ):
        for block in list(data_collection):
            if block.users == 0:
                data_collection.remove(block)


def import_stl(path):
    try:
        bpy.ops.wm.stl_import(filepath=str(path))
    except Exception:
        bpy.ops.import_mesh.stl(filepath=str(path))
    return [obj for obj in bpy.context.selected_objects if obj.type == 'MESH']


def export_stl(path, objects):
    bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    try:
        bpy.ops.wm.stl_export(
            filepath=str(path),
            export_selected_objects=True,
            ascii_format=False,
        )
    except Exception:
        bpy.ops.export_mesh.stl(filepath=str(path), use_selection=True, ascii=False)


def decimate_objects(objects, ratio):
    for obj in objects:
        bpy.context.view_layer.objects.active = obj
        modifier = obj.modifiers.new(name='BalancedDecimate', type='DECIMATE')
        modifier.decimate_type = 'COLLAPSE'
        modifier.ratio = ratio
        if hasattr(modifier, 'use_collapse_triangulate'):
            modifier.use_collapse_triangulate = True
        bpy.ops.object.modifier_apply(modifier=modifier.name)


def split_loose_parts(objects):
    separated = []
    for obj in objects:
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.separate(type='LOOSE')
        bpy.ops.object.mode_set(mode='OBJECT')
        separated.extend(mesh for mesh in bpy.context.selected_objects if mesh.type == 'MESH')
    return separated


def filter_objects_by_face_threshold(objects, min_faces):
    if not min_faces:
        return objects, 0
    loose_objects = split_loose_parts(objects)
    kept = []
    removed_count = 0
    for obj in loose_objects:
        if len(obj.data.polygons) >= min_faces:
            kept.append(obj)
            continue
        bpy.data.objects.remove(obj, do_unlink=True)
        removed_count += 1
    if not kept:
        raise RuntimeError(f'All mesh components were filtered out by min_faces={min_faces}.')
    return kept, removed_count


def process_mesh(source_path, output_path, ratio, component_face_threshold):
    clear_scene()
    objects = import_stl(source_path)
    if not objects:
        raise RuntimeError(f'No mesh objects imported from {source_path}')
    triangles_before = sum(len(obj.data.polygons) for obj in objects)
    objects, removed_components = filter_objects_by_face_threshold(objects, component_face_threshold)
    triangles_after_filter = sum(len(obj.data.polygons) for obj in objects)
    decimate_objects(objects, ratio)
    triangles_after = sum(len(obj.data.polygons) for obj in objects)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    export_stl(output_path, objects)
    return triangles_before, triangles_after_filter, triangles_after, removed_components


def main(argv):
    args = parse_args(argv)
    source_dir = Path(args.source_dir).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    uniform_ratio = float(args.uniform_ratio) if args.uniform_ratio else 0.0
    if uniform_ratio and not (0.0 < uniform_ratio <= 1.0):
        raise ValueError('--uniform-ratio must be in the interval (0, 1].')
    ratio_overrides = {}
    if args.ratio_overrides_json:
        overrides_path = Path(args.ratio_overrides_json).resolve()
        loaded = json.loads(overrides_path.read_text(encoding='utf-8'))
        if not isinstance(loaded, dict):
            raise ValueError('--ratio-overrides-json must point to a JSON object.')
        for key, value in loaded.items():
            ratio = float(value)
            if not (0.0 < ratio <= 1.0):
                raise ValueError(f'Invalid ratio override for {key}: {value}')
            ratio_overrides[str(key)] = ratio
    component_face_thresholds = {}
    if args.component_face_thresholds_json:
        thresholds_path = Path(args.component_face_thresholds_json).resolve()
        loaded = json.loads(thresholds_path.read_text(encoding='utf-8'))
        if not isinstance(loaded, dict):
            raise ValueError('--component-face-thresholds-json must point to a JSON object.')
        for key, value in loaded.items():
            threshold = int(value)
            if threshold < 0:
                raise ValueError(f'Invalid component face threshold for {key}: {value}')
            component_face_thresholds[str(key)] = threshold

    report = []
    total_before_bytes = 0
    total_after_bytes = 0
    total_before_tris = 0
    total_after_tris = 0

    for source_path in sorted(source_dir.glob('*.STL')):
        ratio = ratio_overrides.get(source_path.name, uniform_ratio or DEFAULT_RATIOS.get(source_path.name, 0.35))
        component_face_threshold = component_face_thresholds.get(source_path.name, 0)
        output_path = output_dir / source_path.name
        before_tris_file = stl_triangle_count(source_path)
        before_tris_scene, filtered_tris_scene, after_tris_scene, removed_components = process_mesh(
            source_path,
            output_path,
            ratio,
            component_face_threshold,
        )
        after_tris_file = stl_triangle_count(output_path)
        before_bytes = source_path.stat().st_size
        after_bytes = output_path.stat().st_size
        total_before_bytes += before_bytes
        total_after_bytes += after_bytes
        total_before_tris += before_tris_file
        total_after_tris += after_tris_file
        report.append({
            'name': source_path.name,
            'ratio': ratio,
            'before_bytes': before_bytes,
            'after_bytes': after_bytes,
            'before_triangles_file': before_tris_file,
            'before_triangles_scene': before_tris_scene,
            'after_filter_triangles_scene': filtered_tris_scene,
            'after_triangles_scene': after_tris_scene,
            'after_triangles_file': after_tris_file,
            'component_face_threshold': component_face_threshold,
            'removed_components': removed_components,
        })

    summary = {
        'source_dir': str(source_dir),
        'output_dir': str(output_dir),
        'total_before_bytes': total_before_bytes,
        'total_after_bytes': total_after_bytes,
        'total_before_triangles': total_before_tris,
        'total_after_triangles': total_after_tris,
        'meshes': report,
    }

    if args.report_json:
        report_path = Path(args.report_json).resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')

    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main(sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else sys.argv[1:])
