"""Capture the actual source/build inputs; never patch the external PX4 tree."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git_bytes(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args])


def source_root():
    return Path(git_bytes(Path(__file__).resolve().parent, 'rev-parse', '--show-toplevel').decode().strip())


def snapshot_tree(root, output, label):
    root=Path(root).resolve()
    (output/(label+'.diff')).write_bytes(git_bytes(root, 'diff', '--binary', 'HEAD'))
    paths=git_bytes(root, 'ls-files', '-z', '--cached', '--others', '--exclude-standard').decode().split('\0')
    hashes={}
    for name in sorted(set(paths)-{''}):
        path=root/name
        if path.is_file():
            hashes[name]=digest(path)
    # New runtime files are not represented by git diff; preserve them separately.
    untracked=git_bytes(root, 'ls-files', '-z', '--others', '--exclude-standard').decode().split('\0')
    for name in untracked:
        if name and name.split('/')[0] in ('src', 'scripts', 'config') and (root/name).is_file():
            target=output/(label+'_untracked')/name
            target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(root/name,target)
    return dict(root=str(root),commit=git_bytes(root,'rev-parse','HEAD').decode().strip(),
                status=git_bytes(root,'status','--porcelain').decode(),sha256=hashes)


def capture(output, px4=None):
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    root=source_root();output=Path(output)
    report={'source':snapshot_tree(root,output,'source'), 'runtime_module':str(Path(__file__).resolve()),
            'packages':{},'build_caches':{}}
    for name in ('dragonfly_flight','dragonfly_fcu_bridge','dragonfly_control','dragonfly_sim'):
        prefix=Path(get_package_prefix(name));share=Path(get_package_share_directory(name))
        if not prefix.resolve().is_relative_to(root):
            raise RuntimeError(f'{name} was built outside the selected project: {prefix}')
        report['packages'][name]={'prefix':str(prefix),'share':str(share),'resolved_share':str(share.resolve())}
        report['packages'][name]['shared_libraries_sha256']={str(lib):digest(lib) for lib in (prefix/'lib').glob('*.so')}
    for cache in (root/'build-sim').glob('*/CMakeCache.txt'):
        report['build_caches'][str(cache)]=[line for line in cache.read_text().splitlines()
            if line.startswith(('CMAKE_HOME_DIRECTORY:', 'CMAKE_CACHEFILE_DIR:'))]
    if px4 is not None:
        px4=Path(px4).resolve()
        report['px4']=snapshot_tree(px4,output,'px4_source')
        report['px4']['binary_sha256']=digest(px4/'build/px4_sitl_default/bin/px4')
        manifest=json.loads((root/'src/dragonfly_flight/config/px4_vector/native_manifest.json').read_text())
        mismatches=[entry['path'] for entry in manifest
                    if digest(px4/entry['path']) != entry['sha256']]
        report['px4']['manifest_mismatches']=mismatches
        (output/'provenance.json').write_text(json.dumps(report,indent=2))
        if mismatches:
            raise RuntimeError('External PX4 does not match native_manifest.json: '+str(mismatches))
    report['assets_sha256']={name:digest(output/name) for name in
        ('model.json','robot.urdf','world.sdf','flight_config.yaml','run.json')}
    (output/'provenance.json').write_text(json.dumps(report,indent=2))
