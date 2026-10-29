#!/usr/bin/python3
"""Repair only identified catkin build/devel outputs; never follow symlinks."""
import argparse
import json
import os
from pathlib import Path
import stat


def repair_generated(workspace, uid, gid):
    workspace = Path(workspace)
    if uid <= 0 or gid < 0:
        raise ValueError('A non-root build user and valid group are required')
    roots = []
    for name, marker in (('build', 'CMakeCache.txt'), ('devel', '.catkin')):
        root = workspace/name
        if not root.exists() and not root.is_symlink():
            continue
        if root.is_symlink() or not root.is_dir():
            raise ValueError('Refusing non-directory or symlink output: '+str(root))
        marker_path = root/marker
        if marker_path.is_symlink() or not marker_path.is_file():
            raise ValueError('Not an identified catkin output: '+str(root))
        roots.append(root)
    changed = []
    # Validate both roots before modifying either. Bind-mount boundaries and
    # links cannot expand this operation to source/data or another filesystem.
    for root in roots:
        device = root.stat().st_dev
        candidates = [root]
        for directory, subdirs, files in os.walk(root, followlinks=False):
            subdirs[:] = [name for name in subdirs
                          if not (Path(directory)/name).is_symlink()
                          and (Path(directory)/name).stat().st_dev == device]
            candidates.extend(Path(directory)/name for name in subdirs+files)
        for path in candidates:
            info = path.lstat()
            if info.st_dev != device or not (stat.S_ISREG(info.st_mode) or stat.S_ISDIR(info.st_mode)):
                continue
            if stat.S_ISREG(info.st_mode) and info.st_nlink != 1:
                raise ValueError('Refusing multiply-linked generated file: '+str(path))
            wanted_mode = stat.S_IMODE(info.st_mode) | stat.S_IRUSR | stat.S_IWUSR
            if stat.S_ISDIR(info.st_mode):
                wanted_mode |= stat.S_IXUSR
            if (info.st_uid, info.st_gid, stat.S_IMODE(info.st_mode)) == (uid, gid, wanted_mode):
                continue
            descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
            try:
                opened = os.fstat(descriptor)
                if (opened.st_dev, opened.st_ino) != (info.st_dev, info.st_ino):
                    raise RuntimeError('Generated path changed during repair: '+str(path))
                os.fchown(descriptor, uid, gid)
                os.fchmod(descriptor, wanted_mode)
            finally:
                os.close(descriptor)
            changed.append(str(path))
    return changed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace', type=Path)
    parser.add_argument('--uid', type=int, required=True)
    parser.add_argument('--gid', type=int, required=True)
    args = parser.parse_args()
    changed = repair_generated(args.workspace, args.uid, args.gid)
    print(json.dumps({'repaired_generated_entries': len(changed), 'workspace': str(args.workspace)}))


if __name__ == '__main__':
    main()
